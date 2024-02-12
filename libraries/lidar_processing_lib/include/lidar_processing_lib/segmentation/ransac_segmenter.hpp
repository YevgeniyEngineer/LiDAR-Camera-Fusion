#ifndef LIDAR_PROCESSING_LIB__SEGMENTATION__RANSAC_SEGMENTER_HPP
#define LIDAR_PROCESSING_LIB__SEGMENTATION__RANSAC_SEGMENTER_HPP

#include "i_segmenter.hpp"               // ISegmenter
#include <algorithm>                     // std::sort
#include <array>                         // std::array
#include <chrono>                        // std::chrono
#include <cmath>                         // M_PI
#include <random>                        // std::random_device, std::mt19937, std::uniform_int_distribution
#include <utilities_lib/math.hpp>        // constexprRound
#include <utilities_lib/thread_pool.hpp> // ThreadPool
#include <vector>                        // std::vector

namespace lidar_processing_lib::segmentation
{
class RansacSegmenter : public ISegmenter
{
  public:
    // Polar prid parameters
    // channel: azimuth partition
    // cell: radial partition
    static constexpr float CHANNEL_RESOLUTION_DEG = 1.0F;
    static constexpr float CELL_RESOLUTION_M = 1.5F;
    static constexpr float MAX_CELL_RADIUS = 300.0F;

    // Constants
    static constexpr float DEG_TO_RAD = M_PIf32 / 180.0F;
    static constexpr float RAD_TO_DEG = 180.0F / M_PIf32;

    static constexpr float CHANNEL_RESOLUTION_RAD = CHANNEL_RESOLUTION_DEG * DEG_TO_RAD;

    static constexpr auto NUMBER_OF_CHANNELS =
        static_cast<std::uint32_t>(utilities_lib::constexprRound(360.0F / CHANNEL_RESOLUTION_DEG));

    static constexpr auto NUMBER_OF_CELLS =
        static_cast<std::uint32_t>(utilities_lib::constexprRound(MAX_CELL_RADIUS / CELL_RESOLUTION_M));

    explicit RansacSegmenter(float height_offset, float orthogonal_distance_threshold = 0.1F,
                             std::uint32_t number_of_iterations = 100U, float max_plane_inclination_deg = 25.0F,
                             float consideration_radius = 20.0F, float consideration_height = 0.8F,
                             float classification_radius = 60.0F);

    ~RansacSegmenter();

    void run(const pcl::PointCloud<pcl::PointXYZ> &cloud, std::vector<SegmentationLabel> &labels) override;
    void run(const pcl::PointCloud<pcl::PointXYZI> &cloud, std::vector<SegmentationLabel> &labels) override;

  private:
    float height_offset_;
    float orthogonal_distance_threshold_;
    std::uint32_t number_of_iterations_;
    float max_plane_inclination_deg_;
    float consideration_radius_;
    float consideration_height_;
    float classification_radius_;

    std::vector<pcl::PointXYZ> processing_points_;

    struct PointXYZIL final
    {
        float x;
        float y;
        float z;
        std::uint32_t index;
        SegmentationLabel label;

        PointXYZIL() = default;

        PointXYZIL(float x, float y, float z, std::uint32_t index, SegmentationLabel label)
            : x(x), y(y), z(z), index(index), label(label)
        {
        }
    };

    std::array<std::array<std::vector<PointXYZIL>, NUMBER_OF_CELLS>, NUMBER_OF_CHANNELS> polar_grid_;

    template <typename PointT>
    void embedPointCloudIntoPolarGrid(const pcl::PointCloud<PointT> &cloud,
                                      const std::vector<SegmentationLabel> &labels);

    template <typename PointT>
    void segmentRansac(const pcl::PointCloud<PointT> &cloud, std::vector<SegmentationLabel> &labels);

    template <typename PointT>
    void segment(const pcl::PointCloud<PointT> &cloud, std::vector<SegmentationLabel> &labels);

    void refineClassificationThroughPolarGridTraversal(std::vector<SegmentationLabel> &labels);
};

template <typename PointT>
void RansacSegmenter::embedPointCloudIntoPolarGrid(const pcl::PointCloud<PointT> &cloud,
                                                   const std::vector<SegmentationLabel> &labels)
{
    // Clear old points
    for (auto &channel : polar_grid_)
    {
        for (auto &cell : channel)
        {
            cell.clear();
        }
    }

    // Embed points into each channel
    for (std::uint32_t i = 0U; i < cloud.points.size(); ++i)
    {
        if (labels[i] != SegmentationLabel::UNKNOWN)
        {
            const auto &point = cloud.points[i];

            // Calculate distance
            const float distance = std::sqrt((point.x * point.x) + (point.y * point.y));

            if (distance < MAX_CELL_RADIUS)
            {
                // Convert azimuth angle to degrees and shift range to [0, 360) OR [0, 2 * PI]
                float azimuth_rad = utilities_lib::atan2Approx(point.y, point.x);

                // Adjust to range [0, 2 * pi]
                if (azimuth_rad < 0)
                {
                    azimuth_rad += static_cast<float>(2.0 * M_PI);
                }

                // Determine channel index using floor division
                const std::uint32_t channel_index = std::min(
                    static_cast<std::uint32_t>(azimuth_rad / CHANNEL_RESOLUTION_RAD), (NUMBER_OF_CHANNELS - 1U));

                // Determine bin index
                const std::uint32_t cell_index =
                    std::min(static_cast<std::uint32_t>(distance / CELL_RESOLUTION_M), (NUMBER_OF_CELLS - 1U));

                // Embed point into polar grid channel and cell
                polar_grid_[channel_index][cell_index].emplace_back(point.x, point.y, point.z, i, labels[i]);
            }
        }
    }

    // Sort points in increasing radial order, in each channel in each cell
    for (auto &channel : polar_grid_)
    {
        for (auto &cell : channel)
        {
            if (!cell.empty())
            {
                std::sort(cell.begin(), cell.end(), [](const auto &p1, const auto &p2) noexcept {
                    const float dr1 = (p1.x * p1.x) + (p1.y * p1.y);
                    const float dr2 = (p2.x * p2.x) + (p2.y * p2.y);
                    return dr1 < dr2;
                });
            }
        }
    }
}

void RansacSegmenter::refineClassificationThroughPolarGridTraversal(std::vector<SegmentationLabel> &labels)
{
    const PointXYZIL pivot_point{0.0F, 0.0F, -height_offset_, 0, SegmentationLabel::GROUND};
    std::uint32_t points_reclassified = 0U;

    // Traverse grid in the forward direction
    std::uint32_t reclassified_points_forward_traversal = 0U;
    for (auto &channel : polar_grid_)
    {
        const auto *last_known_ground_point = &pivot_point;

        // Traverse each cell and refine
        for (auto &cell : channel)
        {
            // Traverse each point within cell
            for (auto &point : cell)
            {
                if (point.label == SegmentationLabel::GROUND)
                {
                    last_known_ground_point = &point;
                }
                // Check if can be re-classified as ground
                else
                {
                    const float dx = point.x - last_known_ground_point->x;
                    const float dy = point.y - last_known_ground_point->y;
                    const float dz = point.z - last_known_ground_point->z;
                    const float dr = std::sqrt((dx * dx) + (dy * dy));
                    const float gradient = dz / dr;

                    if ((std::fabs(gradient) < 0.2F) && (dr < 8.0F))
                    {
                        point.label = SegmentationLabel::GROUND;
                        last_known_ground_point = &point;
                        labels[point.index] = SegmentationLabel::GROUND;
                        ++reclassified_points_forward_traversal;
                    }
                }
            }
        }
    }

    // Traverse grid in the horizontal direction
    const auto *last_known_ground_point = &pivot_point;
    std::uint32_t reclassified_points_horizontal_traversal = 0U;
    for (std::uint32_t cell_index = 0U; cell_index < NUMBER_OF_CELLS; ++cell_index)
    {
        for (std::uint32_t channel_index = 0U; channel_index < NUMBER_OF_CHANNELS; ++channel_index)
        {
            // For each point
            for (auto &point : polar_grid_[channel_index][cell_index])
            {
                if (point.label == SegmentationLabel::GROUND)
                {
                    last_known_ground_point = &point;
                }
                // Check if can be re-classified as ground
                else
                {
                    const float dx = point.x - last_known_ground_point->x;
                    const float dy = point.y - last_known_ground_point->y;
                    const float dz = point.z - last_known_ground_point->z;
                    const float dr = std::sqrt((dx * dx) + (dy * dy));
                    const float gradient = dz / dr;

                    if ((std::fabs(gradient) < 0.10F) && (dr < 1.0F))
                    {
                        point.label = SegmentationLabel::GROUND;
                        last_known_ground_point = &point;
                        labels[point.index] = SegmentationLabel::GROUND;
                        ++reclassified_points_horizontal_traversal;
                    }
                }
            }
        }
    }

    // Last stage - per cell reclassification
    std::uint32_t reclassified_points_cell_smoothing = 0U;
    for (auto &channel : polar_grid_)
    {
        for (auto &cell : channel)
        {
            // Find mean ground point within current cells
            float z_mean = 0.0;
            std::uint32_t number_of_ground_points_within_cell = 0U;
            for (const auto &point : cell)
            {
                if (point.label == SegmentationLabel::GROUND)
                {
                    z_mean += point.z;
                    ++number_of_ground_points_within_cell;
                }
            }
            if (number_of_ground_points_within_cell > 0U)
            {
                z_mean /= number_of_ground_points_within_cell;

                // Attempt to smooth out non-ground classifications
                for (auto &point : cell)
                {
                    if (point.label == SegmentationLabel::OBSTACLE)
                    {
                        if (std::fabs(point.z - z_mean) < 0.15F)
                        {
                            point.label = SegmentationLabel::GROUND;
                            ++reclassified_points_cell_smoothing;
                        }
                    }
                }
            }
        }
    }

    points_reclassified = reclassified_points_forward_traversal + reclassified_points_horizontal_traversal +
                          reclassified_points_cell_smoothing;

    std::cerr << "Reclassified points forward traversal: " << reclassified_points_forward_traversal << std::endl;
    std::cerr << "Reclassified points horizontal traversal: " << reclassified_points_horizontal_traversal << std::endl;
    std::cerr << "Reclassified points cell smoothing: " << reclassified_points_cell_smoothing << std::endl;
    std::cerr << "Reclassified points: " << points_reclassified << std::endl;
}

template <typename PointT>
void RansacSegmenter::segmentRansac(const pcl::PointCloud<PointT> &cloud, std::vector<SegmentationLabel> &labels)
{
    // Copy points from the cloud to the processing points
    processing_points_.clear();
    const float consideration_radius_squared = consideration_radius_ * consideration_radius_;
    for (const auto &point : cloud.points)
    {
        // Point shifted to ground level
        if ((std::fabs(point.z + height_offset_) <= consideration_height_) &&
            ((point.x * point.x + point.y * point.y) < consideration_radius_squared))
        {
            processing_points_.emplace_back(point.x, point.y, point.z);
        }
    }

    // Set pivot constraints
    const PointT point_1{0.0F, 0.0F, -height_offset_};
    const float max_plane_inclination_rad = max_plane_inclination_deg_ * M_PIf32 / 180.0F;
    const float max_plane_cosine_angle = std::cos(max_plane_inclination_rad);

    // Plane coefficients
    float a = 0.0F;
    float b = 0.0F;
    float c = 1.0F;
    float d = 0.0F;

    // For index generation
    std::mt19937 generator(42);
    std::uniform_int_distribution<std::uint32_t> distribution(0, processing_points_.size() - 1U);

    // RANSAC
    std::uint32_t best_inlier_count = 0U;
    for (std::uint32_t iteration = 0U; iteration < number_of_iterations_; ++iteration)
    {
        // Choose 2 random points
        const std::uint32_t point_2_index = distribution(generator);
        const auto &point_2 = processing_points_[point_2_index];

        std::uint32_t point_3_index = distribution(generator);
        while (point_2_index == point_3_index)
        {
            point_3_index = distribution(generator);
        }
        const auto &point_3 = processing_points_[point_3_index];

        // Calculate a plane defined by three points
        float normal_x =
            ((point_2.y - point_1.y) * (point_3.z - point_1.z)) - ((point_2.z - point_1.z) * (point_3.y - point_1.y));
        float normal_y =
            ((point_2.z - point_1.z) * (point_3.x - point_1.x)) - ((point_2.x - point_1.x) * (point_3.z - point_1.z));
        float normal_z =
            ((point_2.x - point_1.x) * (point_3.y - point_1.y)) - ((point_2.y - point_1.y) * (point_3.x - point_1.x));

        // Calculate normalization
        const float denominator = std::sqrt((normal_x * normal_x) + (normal_y * normal_y) + (normal_z * normal_z));

        // Check that denominator is not too small
        if (denominator < 1.0e-4F)
        {
            continue;
        }
        const float normalization = 1.0F / denominator;

        // Normalize plane coefficients
        normal_z *= normalization;

        // Constrain plane
        if (std::fabs(normal_z) < max_plane_cosine_angle)
        {
            continue;
        }

        normal_x *= normalization;
        normal_y *= normalization;

        const float plane_d = (normal_x * point_1.x) + (normal_y * point_1.y) + (normal_z * point_1.z);

        // Count inlier points
        std::uint32_t inlier_count = 0U;
        for (const auto &point : processing_points_)
        {
            const float orthogonal_distance =
                std::fabs((normal_x * point.x) + (normal_y * point.y) + (normal_z * point.z) - plane_d);

            if (orthogonal_distance < orthogonal_distance_threshold_)
            {
                ++inlier_count;
            }
        }

        // If the plane is best so far, update the plane coefficients
        if (inlier_count > best_inlier_count)
        {
            best_inlier_count = inlier_count;
            a = normal_x;
            b = normal_y;
            c = normal_z;
            d = plane_d;
        }
    }

    // Decide which points are GROUND and which points are NON-GROUND
    const float classification_radius_squared = classification_radius_ * classification_radius_;
    for (std::size_t i = 0U; i < cloud.points.size(); ++i)
    {
        const auto &point = cloud.points[i];

        if ((point.x * point.x + point.y * point.y) < classification_radius_squared)
        {
            const float orthogonal_distance = std::fabs((a * point.x) + (b * point.y) + (c * point.z) - d);

            if (orthogonal_distance < orthogonal_distance_threshold_)
            {
                labels[i] = SegmentationLabel::GROUND;
            }
            else
            {
                labels[i] = SegmentationLabel::OBSTACLE;
            }
        }
    }
}

template <typename PointT>
void RansacSegmenter::segment(const pcl::PointCloud<PointT> &cloud, std::vector<SegmentationLabel> &labels)
{
    // Set all labels to unknown
    labels.assign(cloud.points.size(), SegmentationLabel::UNKNOWN);

    // If cloud contains less than 3 points
    if (cloud.points.size() < 3U)
    {
        return;
    }

    const auto t0 = std::chrono::steady_clock::now();

    // Form initial segmentation (consensus)
    segmentRansac(cloud, labels);

    const auto t1 = std::chrono::steady_clock::now();

    // Form polar grid
    embedPointCloudIntoPolarGrid(cloud, labels);

    const auto t2 = std::chrono::steady_clock::now();

    // Refine classification (reduce number of false positives)
    refineClassificationThroughPolarGridTraversal(labels);

    const auto t3 = std::chrono::steady_clock::now();

    std::cerr << "Elapsed time RANSAC [ms]: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
              << std::endl;

    std::cerr << "Elapsed time polar grid embedding [ms]: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << std::endl;

    std::cerr << "Elapsed time polar grid classification refinement [ms]: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count() << std::endl;

    std::cerr << "Total segmentation time [ms]: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t0).count() << std::endl;
}
} // namespace lidar_processing_lib::segmentation

#endif // LIDAR_PROCESSING__SEGMENTATION__RANSAC_SEGMENTER_HPP
