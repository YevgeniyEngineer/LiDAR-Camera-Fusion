#ifndef LIDAR_PROCESSING_LIB__SEGMENTATION__RANSAC_SEGMENTER_HPP
#define LIDAR_PROCESSING_LIB__SEGMENTATION__RANSAC_SEGMENTER_HPP

#include "i_segmenter.hpp"               // ISegmenter
#include <array>                         // std::array
#include <cmath>                         // M_PI
#include <random>                        // std::random_device, std::mt19937, std::uniform_int_distribution
#include <utilities_lib/thread_pool.hpp> // ThreadPool
#include <vector>                        // std::vector

namespace lidar_processing_lib::segmentation
{
class RansacSegmenter : public ISegmenter
{
  public:
    explicit RansacSegmenter(float height_offset, float orthogonal_distance_threshold = 0.1F,
                             std::uint32_t number_of_iterations = 100U, float max_plane_inclination_deg = 25.0F,
                             float consideration_radius = 30.0F, float consideration_height = 1.0);

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

    std::vector<pcl::PointXYZ> processing_points_;

    template <typename PointT>
    void segment(const pcl::PointCloud<PointT> &cloud, std::vector<SegmentationLabel> &labels);
};

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
        if (normal_z < max_plane_cosine_angle)
        {
            continue;
        }

        normal_x *= normalization;
        normal_y *= normalization;

        const float plane_d = (normal_x * point_1.x) + (normal_y * point_1.y) + (normal_z * point_1.z);

        // Count inlier points
        std::uint32_t inlier_count = 0U;
        for (std::size_t i = 0U; i < processing_points_.size(); ++i)
        {
            const float orthogonal_distance =
                std::fabs((normal_x * processing_points_[i].x) + (normal_y * processing_points_[i].y) +
                          (normal_z * processing_points_[i].z) - plane_d);

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
    for (std::size_t i = 0U; i < cloud.points.size(); ++i)
    {
        const float orthogonal_distance =
            std::fabs((a * cloud.points[i].x) + (b * cloud.points[i].y) + (c * cloud.points[i].z) - d);

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
} // namespace lidar_processing_lib::segmentation

#endif // LIDAR_PROCESSING__SEGMENTATION__RANSAC_SEGMENTER_HPP
