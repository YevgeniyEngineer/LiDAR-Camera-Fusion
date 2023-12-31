#ifndef LIDAR_PROCESSING_LIB__SEGMENTATION__RANSAC_SEGMENTER_HPP
#define LIDAR_PROCESSING_LIB__SEGMENTATION__RANSAC_SEGMENTER_HPP

#include "i_segmenter.hpp"               // ISegmenter
#include <array>                         // std::array
#include <random>                        // std::random_device, std::mt19937, std::uniform_int_distribution
#include <utilities_lib/thread_pool.hpp> // ThreadPool

namespace lidar_processing_lib::segmentation
{
class RansacSegmenter : public ISegmenter
{
  public:
    explicit RansacSegmenter(float orthogonal_distance_threshold = 0.1F, std::uint32_t number_of_iterations = 100U,
                             std::size_t thread_count = 8U);

    ~RansacSegmenter();

    void run(const pcl::PointCloud<pcl::PointXYZ> &cloud, std::vector<SegmentationLabel> &labels) override;
    void run(const pcl::PointCloud<pcl::PointXYZI> &cloud, std::vector<SegmentationLabel> &labels) override;

  private:
    float orthogonal_distance_threshold_;
    std::uint32_t number_of_iterations_;

    utilities_lib::ThreadPool thread_pool_;
    std::vector<std::future<std::uint32_t>> futures_;

    template <typename PointT>
    void segment(const pcl::PointCloud<PointT> &cloud, std::vector<SegmentationLabel> &labels);
};

template <typename PointT>
void RansacSegmenter::segment(const pcl::PointCloud<PointT> &cloud, std::vector<SegmentationLabel> &labels)
{
    // Set all labels to unknown
    labels.resize(cloud.points.size());
    for (std::uint32_t i = 0U; i < cloud.points.size(); ++i)
    {
        labels[i] = SegmentationLabel::UNKNOWN;
    }

    // If cloud contains less than 3 points
    if (cloud.points.size() < 3U)
    {
        return;
    }

    // Plane coefficients
    float a = 0.0F;
    float b = 0.0F;
    float c = 1.0F;
    float d = 0.0F;

    // For index generation
    std::mt19937 generator(42);
    std::uniform_int_distribution<std::uint32_t> distribution(0, cloud.points.size() - 1U);

    // Calculate chunk size to allocate for each thread
    const std::uint32_t chunk_size = cloud.points.size() / thread_pool_.threadCount();

    // RANSAC
    std::uint32_t best_inlier_count = 0U;
    for (std::uint32_t iteration = 0U; iteration < number_of_iterations_; ++iteration)
    {
        // Choose 3 random points

        // Select first point
        const std::uint32_t point_1_index = distribution(generator);
        const auto &point_1 = cloud.points[point_1_index];

        // Select second point
        std::uint32_t point_2_index = distribution(generator);
        while (point_1_index == point_2_index)
        {
            point_2_index = distribution(generator);
        }
        const auto &point_2 = cloud.points[point_2_index];

        // Select third point
        std::uint32_t point_3_index = distribution(generator);
        while ((point_1_index == point_3_index) && (point_2_index == point_3_index))
        {
            point_3_index = distribution(generator);
        }
        const auto &point_3 = cloud.points[point_3_index];

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
        normal_x *= normalization;
        normal_y *= normalization;
        normal_z *= normalization;

        const float plane_d = (normal_x * point_1.x) + (normal_y * point_1.y) + (normal_z * point_1.z);

        // Parallel inlier counting
        // Assign work to each thread
        for (std::size_t t = 0U; t < thread_pool_.threadCount(); ++t)
        {
            futures_[t] = thread_pool_.enqueue(
                [this, &cloud, chunk_size, normal_x, normal_y, normal_z, plane_d, t]() -> std::uint32_t {
                    std::uint32_t local_inlier_count = 0U;

                    const std::size_t start_index = t * chunk_size;
                    const std::size_t stop_index = std::min((t + 1) * chunk_size, cloud.points.size());

                    for (std::size_t j = start_index; j < stop_index; ++j)
                    {
                        const float orthogonal_distance =
                            std::fabs((normal_x * cloud.points[j].x) + (normal_y * cloud.points[j].y) +
                                      (normal_z * cloud.points[j].z) - plane_d);

                        if (orthogonal_distance < orthogonal_distance_threshold_)
                        {
                            ++local_inlier_count;
                        }
                    }

                    return local_inlier_count;
                });
        }

        // Collect results from all threads
        std::uint32_t inlier_count = 0U;
        for (std::size_t t = 0U; t < thread_pool_.threadCount(); ++t)
        {
            inlier_count += futures_[t].get();
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

    // Update labels
    for (std::size_t t = 0U; t < thread_pool_.threadCount(); ++t)
    {
        futures_[t] = thread_pool_.enqueue([this, &cloud, &labels, chunk_size, a, b, c, d, t]() -> std::uint32_t {
            const std::size_t start_index = t * chunk_size;
            const std::size_t stop_index = std::min((t + 1) * chunk_size, cloud.points.size());

            for (std::size_t j = start_index; j < stop_index; ++j)
            {
                const float orthogonal_distance =
                    std::fabs((a * cloud.points[j].x) + (b * cloud.points[j].y) + (c * cloud.points[j].z) - d);

                if (orthogonal_distance < orthogonal_distance_threshold_)
                {
                    labels[j] = SegmentationLabel::GROUND;
                }
                else
                {
                    labels[j] = SegmentationLabel::OBSTACLE;
                }
            }

            // Note this return is just a placeholder
            return 0U;
        });
    }

    // Wait for results
    for (auto &future : futures_)
    {
        future.get();
    }
}
} // namespace lidar_processing_lib::segmentation

#endif // LIDAR_PROCESSING__SEGMENTATION__RANSAC_SEGMENTER_HPP
