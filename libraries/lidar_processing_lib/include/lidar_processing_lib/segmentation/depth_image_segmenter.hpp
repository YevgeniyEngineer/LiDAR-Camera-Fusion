#ifndef LIDAR_PROCESSING_LIB__SEGMENTATION__DEPTH_IMAGE_SEGMENTER_HPP
#define LIDAR_PROCESSING_LIB__SEGMENTATION__DEPTH_IMAGE_SEGMENTER_HPP

#include "i_segmenter.hpp"        // ISegmenter
#include <chrono>                 // std::chrono
#include <cstdint>                // std::uint32_t
#include <eigen3/Eigen/Dense>     // Eigen::
#include <limits>                 // std::numeric_limits
#include <utilities_lib/math.hpp> // constexprRound
#include <vector>                 // std::vector

namespace lidar_processing_lib::segmentation
{
class DepthImageSegmenter : public ISegmenter
{
  public:
    using SegmentationLabel = data_types_lib::SegmentationLabel;

    struct RangeImagePixel
    {
        // This pixel does not map to point cloud
        static constexpr auto INVALID_INDEX = std::numeric_limits<std::uint32_t>::max();

        float range = std::numeric_limits<float>::max();
        std::uint32_t index = INVALID_INDEX; // index of the point cloud
        std::uint8_t r = 0U;
        std::uint8_t g = 0U;
        std::uint8_t b = 0U;

        inline void reset() noexcept
        {
            range = std::numeric_limits<float>::max();
            index = INVALID_INDEX;
            r = 0U;
            g = 0U;
            b = 0U;
        }
    };

    // Values acceptable for Velodyne HDL-64E (tightest values)
    static constexpr float DEG_TO_RAD = static_cast<float>(M_PI / 180.0F);

    static constexpr float VERTICAL_RESOLUTION_DEG = 0.33F;
    static constexpr float VERTICAL_RESOLUTION_RAD = VERTICAL_RESOLUTION_DEG * DEG_TO_RAD;
    static constexpr float VERTICAL_FIELD_OF_VIEW_DEG = 26.9F;
    static constexpr float VERTICAL_FIELD_OF_VIEW_RAD = VERTICAL_FIELD_OF_VIEW_DEG * DEG_TO_RAD;

    static constexpr float HORIZONTAL_RESOLUTION_DEG = 0.1F;
    static constexpr float HORIZONTAL_RESOLUTION_RAD = HORIZONTAL_RESOLUTION_DEG * DEG_TO_RAD;
    static constexpr float HORIZONTAL_FIELD_OF_VIEW_DEG = 360.0F;
    static constexpr float HORIZONTAL_FIELD_OF_VIEW_RAD = HORIZONTAL_FIELD_OF_VIEW_DEG * DEG_TO_RAD;

    // width ~= 3600
    static constexpr auto DEPTH_IMAGE_WIDTH =
        static_cast<std::uint32_t>(std::ceil(HORIZONTAL_FIELD_OF_VIEW_DEG / HORIZONTAL_RESOLUTION_DEG));

    // height ~= 82
    static constexpr auto DEPTH_IMAGE_HEIGHT =
        static_cast<std::uint32_t>(std::ceil(VERTICAL_FIELD_OF_VIEW_DEG / VERTICAL_RESOLUTION_DEG));

    DepthImageSegmenter(float min_range = 1.0F, float max_range = 80.0F);
    ~DepthImageSegmenter();

    void run(const pcl::PointCloud<pcl::PointXYZ> &cloud, std::vector<SegmentationLabel> &labels) override;
    void run(const pcl::PointCloud<pcl::PointXYZI> &cloud, std::vector<SegmentationLabel> &labels) override;

  private:
    float min_range_;
    float max_range_;

    // Range image - contiguous in memory, index mapping [width x height] - row major order
    std::vector<RangeImagePixel> depth_image_;

    constexpr static inline std::uint32_t rowMajorIndex(const std::uint32_t x, const std::uint32_t y) noexcept
    {
        return (y * DEPTH_IMAGE_WIDTH + x);
    }

    template <typename PointT>
    void segment(const pcl::PointCloud<PointT> &cloud, std::vector<SegmentationLabel> &labels);

    template <typename PointT> static inline float rangeSquared(const PointT &point) noexcept
    {
        return (point.x * point.x) + (point.y * point.y);
    }

    template <typename PointT> static inline float range(const PointT &point) noexcept
    {
        return std::sqrt(rangeSquared(point));
    }

    template <typename PointT> void constructDepthImage(const pcl::PointCloud<PointT> &cloud);
};

template <typename PointT> void DepthImageSegmenter::constructDepthImage(const pcl::PointCloud<PointT> &cloud)
{
    static constexpr float RECIPROCAL_HORIZONTAL_FOV = 1.0F / HORIZONTAL_FIELD_OF_VIEW_RAD;
    static constexpr float RECIPROCAL_VERTICAL_FOV = 1.0F / VERTICAL_FIELD_OF_VIEW_RAD;

    // Ensure the range image is properly sized and reset
    if (depth_image_.size() != (DEPTH_IMAGE_WIDTH * DEPTH_IMAGE_HEIGHT))
    {
        depth_image_.resize(DEPTH_IMAGE_WIDTH * DEPTH_IMAGE_HEIGHT);
    }

    // Reset all pixels of the range image
    std::fill(depth_image_.begin(), depth_image_.end(), RangeImagePixel());

    // Fill range image with point cloud points
    for (std::uint32_t i = 0U; i < cloud.points.size(); ++i)
    {
        const auto &point = cloud.points[i];

        const float dd_sqr = point.x * point.x + point.y * point.y;
        const float range = std::sqrt(dd_sqr + point.z * point.z);

        if ((range < min_range_) || (range > max_range_))
        {
            continue; // Skip points out of range
        }

        float azimuth_rad = std::atan2(point.y, point.x);
        if (azimuth_rad < 0)
        {
            azimuth_rad += static_cast<float>(2.0 * M_PI); // angle mapped to [0, 2*pi]
        }

        const float elevation_rad = std::atan2(point.z, std::sqrt(dd_sqr)) + M_PI_2f; // angle mapped to [0, pi]

        // Convention: (0, 0) coordinate of the image located at the top left corner

        const auto x = std::min(static_cast<std::uint32_t>((azimuth_rad + HORIZONTAL_FIELD_OF_VIEW_RAD / 2) *
                                                           RECIPROCAL_HORIZONTAL_FOV * DEPTH_IMAGE_WIDTH),
                                DEPTH_IMAGE_WIDTH - 1U);

        const auto y = std::min(
            static_cast<std::uint32_t>((M_PI_2f - elevation_rad) * RECIPROCAL_VERTICAL_FOV * DEPTH_IMAGE_HEIGHT),
            DEPTH_IMAGE_HEIGHT - 1U);

        const auto row_major_index = rowMajorIndex(x, y);

        auto &pixel = depth_image_[row_major_index];
        if (range < pixel.range)
        {
            pixel.range = range;
            pixel.index = i;
            pixel.r = 0;
            pixel.g = 255;
            pixel.b = 0;
        }
    }

    std::uint32_t number_of_depth_image_points = 0U;
    for (const auto &point : depth_image_)
    {
        if (point.index != RangeImagePixel::INVALID_INDEX)
        {
            ++number_of_depth_image_points;
        }
    }

    std::cerr << "Number of depth image pixels: " << number_of_depth_image_points << " out of a total of "
              << cloud.points.size() << " points" << std::endl;
}

template <typename PointT>
void DepthImageSegmenter::segment(const pcl::PointCloud<PointT> &cloud, std::vector<SegmentationLabel> &labels)
{
    // Set all labels to unknown
    labels.assign(cloud.points.size(), SegmentationLabel::UNKNOWN);

    // Construct depth image
    const auto t1 = std::chrono::steady_clock::now();

    constructDepthImage(cloud);

    const auto t2 = std::chrono::steady_clock::now();
    std::cerr << "Construction of depth image time [ms]: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << std::endl;
}

} // namespace lidar_processing_lib::segmentation

#endif // LIDAR_PROCESSING_LIB__SEGMENTATION__DEPTH_IMAGE_SEGMENTER_HPP
