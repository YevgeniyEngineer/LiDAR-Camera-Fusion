#ifndef LIDAR_PROCESSING_LIB__SEGMENTATION__DEPTH_IMAGE_SEGMENTER_HPP
#define LIDAR_PROCESSING_LIB__SEGMENTATION__DEPTH_IMAGE_SEGMENTER_HPP

#include "i_segmenter.hpp"        // ISegmenter
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

    // Values acceptable for Velodyne HDL-64E (tightest values)
    static constexpr float DEG_TO_RAD = static_cast<float>(M_PI / 180.0F);

    static constexpr float VERTICAL_RESOLUTION_DEG = 0.33F;
    static constexpr float VERTICAL_RESOLUTION_RAD = VERTICAL_RESOLUTION_DEG * DEG_TO_RAD;
    static constexpr float VERTICAL_FIELD_OF_VIEW_DEG = 26.9F;

    static constexpr float HORIZONTAL_RESOLUTION_DEG = 0.1F;
    static constexpr float HORIZONTAL_RESOLUTION_RAD = HORIZONTAL_RESOLUTION_DEG * DEG_TO_RAD;
    static constexpr float HORIZONTAL_FIELD_OF_VIEW_DEG = 360.0F;

    // width ~= 3600
    static constexpr auto RANGE_IMAGE_WIDTH =
        static_cast<std::uint32_t>(std::ceil(HORIZONTAL_FIELD_OF_VIEW_DEG / HORIZONTAL_RESOLUTION_DEG));

    // height ~= 82
    static constexpr auto RANGE_IMAGE_HEIGHT =
        static_cast<std::uint32_t>(std::ceil(VERTICAL_FIELD_OF_VIEW_DEG / VERTICAL_RESOLUTION_DEG));

    DepthImageSegmenter();
    ~DepthImageSegmenter();

    void run(const pcl::PointCloud<pcl::PointXYZ> &cloud, std::vector<SegmentationLabel> &labels) override;
    void run(const pcl::PointCloud<pcl::PointXYZI> &cloud, std::vector<SegmentationLabel> &labels) override;

  private:
    struct RangeImagePixel
    {
        // This pixel does not map to point cloud
        static constexpr auto INVALID_INDEX = std::numeric_limits<std::uint32_t>::max();

        float range = std::numeric_limits<float>::infinity();
        std::uint32_t index = INVALID_INDEX; // index of the point cloud
        std::uint8_t r = 0U;
        std::uint8_t g = 0U;
        std::uint8_t b = 0U;

        inline void reset() noexcept
        {
            range = std::numeric_limits<float>::infinity();
            index = INVALID_INDEX;
            r = 0U;
            g = 0U;
            b = 0U;
        }
    };

    std::vector<RangeImagePixel> range_image_;

    constexpr static inline std::uint32_t rowMajorIndex(const std::uint32_t x, const std::uint32_t y) noexcept
    {
        return (y * RANGE_IMAGE_WIDTH + x);
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

    template <typename PointT> void constructRangeImage(const pcl::PointCloud<PointT> &cloud);
};

template <typename PointT> void DepthImageSegmenter::constructRangeImage(const pcl::PointCloud<PointT> &cloud)
{
    // Ensure the range image is properly sized and reset
    if (range_image_.size() != RANGE_IMAGE_WIDTH * RANGE_IMAGE_HEIGHT)
    {
        range_image_.resize(RANGE_IMAGE_WIDTH * RANGE_IMAGE_HEIGHT);
    }

    // Reset range image
    std::fill(range_image_.begin(), range_image_.end(), RangeImagePixel()); // Reset all pixels

    // Fill range image with point cloud points
    for (std::uint32_t i = 0U; i < cloud.points.size(); ++i)
    {
        const auto &point = cloud.points[i];

        float azimuth_rad = std::atan2(point.y, point.x);
        if (azimuth_rad < 0)
        {
            azimuth_rad += static_cast<float>(2.0 * M_PI);
        }

        const float dd_sqr = point.x * point.x + point.y * point.y;
        const float elevation_rad = std::atan2(point.z, std::sqrt(dd_sqr)) + M_PI_2f;

        const float range = std::sqrt(dd_sqr + point.z * point.z);

        const auto x = static_cast<std::uint32_t>(azimuth_rad / HORIZONTAL_RESOLUTION_RAD) % RANGE_IMAGE_WIDTH;
        const auto y = static_cast<std::uint32_t>(elevation_rad / VERTICAL_RESOLUTION_RAD) % RANGE_IMAGE_HEIGHT;

        const auto row_major_index = rowMajorIndex(x, y);

        auto &pixel = range_image_[row_major_index];
        if (range < pixel.range)
        {
            pixel.range = range;
            pixel.index = i;
            pixel.r = 0;
            pixel.g = 255;
            pixel.b = 0;
        }
    }
}

template <typename PointT>
void DepthImageSegmenter::segment(const pcl::PointCloud<PointT> &cloud, std::vector<SegmentationLabel> &labels)
{
    // Set all labels to unknown
    labels.assign(cloud.points.size(), SegmentationLabel::UNKNOWN);
}

} // namespace lidar_processing_lib::segmentation

#endif // LIDAR_PROCESSING_LIB__SEGMENTATION__DEPTH_IMAGE_SEGMENTER_HPP
