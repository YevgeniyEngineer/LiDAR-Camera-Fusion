#ifndef LIDAR_PROCESSING__LIDAR_SEGMENTATION__SEGMENTATION_LABEL_HPP
#define LIDAR_PROCESSING__LIDAR_SEGMENTATION__SEGMENTATION_LABEL_HPP

#include <cstdint> // std::uint8_t

namespace lidar_processing::lidar_segmentation
{
enum class SegmentationLabel : std::uint8_t
{
    UNKNOWN = 0U,
    GROUND,
    OBSTACLE,
    TRANSITIONAL_GROUND,
    TRANSITIONAL_OBSTACLE
};
} // namespace lidar_processing::lidar_segmentation

#endif // LIDAR_PROCESSING__LIDAR_SEGMENTATION__SEGMENTATION_LABEL_HPP
