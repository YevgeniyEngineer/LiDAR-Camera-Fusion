#ifndef DATA_TYPES_LIB__SEGMENTATION_LABEL_HPP
#define DATA_TYPES_LIB__SEGMENTATION_LABEL_HPP

#include <cstdint>

namespace data_types_lib
{
enum class SegmentationLabel : std::uint8_t
{
    UNKNOWN = 0,
    GROUND,
    OBSTACLE,
    TRANSITIONAL_GROUND,
    TRANSITIONAL_OBSTACLE
};
} // namespace data_types_lib

#endif // DATA_TYPES_LIB__SEGMENTATION_LABEL_HPP
