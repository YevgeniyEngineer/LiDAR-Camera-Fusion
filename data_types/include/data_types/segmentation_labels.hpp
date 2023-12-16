#pragma once

#include <cstdint>

namespace data_types
{
enum class GroundSegmentationLabels : std::uint32_t
{
    UNCLASSIFIED = 0,
    GROUND,
    OBSTACLE,
    UNKNOWN
};
} // namespace data_types
