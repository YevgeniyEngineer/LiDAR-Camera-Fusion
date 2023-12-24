#pragma once

#include <cstdint>

namespace data_types_lib
{
enum class ClassificationLabels : std::uint32_t
{
    UNCLASSIFIED = 0,
    NOISE,
    CAR,
    BICYCLE,
    BUS,
    MOTORCYCLE,
    TRUCK,
    ANIMAL,
    BIRD,
    PEDESTRIAN,
    BICYCLIST,
    MOTORCYCLIST,
    ROAD,
    SIDEWALK,
    BUILDING,
    FENCE,
    VEGETATION,
    TERRAIN,
    POLE,
    TRAFFIC_SIGN
};
} // namespace data_types_lib
