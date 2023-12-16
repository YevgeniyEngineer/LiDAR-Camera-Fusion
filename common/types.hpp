#pragma once

#include <cstdint>

namespace common
{
enum class GroundSegmentationLabels : std::uint32_t
{
    UNCLASSIFIED = 0,
    GROUND,
    OBSTACLE,
    UNKNOWN
};

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

struct PointCartesian
{
    float x_m;
    float y_m;
    float z_m;
    float intensity;
};

struct PointSpherical
{
    float range_m;
    float azimuth_rad;
    float elevation_rad;
    float intensity;
};

struct ImagePixel
{
    float x;
    float y;
    float red;
    float green;
    float blue;
};

} // namespace common
