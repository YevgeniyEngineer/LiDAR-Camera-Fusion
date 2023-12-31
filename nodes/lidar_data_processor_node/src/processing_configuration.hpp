#ifndef PROCESSING_CONFIGURATION_HPP
#define PROCESSING_CONFIGURATION_HPP

#include <array>   // std::array
#include <cstdint> // std::uint32_t
#include <string>  // std::string
#include <utility> // std::pair
#include <vector>  // std::vector

struct PointXY
{
    float x;
    float y;
};

struct RansacConfiguration final
{
    float orthogonal_distance_threshold;
    std::uint32_t number_of_iterations;
    std::uint32_t thread_count;
};

struct SegmentationConfiguration final
{
    std::string algorithm;
    RansacConfiguration ransac;
};

struct ProcessingConfiguration final
{
    float height_offset;
    std::array<PointXY, 4> bounding_box;
    SegmentationConfiguration segmentation;
};

#endif // PROCESSING_CONFIGURATION_HPP
