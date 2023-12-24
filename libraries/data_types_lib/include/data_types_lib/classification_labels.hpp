#ifndef DATA_TYPES_LIB__CLASSIFICATION_LABEL_HPP
#define DATA_TYPES_LIB__CLASSIFICATION_LABEL_HPP

#include <cstdint>

namespace data_types_lib
{
enum class ClassificationLabel : std::uint8_t
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

#endif // DATA_TYPES_LIB__CLASSIFICATION_LABEL_HPP
