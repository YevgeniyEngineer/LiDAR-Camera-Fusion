#ifndef DATA_TYPES_LIB__CARTESIAN_RETURN_HPP
#define DATA_TYPES_LIB__CARTESIAN_RETURN_HPP

namespace data_types_lib
{
struct __attribute__((packed)) CartesianReturn final
{
    // Forward positive, in meters
    float x;

    // Leftward positive, in meters
    float y;

    // Upward positive, in meters
    float z;

    // Point intensity or reflectance
    float intensity;
};
} // namespace data_types_lib

#endif // DATA_TYPES_LIB__CARTESIAN_RETURN_HPP
