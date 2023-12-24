#ifndef DATA_TYPES_LIB__SPHERICAL_RETURN_HPP
#define DATA_TYPES_LIB__SPHERICAL_RETURN_HPP

namespace data_types_lib
{
struct __attribute__((packed)) SphericalReturn final
{
    // Range in meters
    // Offset from the sensor origin to the detected return
    float range;

    // Azimuth in radians
    // Zero straight ahead, counterclockwise
    float azimuth;

    // Elevation in radians
    // Zero straight ahead, positive upward
    float elevation;

    // Point intensity or reflectance
    float intensity;
};
} // namespace data_types_lib

#endif // DATA_TYPES_LIB__SPHERICAL_RETURN_HPP
