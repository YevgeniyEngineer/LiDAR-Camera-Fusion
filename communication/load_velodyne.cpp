#include "load_velodyne.hpp"

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>

namespace communication
{
std::vector<common::PointCartesian> loadPointCloudDataFromBin(const std::string &filename)
{
    // open file
    std::ifstream input(filename.c_str(), std::ios::binary);
    if (!input.good())
    {
        std::cerr << "Could not read file: " << filename << std::endl;
        return {};
    }

    // read number of points
    input.seekg(0, std::ios::end);
    std::size_t num_points = input.tellg() / (4 * sizeof(float));
    input.seekg(0, std::ios::beg);

    // load point cloud
    std::vector<float> raw_data(num_points);
    input.read((char *)&raw_data[0], num_points * 4 * sizeof(float));
    input.close();

    // Copy raw floats into point cloud XYZI fields
    common::PointCartesian point_cache;
    std::vector<common::PointCartesian> point_cloud{};
    point_cloud.reserve(raw_data.size() / 4);
    for (std::size_t i = 0; i < raw_data.size(); i += 4)
    {
        point_cache.x_m = raw_data[i];
        point_cache.y_m = raw_data[i + 1];
        point_cache.z_m = raw_data[i + 2];
        point_cache.intensity = raw_data[i + 3];
        point_cloud.push_back(point_cache);
    }

    return point_cloud;
}
} // namespace communication
