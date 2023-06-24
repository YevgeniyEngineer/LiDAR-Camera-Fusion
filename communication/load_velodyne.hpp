#include <types.hpp>

#include <string>
#include <vector>

namespace communication
{
std::vector<common::PointCartesian> loadPointCloudDataFromBin(const std::string &filename);
} // namespace communication