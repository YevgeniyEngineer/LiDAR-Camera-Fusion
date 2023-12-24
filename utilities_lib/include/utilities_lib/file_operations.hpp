#pragma once

#include <data_types_lib/cartesian_return.hpp> // CartesianReturn

#include <cstdint>    // std::int64_t
#include <filesystem> // std::filesystem
#include <string>     // std::string
#include <vector>     // std::vector

namespace utilities_lib
{
void readFileNamesWithExtensionFromDirectory(const std::filesystem::path &data_path, const std::string &file_extension,
                                             std::vector<std::filesystem::path> &file_paths);

void loadPointCloudDataFromBinFile(const std::filesystem::path &file_path,
                                   std::vector<data_types_lib::CartesianReturn> &point_cloud);

void readTimestampsFromTxtFile(const std::filesystem::path &file_path, std::vector<std::int64_t> &timestamps);
} // namespace utilities_lib
