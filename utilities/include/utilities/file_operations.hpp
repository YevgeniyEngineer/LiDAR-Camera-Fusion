#pragma once

#include <filesystem> // std::filesystem
#include <string>     // std::string
#include <vector>     // std::vector

namespace utilities
{
void readFileNamesWithExtensionFromDirectory(const std::filesystem::path &data_path, const std::string &file_extension,
                                             std::vector<std::filesystem::path> &file_paths);
} // namespace utilities
