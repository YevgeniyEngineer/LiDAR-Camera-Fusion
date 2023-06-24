#pragma once

#include <algorithm>
#include <filesystem>
#include <vector>

std::vector<std::filesystem::path> readFilenames(const std::filesystem::path &data_path,
                                                 const std::string &file_extension)
{
    std::vector<std::filesystem::path> file_names;

    for (const auto &path : std::filesystem::directory_iterator(data_path))
    {
        if (std::filesystem::is_regular_file(path) && (path.path().extension() == file_extension))
        {
            file_names.push_back(path.path());
        }
    }

    // Ensure data is sorted in lexicographic order
    std::sort(file_names.begin(), file_names.end());

    return file_names;
}