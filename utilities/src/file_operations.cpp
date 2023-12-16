#include <utilities/file_operations.hpp>

#include <algorithm> // std::sort

namespace utilities
{
void readFileNamesWithExtensionFromDirectory(const std::filesystem::path &data_path, const std::string &file_extension,
                                             std::vector<std::filesystem::path> &file_paths)
{
    // Get all files that match extension from the specified folder
    for (const auto &path : std::filesystem::directory_iterator(data_path))
    {
        if (std::filesystem::is_regular_file(path) && (path.path().extension() == file_extension))
        {
            file_paths.push_back(path.path());
        }
    }

    // Ensure data is sorted in lexicographic order
    std::sort(file_paths.begin(), file_paths.end());
}
} // namespace utilities