#include <utilities_lib/file_operations.hpp>

#include <algorithm> // std::sort
#include <cstring>   // std::memcpy
#include <ctime>     // std::tm
#include <fstream>   // std::ifstream
#include <ios>       // std::ios::end
#include <iostream>  // std::cout
#include <sstream>   // std::stringstream
#include <string>    // std::string

namespace utilities_lib
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

void loadPointCloudDataFromBinFile(const std::filesystem::path &file_path,
                                   std::vector<data_types_lib::CartesianReturn> &point_cloud)
{
    // Clean the cache
    point_cloud.clear();

    // Open the binary file
    std::ifstream input_file{file_path.c_str(), std::ios::binary};

    // Check for errors during loading
    if (!input_file.good())
    {
        std::cerr << "Could not read the file: " << file_path << std::endl;
        return;
    }

    // Read the number of points
    input_file.seekg(0, std::ios::end);
    const std::size_t file_size = input_file.tellg();
    const std::size_t number_of_points = file_size / sizeof(data_types_lib::CartesianReturn);

    if (file_size % sizeof(data_types_lib::CartesianReturn) != 0)
    {
        std::cerr << "File size is not a multiple of point size, potential data misalignment." << std::endl;
    }

    // Start from the beginning of the file
    input_file.seekg(0, std::ios::beg);

    // Resize the cloud
    point_cloud.resize(number_of_points);

    // Load point cloud
    input_file.read(reinterpret_cast<char *>(point_cloud.data()),
                    number_of_points * sizeof(data_types_lib::CartesianReturn));

    if (!input_file)
    {
        std::cerr << "Error occurred while reading the file." << std::endl;
        point_cloud.clear();
    }

    // Close the input file
    input_file.close();
}

void readTimestampsFromTxtFile(const std::filesystem::path &file_path, std::vector<std::int64_t> &timestamps)
{
    // Clear cache
    timestamps.clear();

    // Check the file extension
    if (file_path.extension() != ".txt")
    {
        std::cerr << "Invalid extension, expected .txt, provided path: " << file_path << std::endl;
        return;
    }

    // Open input file
    std::ifstream input_file{file_path};
    if (!input_file.is_open())
    {
        std::cerr << "Could not read the file: " << file_path << std::endl;
        return;
    }

    // Read data from the file
    std::string line;
    while (std::getline(input_file, line))
    {
        std::tm tm{};
        std::stringstream ss{line};
        ss >> std::get_time(&tm, "%Y-%m-%d %H:%M:%S"); // strptime does not exist in C++ so we have to do this
        tm.tm_isdst = -1;                              // DST information unknown
        const std::time_t time = std::mktime(&tm);
        if (time != -1)
        {
            auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::from_time_t(time).time_since_epoch());
            const std::string nanoseconds = line.substr(20);
            nanosec += std::chrono::nanoseconds(std::stoll(nanoseconds));
            timestamps.push_back(nanosec.count());
        }
    }
}
} // namespace utilities_lib
