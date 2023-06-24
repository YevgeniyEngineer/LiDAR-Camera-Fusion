#pragma once

#include <chrono>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <vector>

/// @throws runtime_error if file could not be found
std::vector<std::int64_t> readTimestamps(const std::string &filename)
{
    std::vector<std::int64_t> timestamps;
    std::ifstream file(filename);

    if (!file.is_open())
    {
        throw std::runtime_error("Could not open file");
    }

    std::string line;
    while (std::getline(file, line))
    {
        std::tm tm = {};
        std::stringstream ss(line);
        ss >> std::get_time(&tm, "%Y-%m-%d %H:%M:%S"); // strptime does not exist in C++ so we have to do this
        tm.tm_isdst = -1;                              // DST information unknown
        std::time_t time_t = std::mktime(&tm);
        if (time_t != -1)
        {
            auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::from_time_t(time_t).time_since_epoch());
            std::string nanoseconds = line.substr(20);
            nanosec += std::chrono::nanoseconds(std::stoll(nanoseconds));
            timestamps.push_back(nanosec.count());
        }
    }

    return timestamps;
}