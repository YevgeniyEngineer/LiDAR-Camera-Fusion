// Local
#include "types.hpp"

// STL
#include <chrono> // std::chrono
#include <chrono>
#include <cstdint>    // std::uint32_t
#include <cstring>    // std::memcpy
#include <filesystem> // std::filesystem
#include <fstream>
#include <functional> // std::bind
#include <iomanip>
#include <memory>    // std::shared_ptr
#include <stdexcept> // std::runtime_error
#include <string>    // std::string
#include <tuple>     // std::tuple
#include <utility>   // std::move
#include <vector>    // std::vector
#include <vector>

// ROS2
#include <rclcpp/executors.hpp>             // rclcpp::spin
#include <rclcpp/node.hpp>                  // rclcpp::Node
#include <rclcpp/publisher.hpp>             // rclcpp::Publisher
#include <rclcpp/qos.hpp>                   // rclcpp::QoS
#include <rclcpp/timer.hpp>                 // rclcpp::TimerBase
#include <rclcpp/utilities.hpp>             // rclcpp::shutdown
#include <sensor_msgs/msg/point_cloud2.hpp> // sensor_msgs::msg::PointCloud2
#include <sensor_msgs/msg/point_field.hpp>  // sensor_msgs::msg::PointField

using namespace std::chrono_literals;

enum PointFieldTypes
{
    INT8 = 1,
    UINT8 = 2,
    INT16 = 3,
    UINT16 = 4,
    INT32 = 5,
    UINT32 = 6,
    FLOAT32 = 7,
    FLOAT64 = 8
};

class DataReaderNode : public rclcpp::Node
{
  public:
    static constexpr std::size_t MAX_POINTS = 200'000U;

    using PointCloud2 = sensor_msgs::msg::PointCloud2;

    DataReaderNode(const std::filesystem::path &data_path, std::string topic = "pointcloud")
        : rclcpp::Node::Node("data_reader_node"), cloud_publisher_(nullptr)
    {
        // Check if directories exist
        if (!std::filesystem::exists(data_path))
        {
            throw std::runtime_error("Specified data path does not exist.");
        }

        std::filesystem::path timestamps_file = data_path / "timestamps_start.txt";
        if (!std::filesystem::exists(timestamps_file))
        {
            throw std::runtime_error("Timestamp data file timestamps_start.txt was not found.");
        }

        std::filesystem::path binary_data_path = data_path / "data";
        if (!std::filesystem::exists(binary_data_path))
        {
            throw std::runtime_error("Data path containing *.bin files was not found..");
        }

        // Accumulate timestamps
        timestamp_cache_ = readTimestamps(timestamps_file);
        std::cout << "Read " << timestamp_cache_.size() << " timestamps\n";

        // Read files in ascending order
        std::vector<std::filesystem::path> data_files;
        data_files.reserve(timestamp_cache_.size());
        for (const auto &file_path : std::filesystem::directory_iterator(binary_data_path))
        {
            if (std::filesystem::is_regular_file(file_path) && file_path.path().extension() == ".bin")
            {
                data_files.push_back(file_path.path());
            }
        }

        // Ensure data files are correctly ordered - sort lexicographically
        std::sort(data_files.begin(), data_files.end());
        std::cout << "Read " << data_files.size() << " data files\n";

        // Check the number of timestamps matches the number of data files
        if (data_files.size() != timestamp_cache_.size())
        {
            throw std::runtime_error("The number of timestamps does not equal the number of data files");
        }

        // Reserve cache memory
        point_cloud_cache_.reserve(timestamp_cache_.size());
        for (std::size_t i = 0; i < timestamp_cache_.size(); ++i)
        {
            // Read the data from the file
            std::vector<common::PointCartesian> point_cloud_data = loadPointCloudDataFromBin(data_files[i]);

            if (point_cloud_data.empty())
            {
                std::cout << "Empty binary file, skipping.\n";
                continue;
            }

            std::cout << "Loaded data from bin of size " << point_cloud_data.size() << std::endl;

            // Convert PointCartesian to PointCloudCache and add to the container
            PointCloud2 point_cloud_message;
            point_cloud_message.height = 1;
            point_cloud_message.width = point_cloud_data.size();
            point_cloud_message.is_bigendian = false;
            point_cloud_message.point_step = sizeof(common::PointCartesian);
            point_cloud_message.row_step = sizeof(common::PointCartesian) * point_cloud_data.size();
            point_cloud_message.is_dense = true;

            point_cloud_message.header.stamp.sec =
                static_cast<std::int32_t>(static_cast<double>(timestamp_cache_[i]) * 1e-9);
            point_cloud_message.header.stamp.nanosec = static_cast<std::uint32_t>(
                timestamp_cache_[i] -
                static_cast<std::int64_t>(static_cast<double>(point_cloud_message.header.stamp.sec) * 1e9));

            point_cloud_message.header.frame_id = "pointcloud";

            std::vector<std::tuple<std::string, std::uint32_t, std::uint8_t, std::uint32_t>> fields = {
                {"x", offsetof(common::PointCartesian, x_m), PointFieldTypes::FLOAT32, 1},
                {"y", offsetof(common::PointCartesian, y_m), PointFieldTypes::FLOAT32, 1},
                {"z", offsetof(common::PointCartesian, z_m), PointFieldTypes::FLOAT32, 1},
                {"intensity", offsetof(common::PointCartesian, intensity), PointFieldTypes::FLOAT32, 1}};

            point_cloud_message.fields.reserve(fields.size());
            for (const auto &field : fields)
            {
                sensor_msgs::msg::PointField field_cache;
                field_cache.name = std::get<0>(field);
                field_cache.offset = std::get<1>(field);
                field_cache.datatype = std::get<2>(field);
                field_cache.count = std::get<3>(field);
                point_cloud_message.fields.push_back(field_cache);
            }

            // Copy byte data
            point_cloud_message.data.resize(sizeof(common::PointCartesian) * point_cloud_data.size());
            std::memcpy(point_cloud_message.data.data(), point_cloud_data.data(),
                        sizeof(common::PointCartesian) * point_cloud_data.size());

            // Add message to the message cache
            point_cloud_cache_.push_back(std::move(point_cloud_message));
        }

        // Set iterator to the first position
        current_index_ = 0;

        // Specify QoS settings
        rclcpp::QoS qos(2);
        qos.keep_last(2);
        qos.reliable();
        qos.durability_volatile();
        qos.liveliness(rclcpp::LivelinessPolicy::SystemDefault);

        // How long a node must wait before declaring itself "alive" to the rest of the system again
        // If the node fails to send out a liveliness message within the specified lease duration, it is considered
        // "dead" or "unresponsive" by the rest of the system
        qos.liveliness_lease_duration(std::chrono::seconds(1));

        // How long a node must wait for a response from a remote node before declaring it as "dead" or "unresponsive"
        // If the remote node fails to respond within the specified deadline, the requesting node considers the remote
        // node as "dead" or "unresponsive"
        qos.deadline(std::chrono::seconds(1));

        // Create publisher for PointCloud2 message type
        cloud_publisher_ = this->create_publisher<PointCloud2>(topic, qos);

        // Create a timer with a dummy time point
        std::cout << "Aggregated " << point_cloud_cache_.size() << " point clouds.\n";
        std::cout << "Starting publication.\n";
        updateTimerAndPublish(std::chrono::nanoseconds(timestamp_cache_[1] - timestamp_cache_[0]));
    }

    ~DataReaderNode()
    {
    }

  private:
    void updateTimerAndPublish(std::chrono::nanoseconds interval)
    {
        // Create a new timer
        timer_ = this->create_wall_timer(interval, [this]() {
            // Is index valid?
            if (current_index_ == point_cloud_cache_.size())
            {
                // wraparound
                current_index_ = 0U;
            }

            // Publish a message
            cloud_publisher_->publish(point_cloud_cache_[current_index_]);

            // Create a new timer
            // Sleep 100ms at the boundary
            std::chrono::nanoseconds sleep_duration_nanosec = 100'000'000ns;

            // std::cout << "Current index: " << current_index_ << " Timestamp size: " << timestamp_cache_.size()
            //           << std::endl;

            if (current_index_ + 1 != timestamp_cache_.size())
            {
                sleep_duration_nanosec =
                    std::chrono::nanoseconds(timestamp_cache_[current_index_ + 1] - timestamp_cache_[current_index_]);
            }

            // std::cout << "Sleep duration (ms): "
            //           << std::chrono::duration_cast<std::chrono::milliseconds>(sleep_duration_nanosec).count()
            //           << std::endl;

            ++current_index_;
            updateTimerAndPublish(std::move(sleep_duration_nanosec));
        });
    }

    std::vector<common::PointCartesian> loadPointCloudDataFromBin(const std::string &filename)
    {
        // allocate 4 MB buffer (only ~130*4*4 KB are needed)
        std::size_t buffer_size = 1'000'000U;
        std::vector<float> data_buffer(buffer_size);

        // load point cloud
        std::ifstream input(filename.c_str(), std::ios::binary);
        if (!input.good())
        {
            std::cerr << "Could not read file: " << filename << std::endl;
            return {};
        }

        input.read(reinterpret_cast<char *>(data_buffer.data()), buffer_size * sizeof(float));
        std::size_t number_of_floats = input.gcount() / sizeof(float);

        // Copy raw floats into point cloud XYZI fields
        common::PointCartesian point_cache;
        std::vector<common::PointCartesian> point_cloud;
        point_cloud.reserve(number_of_floats / 4);
        for (std::size_t i = 0; i < number_of_floats; i += 4)
        {
            point_cache.x_m = data_buffer[i];
            point_cache.y_m = data_buffer[i + 1];
            point_cache.z_m = data_buffer[i + 2];
            point_cache.intensity = data_buffer[i + 3];
            point_cloud.push_back(point_cache);
        }

        input.close();

        return point_cloud;
    }

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

    std::vector<std::int64_t> timestamp_cache_;
    std::vector<PointCloud2> point_cloud_cache_;
    std::uint32_t current_index_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<PointCloud2>::SharedPtr cloud_publisher_;
};

int main(int argc, const char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::install_signal_handlers();

    const auto data_path = "/home/yevgeniy/Documents/GitHub/LiDAR-Camera-Fusion/a_kitti_dataset/"
                           "2011_09_26_drive_0013_sync/velodyne_points";

    try
    {
        rclcpp::spin(std::make_shared<DataReaderNode>(data_path));
    }
    catch (const std::exception &ex)
    {
        std::cerr << "Exception: " << ex.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "Unknown exception!" << std::endl;
    }

    rclcpp::shutdown();

    return 0;
}