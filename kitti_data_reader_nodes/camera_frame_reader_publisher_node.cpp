// Local
#include "filename_reader.hpp"
#include "synchronization_time.hpp"
#include "timestamp_reader.hpp"

// STL
#include <chrono>     // std::chrono
#include <cstdint>    // std::uint32_t
#include <cstring>    // std::memcpy
#include <filesystem> // std::filesystem
#include <fstream>
#include <functional> // std::bind
#include <iomanip>
#include <memory>    // std::shared_ptr
#include <stdexcept> // std::runtime_error
#include <string>    // std::string
#include <thread>
#include <tuple>   // std::tuple
#include <utility> // std::move
#include <vector>  // std::vector

// ROS2
#include <rclcpp/executors.hpp> // rclcpp::spin
#include <rclcpp/node.hpp>      // rclcpp::Node
#include <rclcpp/publisher.hpp> // rclcpp::Publisher
#include <rclcpp/qos.hpp>       // rclcpp::QoS
#include <rclcpp/timer.hpp>     // rclcpp::TimerBase
#include <rclcpp/utilities.hpp> // rclcpp::shutdown
#include <sensor_msgs/msg/image.hpp>

// OpenCV
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class CameraFrameReaderPublisherNode : public rclcpp::Node
{
  public:
    using Image = sensor_msgs::msg::Image;

    CameraFrameReaderPublisherNode(const std::filesystem::path &data_path, const std::filesystem::path &camera_path,
                                   std::string topic = "camera_frame")
        : Node("camera_frame_reader_publisher_node"), image_message_publisher_(nullptr)
    {
        auto now = std::chrono::system_clock::now();

        // Check if directories exist
        if (!std::filesystem::exists(data_path))
        {
            throw std::runtime_error("Specified data path does not exist.");
        }

        std::filesystem::path timestamps_file = (data_path / camera_path) / "timestamps.txt";
        if (!std::filesystem::exists(timestamps_file))
        {
            throw std::runtime_error("Timestamp data file timestamps.txt was not found.");
        }

        std::filesystem::path image_data_path = (data_path / camera_path) / "data";
        if (!std::filesystem::exists(image_data_path))
        {
            throw std::runtime_error("Data path containing *.png files was not found..");
        }

        // Accumulate timestamps
        timestamp_cache_ = readTimestamps(timestamps_file);
        // std::cout << "Read " << timestamp_cache_.size() << " timestamps\n";

        // Read file names in ascending order
        const auto data_files = readFilenames(image_data_path, ".png");
        // std::cout << "Read " << data_files.size() << " data files\n";

        // Load images into memory
        image_message_cache_.reserve(timestamp_cache_.size());
        for (std::size_t i = 0; i < timestamp_cache_.size(); ++i)
        {
            // Read data from the file
            cv::Mat image = cv::imread(data_files[i], cv::IMREAD_COLOR);

            // Convert image into image message format
            Image image_message;

            image_message.header.frame_id = "camera_frame";
            image_message.height = image.rows;
            image_message.width = image.cols;
            image_message.encoding = "bgr8"; // For color image, adjust if using grayscale or other types
            image_message.is_bigendian = false;
            image_message.step = image.step;

            // Set timestamp
            image_message.header.stamp.sec = static_cast<std::int32_t>(static_cast<double>(timestamp_cache_[i]) * 1e-9);
            image_message.header.stamp.nanosec = static_cast<std::uint32_t>(
                timestamp_cache_[i] -
                static_cast<std::int64_t>(static_cast<double>(image_message.header.stamp.sec) * 1e9));

            // Copy byte data
            image_message.data.resize(image_message.step * image_message.height);
            std::memcpy(&image_message.data[0], image.data, image_message.step * image_message.height);

            // Add message to the message cache
            image_message_cache_.push_back(std::move(image_message));
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

        // Create publisher for Image message type
        image_message_publisher_ = this->create_publisher<Image>(topic, qos);

        // Sleep for synchronization
        std::this_thread::sleep_until(now + SYNCHRONIZATION_TIME);

        // Create a timer callback based on aggregated timestamps
        updateTimerAndPublish(std::chrono::nanoseconds(timestamp_cache_[1] - timestamp_cache_[0]));
    }

  private:
    void updateTimerAndPublish(std::chrono::nanoseconds interval)
    {
        // Create a new timer
        timer_ = this->create_wall_timer(interval, [this]() {
            // Is index valid?
            if (current_index_ == image_message_cache_.size())
            {
                // wraparound
                current_index_ = 0U;
            }

            // Publish a message
            image_message_publisher_->publish(image_message_cache_[current_index_]);

            // Create a new timer
            // Sleep 100ms at the boundary
            std::chrono::nanoseconds sleep_duration_nanosec = 100'000'000ns;

            if (current_index_ + 1 != timestamp_cache_.size())
            {
                sleep_duration_nanosec =
                    std::chrono::nanoseconds(timestamp_cache_[current_index_ + 1] - timestamp_cache_[current_index_]);
            }

            ++current_index_;
            updateTimerAndPublish(std::move(sleep_duration_nanosec));
        });
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<Image>::SharedPtr image_message_publisher_;
    std::size_t current_index_;
    std::vector<std::int64_t> timestamp_cache_;
    std::vector<Image> image_message_cache_;
};

int main(int argc, const char **const argv)
{
    if (argc < 4)
    {
        std::cout << "Usage: " << argv[0] << " <data_path> <file_path> <topic>" << std::endl;
        return 1;
    }

    const char *const data_path = argv[1];
    const char *const file_path = argv[2];
    const char *const topic = argv[3];

    rclcpp::init(argc, argv);
    rclcpp::install_signal_handlers();

    try
    {
        rclcpp::spin(std::make_shared<CameraFrameReaderPublisherNode>(data_path, file_path, topic));
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