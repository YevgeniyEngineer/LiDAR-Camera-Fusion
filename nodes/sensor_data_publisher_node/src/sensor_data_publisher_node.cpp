/// Local
#include <data_types_lib/cartesian_return.hpp>
#include <utilities_lib/file_operations.hpp>

// ROS2
#include <rclcpp/executors.hpp>             // rclcpp::spin
#include <rclcpp/logging.hpp>               // RCLCPP_INFO
#include <rclcpp/node.hpp>                  // rclcpp::Node
#include <rclcpp/publisher.hpp>             // rclcpp::Publisher
#include <rclcpp/qos.hpp>                   // rclcpp::QoS
#include <rclcpp/timer.hpp>                 // rclcpp::TimerBase
#include <rclcpp/utilities.hpp>             // rclcpp::shutdown
#include <sensor_msgs/msg/image.hpp>        // sensor_msgs::msg::Image
#include <sensor_msgs/msg/point_cloud2.hpp> // sensor_msgs::msg::PointCloud2
#include <sensor_msgs/msg/point_field.hpp>  // sensor_msgs::msg::PointField

// STL
#include <chrono>    // std::chrono
#include <exception> // std::exception
#include <iostream>  // std::cerr
#include <iterator>  // std::distance
#include <limits>    // std::numeric_limits
#include <thread>    // std::this_thread
#include <tuple>     // std::tuple
#include <utility>   // std::pair

// OpenCV
#include <opencv2/opencv.hpp> // cv::

class SensorDataPublisherNode final : public rclcpp::Node
{
    // Forward declaration
    template <typename MessageType> struct PublicationInfo;

  public:
    using PointCloud2 = sensor_msgs::msg::PointCloud2;
    using Image = sensor_msgs::msg::Image;

    // Copy and move operations are not allowed.
    SensorDataPublisherNode(const SensorDataPublisherNode &) = delete;
    SensorDataPublisherNode(SensorDataPublisherNode &&) = delete;
    SensorDataPublisherNode &operator=(const SensorDataPublisherNode &) = delete;
    SensorDataPublisherNode &operator=(SensorDataPublisherNode &&) = delete;

    /// @brief Constructor of the node.
    SensorDataPublisherNode();

    /// @brief Default destructor.
    ~SensorDataPublisherNode() = default;

    /// @brief Replay sensor data.
    void run();

  private:
    template <typename MessageType> struct PublicationInfo final
    {
        // Messages containing <time, message>
        std::vector<std::pair<std::int64_t, MessageType>> stamped_messages;

        // Data publisher for MessageType
        typename rclcpp::Publisher<MessageType>::SharedPtr publisher{nullptr};
    };

    // Message cache
    PublicationInfo<PointCloud2> point_cloud_info_{};
    PublicationInfo<Image> camera_1_info_{};
    PublicationInfo<Image> camera_2_info_{};
    PublicationInfo<Image> camera_3_info_{};
    PublicationInfo<Image> camera_4_info_{};

    /// @brief Load point cloud messages.
    void loadCache(const std::filesystem::path &data_path, const std::string &topic_name,
                   PublicationInfo<PointCloud2> &info);

    /// @brief Load camera messages.
    void loadCache(const std::filesystem::path &data_path, const std::string &topic_name, PublicationInfo<Image> &info);

    /// @brief Helper method to get the earliest timestamp.
    template <typename MessageType> std::int64_t getEarliestTimestamp(const PublicationInfo<MessageType> &info) const;

    /// @brief Shifts timestamp of each sensor to the common time point.
    template <typename MessageType> void shiftTimestamps(PublicationInfo<MessageType> &info, std::int64_t time_offset);

    /// @brief Check iterator and publish.
    /// @returns True if message was published.
    template <typename MessageType>
    bool tryPublishMessage(PublicationInfo<MessageType> &info,
                           typename std::vector<std::pair<std::int64_t, MessageType>>::iterator &message_iterator,
                           const std::int64_t elapsed_time_point);
};

SensorDataPublisherNode::SensorDataPublisherNode() : rclcpp::Node{"sensor_data_publisher_node"}
{
    // Declare parameters
    this->declare_parameter<std::string>("lidar.data_path");
    this->declare_parameter<std::string>("lidar.topic");
    this->declare_parameter<std::int32_t>("lidar.sensor_id");

    this->declare_parameter<std::string>("camera_1.data_path");
    this->declare_parameter<std::string>("camera_1.topic");
    this->declare_parameter<std::int32_t>("camera_1.sensor_id");

    this->declare_parameter<std::string>("camera_2.data_path");
    this->declare_parameter<std::string>("camera_2.topic");
    this->declare_parameter<std::int32_t>("camera_2.sensor_id");

    this->declare_parameter<std::string>("camera_3.data_path");
    this->declare_parameter<std::string>("camera_3.topic");
    this->declare_parameter<std::int32_t>("camera_3.sensor_id");

    this->declare_parameter<std::string>("camera_4.data_path");
    this->declare_parameter<std::string>("camera_4.topic");
    this->declare_parameter<std::int32_t>("camera_4.sensor_id");

    // Specify QoS
    rclcpp::QoS qos{2};
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

    // LiDAR publisher
    try
    {
        loadCache(this->get_parameter("lidar.data_path").as_string(), this->get_parameter("lidar.topic").as_string(),
                  point_cloud_info_);

        point_cloud_info_.publisher =
            this->create_publisher<PointCloud2>(this->get_parameter("lidar.topic").as_string(), qos);

        RCLCPP_INFO(this->get_logger(), "%s", "Created LiDAR publisher.");
    }
    catch (const std::exception &ex)
    {
        RCLCPP_INFO(this->get_logger(), "Exception: %s", ex.what());
    }

    // Camera 1 publisher
    try
    {
        loadCache(this->get_parameter("camera_1.data_path").as_string(),
                  this->get_parameter("camera_1.topic").as_string(), camera_1_info_);

        camera_1_info_.publisher =
            this->create_publisher<Image>(this->get_parameter("camera_1.topic").as_string(), qos);

        RCLCPP_INFO(this->get_logger(), "%s", "Created Camera 1 publisher.");
    }
    catch (const std::exception &ex)
    {
        RCLCPP_INFO(this->get_logger(), "Exception: %s", ex.what());
    }

    // Camera 2 publisher
    try
    {
        loadCache(this->get_parameter("camera_2.data_path").as_string(),
                  this->get_parameter("camera_2.topic").as_string(), camera_2_info_);

        camera_2_info_.publisher =
            this->create_publisher<Image>(this->get_parameter("camera_2.topic").as_string(), qos);

        RCLCPP_INFO(this->get_logger(), "%s", "Created Camera 2 publisher.");
    }
    catch (const std::exception &ex)
    {
        RCLCPP_INFO(this->get_logger(), "Exception: %s", ex.what());
    }

    // Camera 3 publisher
    try
    {
        loadCache(this->get_parameter("camera_3.data_path").as_string(),
                  this->get_parameter("camera_3.topic").as_string(), camera_3_info_);

        camera_3_info_.publisher =
            this->create_publisher<Image>(this->get_parameter("camera_3.topic").as_string(), qos);

        RCLCPP_INFO(this->get_logger(), "%s", "Created Camera 3 publisher.");
    }
    catch (const std::exception &ex)
    {
        RCLCPP_INFO(this->get_logger(), "Exception: %s", ex.what());
    }

    // Camera 4 publisher
    try
    {
        loadCache(this->get_parameter("camera_4.data_path").as_string(),
                  this->get_parameter("camera_4.topic").as_string(), camera_4_info_);

        camera_4_info_.publisher =
            this->create_publisher<Image>(this->get_parameter("camera_4.topic").as_string(), qos);

        RCLCPP_INFO(this->get_logger(), "%s", "Created Camera 4 publisher.");
    }
    catch (const std::exception &ex)
    {
        RCLCPP_INFO(this->get_logger(), "Exception: %s", ex.what());
    }
}

void SensorDataPublisherNode::loadCache(const std::filesystem::path &data_path, const std::string &topic_name,
                                        PublicationInfo<PointCloud2> &info)
{
    // Check if the directory exists
    if (!std::filesystem::exists(data_path))
    {
        throw std::runtime_error("Specified data path " + data_path.string() + " does not exist.");
    }

    // Load timestamps
    std::vector<std::int64_t> timestamps;
    const auto timestamps_path = data_path / "timestamps.txt";
    utilities_lib::readTimestampsFromTxtFile(timestamps_path, timestamps);
    timestamps.shrink_to_fit();

    // Check if timestamps were loaded
    if (!timestamps.empty())
    {
        // Reserve memory for stamped messages
        info.stamped_messages.reserve(timestamps.size());

        // Read data files
        std::vector<std::filesystem::path> file_paths;
        file_paths.reserve(timestamps.size());

        const auto data_folder = data_path / "data";
        utilities_lib::readFileNamesWithExtensionFromDirectory(data_folder, ".bin", file_paths);

        // Check the number of files match the number of timestamps
        if (file_paths.size() != timestamps.size())
        {
            throw std::runtime_error("The number of messages does not match the number of timestamps.");
        }

        // Fields to be used when populating messages
        // <field name, field offset, field type, field count>
        const std::vector<std::tuple<std::string, std::uint32_t, std::uint8_t, std::uint32_t>> fields = {
            {"x", offsetof(data_types_lib::CartesianReturn, x), sensor_msgs::msg::PointField::FLOAT32, 1},
            {"y", offsetof(data_types_lib::CartesianReturn, y), sensor_msgs::msg::PointField::FLOAT32, 1},
            {"z", offsetof(data_types_lib::CartesianReturn, z), sensor_msgs::msg::PointField::FLOAT32, 1},
            {"intensity", offsetof(data_types_lib::CartesianReturn, intensity), sensor_msgs::msg::PointField::FLOAT32,
             1}};

        // Aggregate messages into cache
        point_cloud_info_.stamped_messages.reserve(timestamps.size());
        for (std::size_t message_number = 0U; message_number < timestamps.size(); ++message_number)
        {
            const auto &timestamp = timestamps[message_number];
            const auto &file_path = file_paths[message_number];

            // Read lidar cloud
            std::vector<data_types_lib::CartesianReturn> point_cloud_data;
            utilities_lib::loadPointCloudDataFromBinFile(file_path, point_cloud_data);

            if (point_cloud_data.empty())
            {
                std::cerr << "Empty binary file, skipping." << std::endl;
                continue;
            }

            // Construct output message
            PointCloud2 point_cloud_message;
            point_cloud_message.height = 1;
            point_cloud_message.width = point_cloud_data.size();
            point_cloud_message.is_bigendian = false;
            point_cloud_message.point_step = sizeof(data_types_lib::CartesianReturn);
            point_cloud_message.row_step = sizeof(data_types_lib::CartesianReturn) * point_cloud_data.size();
            point_cloud_message.is_dense = true;
            point_cloud_message.header.frame_id = topic_name;

            // Set timestamp
            point_cloud_message.header.stamp.sec = static_cast<std::int32_t>(static_cast<double>(timestamp) * 1e-9);
            point_cloud_message.header.stamp.nanosec = static_cast<std::uint32_t>(
                timestamp - static_cast<std::int64_t>(static_cast<double>(point_cloud_message.header.stamp.sec) * 1e9));

            // Populate fields
            point_cloud_message.fields.reserve(fields.size());
            for (const auto &field : fields)
            {
                sensor_msgs::msg::PointField field_cache;
                field_cache.name = std::get<0>(field);
                field_cache.offset = std::get<1>(field);
                field_cache.datatype = std::get<2>(field);
                field_cache.count = std::get<3>(field);
                point_cloud_message.fields.push_back(std::move(field_cache));
            }

            // Copy the point cloud data
            point_cloud_message.data.resize(sizeof(data_types_lib::CartesianReturn) * point_cloud_data.size());
            std::memcpy(point_cloud_message.data.data(), point_cloud_data.data(),
                        sizeof(data_types_lib::CartesianReturn) * point_cloud_data.size());

            // Add message to the message cache
            point_cloud_info_.stamped_messages.emplace_back(timestamp, std::move(point_cloud_message));
        }
    }
}

void SensorDataPublisherNode::loadCache(const std::filesystem::path &data_path, const std::string &topic_name,
                                        PublicationInfo<Image> &info)
{
    // Check if the directory exists
    if (!std::filesystem::exists(data_path))
    {
        throw std::runtime_error("Specified data path " + data_path.string() + " does not exist.");
    }

    // Load timestamps
    std::vector<std::int64_t> timestamps;
    const auto timestamps_path = data_path / "timestamps.txt";
    utilities_lib::readTimestampsFromTxtFile(timestamps_path, timestamps);
    timestamps.shrink_to_fit();

    // Check if timestamps were loaded
    if (!timestamps.empty())
    {
        // Reserve memory for stamped messages
        info.stamped_messages.reserve(timestamps.size());

        // Read data files
        std::vector<std::filesystem::path> file_paths;
        file_paths.reserve(timestamps.size());

        const auto data_folder = data_path / "data";
        utilities_lib::readFileNamesWithExtensionFromDirectory(data_folder, ".png", file_paths);

        // Check the number of files match the number of timestamps
        if (file_paths.size() != timestamps.size())
        {
            throw std::runtime_error("The number of messages does not match the number of timestamps: " +
                                     std::to_string(file_paths.size()) + " vs " + std::to_string(timestamps.size()));
        }

        // Aggregate messages into cache
        point_cloud_info_.stamped_messages.reserve(timestamps.size());
        for (std::size_t message_number = 0U; message_number < timestamps.size(); ++message_number)
        {
            const auto &timestamp = timestamps[message_number];

            // Read data from the file
            const cv::Mat image = cv::imread(file_paths[message_number], cv::IMREAD_COLOR);

            // Convert image into image message format
            Image image_message;

            image_message.header.frame_id = topic_name;
            image_message.height = image.rows;
            image_message.width = image.cols;
            image_message.encoding = "bgr8"; // For color image, adjust if using grayscale or other types
            image_message.is_bigendian = false;
            image_message.step = image.step;

            // Set timestamp
            image_message.header.stamp.sec = static_cast<std::int32_t>(static_cast<double>(timestamp) * 1e-9);
            image_message.header.stamp.nanosec = static_cast<std::uint32_t>(
                timestamp - static_cast<std::int64_t>(static_cast<double>(image_message.header.stamp.sec) * 1e9));

            // Copy byte data
            image_message.data.resize(image_message.step * image_message.height);
            std::memcpy(&image_message.data[0], image.data, image_message.step * image_message.height);

            // Add message to the message cache
            info.stamped_messages.emplace_back(timestamp, std::move(image_message));
        }
    }
}

template <typename MessageType>
std::int64_t SensorDataPublisherNode::getEarliestTimestamp(const PublicationInfo<MessageType> &info) const
{
    return info.stamped_messages.empty() ? std::numeric_limits<std::int64_t>::max()
                                         : info.stamped_messages.front().first;
}

template <typename MessageType>
void SensorDataPublisherNode::shiftTimestamps(PublicationInfo<MessageType> &info, std::int64_t time_offset)
{
    for (auto &message : info.stamped_messages)
    {
        message.first += time_offset;
    }
}

template <typename MessageType>
bool SensorDataPublisherNode::tryPublishMessage(
    PublicationInfo<MessageType> &info,
    typename std::vector<std::pair<std::int64_t, MessageType>>::iterator &message_iterator,
    const std::int64_t elapsed_time_point)
{
    bool published = false;

    if (message_iterator != info.stamped_messages.end())
    {
        if (elapsed_time_point >=
            info.stamped_messages[std::distance(info.stamped_messages.begin(), message_iterator)].first)
        {
            // Update timestamp
            message_iterator->second.header.stamp.sec =
                static_cast<std::int32_t>(static_cast<double>(elapsed_time_point) * 1e-9);
            message_iterator->second.header.stamp.nanosec = static_cast<std::uint32_t>(
                elapsed_time_point -
                static_cast<std::int64_t>(static_cast<double>(message_iterator->second.header.stamp.sec) * 1e9));

            // Publish
            info.publisher->publish(message_iterator->second);
            ++message_iterator;
            published = true;
        }
    }

    return published;
}

void SensorDataPublisherNode::run()
{
    // Initialize iterators for each sensor's messages
    auto point_cloud_iterator = point_cloud_info_.stamped_messages.begin();
    auto camera_1_iterator = camera_1_info_.stamped_messages.begin();
    auto camera_2_iterator = camera_2_info_.stamped_messages.begin();
    auto camera_3_iterator = camera_3_info_.stamped_messages.begin();
    auto camera_4_iterator = camera_4_info_.stamped_messages.begin();

    // Find the earliest timestamp across all sensors
    const std::int64_t earliest_timestamp_nanosec =
        std::min({getEarliestTimestamp(point_cloud_info_), getEarliestTimestamp(camera_1_info_),
                  getEarliestTimestamp(camera_2_info_), getEarliestTimestamp(camera_3_info_),
                  getEarliestTimestamp(camera_4_info_)});

    // Get the start time
    const auto start_time = std::chrono::steady_clock::now();

    // Shift timestamps for all sensors towards the earliest timestamp
    const auto start_time_nanosec =
        std::chrono::duration_cast<std::chrono::nanoseconds>(start_time.time_since_epoch()).count();

    const std::int64_t time_offset = start_time_nanosec - earliest_timestamp_nanosec;

    shiftTimestamps(point_cloud_info_, time_offset);
    shiftTimestamps(camera_1_info_, time_offset);
    shiftTimestamps(camera_2_info_, time_offset);
    shiftTimestamps(camera_3_info_, time_offset);
    shiftTimestamps(camera_4_info_, time_offset);

    // Publish indefinitely
    while (rclcpp::ok())
    {
        const auto current_time_point_nanosec =
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
                .count();

        // Publish next message from each sensor if the timestamp is reached
        const bool point_cloud_message_published =
            tryPublishMessage(point_cloud_info_, point_cloud_iterator, current_time_point_nanosec);
        const bool camera_1_message_published =
            tryPublishMessage(camera_1_info_, camera_1_iterator, current_time_point_nanosec);
        const bool camera_2_message_published =
            tryPublishMessage(camera_2_info_, camera_2_iterator, current_time_point_nanosec);
        const bool camera_3_message_published =
            tryPublishMessage(camera_3_info_, camera_3_iterator, current_time_point_nanosec);
        const bool camera_4_message_published =
            tryPublishMessage(camera_4_info_, camera_4_iterator, current_time_point_nanosec);

        // If all messages were published, wraparound
        if ((point_cloud_iterator == point_cloud_info_.stamped_messages.end()) &&
            (camera_1_iterator == camera_1_info_.stamped_messages.end()) &&
            (camera_2_iterator == camera_2_info_.stamped_messages.end()) &&
            (camera_3_iterator == camera_3_info_.stamped_messages.end()) &&
            (camera_4_iterator == camera_4_info_.stamped_messages.end()))
        {
            // Reset iterators
            point_cloud_iterator = point_cloud_info_.stamped_messages.begin();
            camera_1_iterator = camera_1_info_.stamped_messages.begin();
            camera_2_iterator = camera_2_info_.stamped_messages.begin();
            camera_3_iterator = camera_3_info_.stamped_messages.begin();
            camera_4_iterator = camera_4_info_.stamped_messages.begin();

            // Recalculate earliest time point
            const std::int64_t earliest_timestamp_nanosec =
                std::min({getEarliestTimestamp(point_cloud_info_), getEarliestTimestamp(camera_1_info_),
                          getEarliestTimestamp(camera_2_info_), getEarliestTimestamp(camera_3_info_),
                          getEarliestTimestamp(camera_4_info_)});

            // Shift timestamps to the latest time point
            const std::int64_t time_offset = current_time_point_nanosec - earliest_timestamp_nanosec;

            shiftTimestamps(point_cloud_info_, time_offset);
            shiftTimestamps(camera_1_info_, time_offset);
            shiftTimestamps(camera_2_info_, time_offset);
            shiftTimestamps(camera_3_info_, time_offset);
            shiftTimestamps(camera_4_info_, time_offset);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::install_signal_handlers();

    try
    {
        auto node = std::make_shared<SensorDataPublisherNode>();
        node->run();
    }
    catch (const std::exception &ex)
    {
        std::cerr << "Exception: " << ex.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "Unknown exception." << std::endl;
    }

    rclcpp::shutdown();

    return 0;
}
