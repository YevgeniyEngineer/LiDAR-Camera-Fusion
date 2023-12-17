// Local
#include <data_types/cartesian_return.hpp>
#include <utilities/file_operations.hpp>

// ROS2
#include <rclcpp/executors.hpp>             // rclcpp::spin
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

class SensorDataPublisherNode final : public rclcpp::Node
{
    // Forward declaration
    template <typename MessageType> struct PublicationInfo;

  public:
    using PointCloud2 = sensor_msgs::msg::PointCloud2;
    using Image = sensor_msgs::msg::Image;

    /// @brief Constructor of the node.
    /// @param data_paths containing a tuple of <sensor id, data path, topic name>
    SensorDataPublisherNode(
        const std::vector<std::tuple<std::int32_t, std::filesystem::path, std::string>> &data_paths);

    /// @brief Default destructor.
    ~SensorDataPublisherNode() = default;

    /// @brief Load point cloud messages.
    void loadCache(const std::filesystem::path &data_path, PublicationInfo<PointCloud2> &info);

    /// @brief Load camera messages.
    void loadCache(const std::filesystem::path &data_path, PublicationInfo<Image> &info);

    /// @brief Replay sensor data.
    void replayData();

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

    /// @brief Helper method to get the earliest timestamp.
    template <typename MessageType> std::int64_t getEarliestTimestamp(const PublicationInfo<MessageType> &info) const;

    /// @brief Shifts timestamp of each sensor to the common time point.
    template <typename MessageType> void shiftTimestamps(PublicationInfo<MessageType> &info, std::int64_t time_offset);

    /// @brief Check iterator and publish.
    /// @returns True if message was published.
    template <typename MessageType>
    bool tryPublishMessage(PublicationInfo<MessageType> &info,
                           typename std::vector<std::pair<std::int64_t, MessageType>>::iterator &message_iterator,
                           const std::int64_t elapsed_time);
};

SensorDataPublisherNode::SensorDataPublisherNode(
    const std::vector<std::tuple<std::int32_t, std::filesystem::path, std::string>> &data_paths)
    : rclcpp::Node{"sensor_data_publisher_node"}
{
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

    // Create publishers
    for (const auto &[sensor_id, data_path, topic_name] : data_paths)
    {
        switch (sensor_id)
        {
        case 0: {
            loadCache(data_path, point_cloud_info_);
            point_cloud_info_.publisher = this->create_publisher<PointCloud2>(topic_name, qos);
            break;
        }
        case 1: {
            loadCache(data_path, camera_1_info_);
            camera_1_info_.publisher = this->create_publisher<Image>(topic_name, qos);
            break;
        }
        case 2: {
            loadCache(data_path, camera_2_info_);
            camera_2_info_.publisher = this->create_publisher<Image>(topic_name, qos);
            break;
        }
        case 3: {
            loadCache(data_path, camera_3_info_);
            camera_3_info_.publisher = this->create_publisher<Image>(topic_name, qos);
            break;
        }
        case 4: {
            loadCache(data_path, camera_4_info_);
            camera_4_info_.publisher = this->create_publisher<Image>(topic_name, qos);
            break;
        }
        }
    }

    // Replay data indefinitely
    replayData();
}

void SensorDataPublisherNode::loadCache(const std::filesystem::path &data_path, PublicationInfo<PointCloud2> &info)
{
    // TODO
}

void SensorDataPublisherNode::loadCache(const std::filesystem::path &data_path, PublicationInfo<Image> &info)
{
    // TODO
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
    const std::int64_t elapsed_time)
{
    bool published = false;

    if ((message_iterator != info.stamped_messages.end()) &&
        (elapsed_time >= info.stamped_messages[std::distance(info.stamped_messages.begin(), message_iterator)].first))
    {
        info.publisher->publish(message_iterator->second);
        ++message_iterator;
        published = true;
    }

    return published;
}

void SensorDataPublisherNode::replayData()
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
        const auto current_time = std::chrono::steady_clock::now();
        const auto elapsed_time =
            std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();

        // Publish next message from each sensor if the timestamp is reached
        const bool point_cloud_message_published =
            tryPublishMessage(point_cloud_info_, point_cloud_iterator, elapsed_time);

        const bool camera_1_message_published = tryPublishMessage(camera_1_info_, camera_1_iterator, elapsed_time);
        const bool camera_2_message_published = tryPublishMessage(camera_2_info_, camera_2_iterator, elapsed_time);
        const bool camera_3_message_published = tryPublishMessage(camera_3_info_, camera_3_iterator, elapsed_time);
        const bool camera_4_message_published = tryPublishMessage(camera_4_info_, camera_4_iterator, elapsed_time);

        // If all messages were published, wraparound
        if (point_cloud_message_published && camera_1_message_published && camera_2_message_published &&
            camera_3_message_published && camera_4_message_published)
        {
            point_cloud_iterator = point_cloud_info_.stamped_messages.begin();
            camera_1_iterator = camera_1_info_.stamped_messages.begin();
            camera_2_iterator = camera_2_info_.stamped_messages.begin();
            camera_3_iterator = camera_3_info_.stamped_messages.begin();
            camera_4_iterator = camera_4_info_.stamped_messages.begin();
        }

        rclcpp::spin_some(shared_from_this());
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

int main(int argc, char **argv)
{
    if (argc < 11)
    {
        std::cout << "Usage: " << argv[0]
                  << " <lidar data path> <camera 1 data path> <camera 2 data path> <camera 3 data path> <camera 4 data "
                     "path> <point cloud topic> <camera 1 topic> <camera 2 topic> <camera 3 topic> <camera 4 topic>"
                  << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);
    rclcpp::install_signal_handlers();

    const std::filesystem::path lidar_data_path{argv[1]};
    const std::filesystem::path camera_1_data_path{argv[2]};
    const std::filesystem::path camera_2_data_path{argv[3]};
    const std::filesystem::path camera_3_data_path{argv[4]};
    const std::filesystem::path camera_4_data_path{argv[5]};

    const std::string point_cloud_topic{argv[6]};
    const std::string camera_1_topic{argv[7]};
    const std::string camera_2_topic{argv[8]};
    const std::string camera_3_topic{argv[9]};
    const std::string camera_4_topic{argv[10]};

    try
    {
        const std::vector<std::tuple<std::int32_t, std::filesystem::path, std::string>> data_paths{
            {0, lidar_data_path, point_cloud_topic},
            {1, camera_1_data_path, camera_1_topic},
            {2, camera_2_data_path, camera_2_topic},
            {3, camera_3_data_path, camera_3_topic},
            {4, camera_4_data_path, camera_4_topic}};

        rclcpp::spin(std::make_shared<SensorDataPublisherNode>(data_paths));
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
