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
#include <exception> // std::exception
#include <iostream>  // std::cerr

class SensorDataPublisherNode final : public rclcpp::Node
{
  public:
    using PointCloud2 = sensor_msgs::msg::PointCloud2;
    using Image = sensor_msgs::msg::Image;

    SensorDataPublisherNode(const std::filesystem::path &data_path, std::string lidar_topic = "lidar",
                            std::string camera_1_topic = "camera_1", std::string camera_2_topic = "camera_2",
                            std::string camera_3_topic = "camera_3", std::string camera_4_topic = "camera_4");

    ~SensorDataPublisherNode() = default;

    void loadCache();
    void replayData();

  private:
    template <typename MessageType> struct PublicationInfo final
    {
        std::vector<std::int64_t> timestamps{};
        std::vector<MessageType> messages{};
        typename rclcpp::Publisher<MessageType>::SharedPtr publisher{nullptr};
    };

    // Message cache
    PublicationInfo<PointCloud2> point_cloud_info_{};
    PublicationInfo<Image> camera_1_info_{};
    PublicationInfo<Image> camera_2_info_{};
    PublicationInfo<Image> camera_3_info_{};
    PublicationInfo<Image> camera_4_info_{};
};

SensorDataPublisherNode::SensorDataPublisherNode(const std::filesystem::path &data_path, std::string lidar_topic,
                                                 std::string camera_1_topic, std::string camera_2_topic,
                                                 std::string camera_3_topic, std::string camera_4_topic)
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
    point_cloud_info_.publisher = this->create_publisher<PointCloud2>(lidar_topic, qos);
    camera_1_info_.publisher = this->create_publisher<Image>(camera_1_topic, qos);
    camera_2_info_.publisher = this->create_publisher<Image>(camera_2_topic, qos);
    camera_3_info_.publisher = this->create_publisher<Image>(camera_3_topic, qos);
    camera_4_info_.publisher = this->create_publisher<Image>(camera_4_topic, qos);
}

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    rclcpp::install_signal_handlers();

    try
    {
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
