// Configuration
#include "processing_configuration.hpp"

// ROS2
#include <rclcpp/executors.hpp>                    // rclcpp::spin
#include <rclcpp/node.hpp>                         // rclcpp::Node
#include <rclcpp/publisher.hpp>                    // rclcpp::Publisher
#include <rclcpp/qos.hpp>                          // rclcpp::QoS
#include <rclcpp/subscription.hpp>                 // rclcpp::Subscription
#include <rclcpp/timer.hpp>                        // rclcpp::TimerBase
#include <rclcpp/utilities.hpp>                    // rclcpp::shutdown
#include <sensor_msgs/msg/image.hpp>               // sensor_msgs::msg::Image
#include <sensor_msgs/msg/point_cloud2.hpp>        // sensor_msgs::msg::PointCloud2
#include <sensor_msgs/msg/point_field.hpp>         // sensor_msgs::msg::PointField
#include <visualization_msgs/msg/marker_array.hpp> // visualization_msgs::msg::MarkerArray

// STL
#include <chrono>     // std::chrono
#include <exception>  // std::exception
#include <functional> // std::bind
#include <iostream>   // std::cerr
#include <string>     // std::string
#include <vector>     // std::vector

class LidarDataProcessorNode final : public rclcpp::Node
{
  public:
    using PointCloud2 = sensor_msgs::msg::PointCloud2;
    using MarkerArray = visualization_msgs::msg::MarkerArray;

    // Copy and move operations are not allowed.
    LidarDataProcessorNode(const LidarDataProcessorNode &) = delete;
    LidarDataProcessorNode(LidarDataProcessorNode &&) = delete;
    LidarDataProcessorNode &operator=(const LidarDataProcessorNode &) = delete;
    LidarDataProcessorNode &operator=(LidarDataProcessorNode &&) = delete;

    /// @brief Constructor of the node.
    LidarDataProcessorNode();

    /// @brief Destructor of the node.
    ~LidarDataProcessorNode() = default;

    /// @brief Replay sensor data.
    void run(const PointCloud2 &input_message);

  private:
    // A subscriber that receives raw lidar data.
    rclcpp::Subscription<PointCloud2>::SharedPtr subscriber_;

    // Ground segmentation data publishers
    rclcpp::Publisher<PointCloud2>::SharedPtr publisher_unknown_cloud_;
    rclcpp::Publisher<PointCloud2>::SharedPtr publisher_ground_cloud_;
    rclcpp::Publisher<PointCloud2>::SharedPtr publisher_obstacle_cloud_;

    // Obstacle clusters
    rclcpp::Publisher<PointCloud2>::SharedPtr publisher_clustered_cloud_;

    // Polygonization publication
    rclcpp::Publisher<MarkerArray>::SharedPtr publisher_polygonized_cloud_;

    // Configuration
    ProcessingConfiguration processing_configuration_;
};

LidarDataProcessorNode::LidarDataProcessorNode() : rclcpp::Node{"data_processor_node"}
{
    // Declare parameters
    this->declare_parameter<std::string>("subscription_topics.input_cloud");
    this->declare_parameter<std::string>("publication_topics.segmentation.unknown_cloud");
    this->declare_parameter<std::string>("publication_topics.segmentation.ground_cloud");
    this->declare_parameter<std::string>("publication_topics.segmentation.obstacle_cloud");
    this->declare_parameter<std::string>("publication_topics.clustering.clustered_cloud");
    this->declare_parameter<std::string>("publication_topics.polygonization.polygonized_cloud");

    this->declare_parameter<double>("processing_configuration.height_offset");
    this->declare_parameter<std::vector<double>>("processing_configuration.bounding_box");
    this->declare_parameter<std::string>("processing_configuration.segmentation.algorithm");
    this->declare_parameter<double>("processing_configuration.segmentation.ransac.orthogonal_distance_threshold");
    this->declare_parameter<std::int64_t>("processing_configuration.segmentation.ransac.number_of_iterations");
    this->declare_parameter<std::int64_t>("processing_configuration.segmentation.ransac.thread_count");

    processing_configuration_.height_offset = this->get_parameter("processing_configuration.height_offset").as_double();

    const std::vector<double> bounding_box =
        this->get_parameter("processing_configuration.bounding_box").as_double_array();

    processing_configuration_.bounding_box[0].x = bounding_box[0];
    processing_configuration_.bounding_box[0].y = bounding_box[1];
    processing_configuration_.bounding_box[1].x = bounding_box[2];
    processing_configuration_.bounding_box[1].y = bounding_box[3];
    processing_configuration_.bounding_box[2].x = bounding_box[4];
    processing_configuration_.bounding_box[2].y = bounding_box[5];
    processing_configuration_.bounding_box[3].x = bounding_box[6];
    processing_configuration_.bounding_box[3].y = bounding_box[7];

    processing_configuration_.segmentation.algorithm =
        this->get_parameter("processing_configuration.segmentation.algorithm").as_string();

    processing_configuration_.segmentation.ransac.orthogonal_distance_threshold =
        this->get_parameter("processing_configuration.segmentation.ransac.orthogonal_distance_threshold").as_double();
    processing_configuration_.segmentation.ransac.number_of_iterations =
        this->get_parameter("processing_configuration.segmentation.ransac.number_of_iterations").as_int();
    processing_configuration_.segmentation.ransac.thread_count =
        this->get_parameter("processing_configuration.segmentation.ransac.thread_count").as_int();

    // QoS
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

    // Subscriber(s)
    subscriber_ =
        this->create_subscription<PointCloud2>(this->get_parameter("subscription_topics.input_cloud").as_string(), qos,
                                               std::bind(&LidarDataProcessorNode::run, this, std::placeholders::_1));

    // Publisher(s)
    publisher_unknown_cloud_ = this->create_publisher<PointCloud2>(
        this->get_parameter("publication_topics.segmentation.unknown_cloud").as_string(), qos);
    publisher_ground_cloud_ = this->create_publisher<PointCloud2>(
        this->get_parameter("publication_topics.segmentation.ground_cloud").as_string(), qos);
    publisher_obstacle_cloud_ = this->create_publisher<PointCloud2>(
        this->get_parameter("publication_topics.segmentation.obstacle_cloud").as_string(), qos);
    publisher_clustered_cloud_ = this->create_publisher<PointCloud2>(
        this->get_parameter("publication_topics.clustering.clustered_cloud").as_string(), qos);
    publisher_polygonized_cloud_ = this->create_publisher<MarkerArray>(
        this->get_parameter("publication_topics.polygonization.polygonized_cloud").as_string(), qos);
}

void LidarDataProcessorNode::run(const PointCloud2 &input_message)
{
    RCLCPP_INFO(this->get_logger(), "%s", "Received_message");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::install_signal_handlers();

    try
    {
        auto node = std::make_shared<LidarDataProcessorNode>();
        rclcpp::spin(node);
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
