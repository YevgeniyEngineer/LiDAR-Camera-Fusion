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
#include <chrono>    // std::chrono
#include <exception> // std::exception
#include <iostream>  // std::cerr
#include <vector>    // std::vector

class DataProcessorNode final : public rclcpp::Node
{
  public:
    using PointCloud2 = sensor_msgs::msg::PointCloud2;
    using MarkerArray = visualization_msgs::msg::MarkerArray;

    // Copy and move operations are not allowed.
    DataProcessorNode(const DataProcessorNode &) = delete;
    DataProcessorNode(DataProcessorNode &&) = delete;
    DataProcessorNode &operator=(const DataProcessorNode &) = delete;
    DataProcessorNode &operator=(DataProcessorNode &&) = delete;

    /// @brief Constructor of the node.
    DataProcessorNode();

    /// @brief Destructor of the node.
    ~DataProcessorNode() = default;

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
};

DataProcessorNode::DataProcessorNode() : rclcpp::Node{"data_processor_node"}
{
}

void DataProcessorNode::run(const PointCloud2 &input_message)
{
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::install_signal_handlers();

    try
    {
        auto node = std::make_shared<DataProcessorNode>();
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
