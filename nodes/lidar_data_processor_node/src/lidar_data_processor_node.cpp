// Configuration
#include "processing_configuration.hpp"

// Processing
#include <lidar_processing_lib/segmentation/ransac_segmenter.hpp>

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
#include <cstring>    // std::memcpy
#include <exception>  // std::exception
#include <functional> // std::bind
#include <iostream>   // std::cerr
#include <string>     // std::string
#include <tuple>      // std::tuple
#include <vector>     // std::vector

class LidarDataProcessorNode final : public rclcpp::Node
{
  public:
    static constexpr std::uint32_t MAX_CLOUD_SIZE = 250'000U;

    using PointCloud2 = sensor_msgs::msg::PointCloud2;
    using PointField = sensor_msgs::msg::PointField;
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
    PointCloud2 unknown_cloud_;

    rclcpp::Publisher<PointCloud2>::SharedPtr publisher_ground_cloud_;
    PointCloud2 ground_cloud_;

    rclcpp::Publisher<PointCloud2>::SharedPtr publisher_obstacle_cloud_;
    PointCloud2 obstacle_cloud_;

    // Obstacle clusters
    rclcpp::Publisher<PointCloud2>::SharedPtr publisher_clustered_cloud_;
    PointCloud2 clustered_cloud_;

    // Polygonization publication
    rclcpp::Publisher<MarkerArray>::SharedPtr publisher_polygonized_cloud_;
    MarkerArray polygonized_cloud_;

    // Configuration
    ProcessingConfiguration processing_configuration_;

    // Processing
    lidar_processing_lib::segmentation::ISegmenter::UniquePtr segmenter_ptr_;
    pcl::PointCloud<pcl::PointXYZI> segmenter_input_cloud_;
    std::vector<data_types_lib::SegmentationLabel> segmentation_labels_;

    /// @brief Reserve memory.
    void initialize();
};

void LidarDataProcessorNode::initialize()
{
    unknown_cloud_.data.reserve(MAX_CLOUD_SIZE * sizeof(pcl::PointXYZRGB));
    ground_cloud_.data.reserve(MAX_CLOUD_SIZE * sizeof(pcl::PointXYZRGB));
    obstacle_cloud_.data.reserve(MAX_CLOUD_SIZE * sizeof(pcl::PointXYZRGB));
    clustered_cloud_.data.reserve(MAX_CLOUD_SIZE * sizeof(pcl::PointXYZRGB));

    // TODO: Reserve markers when used
    segmenter_input_cloud_.points.reserve(MAX_CLOUD_SIZE);
    segmentation_labels_.reserve(MAX_CLOUD_SIZE);

    // name, offset, type, count
    std::vector<std::tuple<std::string, std::uint32_t, std::uint8_t, std::uint32_t>> fields = {
        {"x", offsetof(pcl::PointXYZRGB, x), PointField::FLOAT32, 1},
        {"y", offsetof(pcl::PointXYZRGB, y), PointField::FLOAT32, 1},
        {"z", offsetof(pcl::PointXYZRGB, z), PointField::FLOAT32, 1},
        {"rgb", offsetof(pcl::PointXYZRGB, rgb), PointField::UINT32, 1}};

    const auto setFields = [&fields](PointCloud2 &cloud) -> void {
        for (const auto &field : fields)
        {
            sensor_msgs::msg::PointField field_cache;
            field_cache.name = std::get<0>(field);
            field_cache.offset = std::get<1>(field);
            field_cache.datatype = std::get<2>(field);
            field_cache.count = std::get<3>(field);
            cloud.fields.emplace_back(std::move(field_cache));
        }
    };

    setFields(unknown_cloud_);
    setFields(ground_cloud_);
    setFields(obstacle_cloud_);
    setFields(clustered_cloud_);

    unknown_cloud_.is_dense = true;
    unknown_cloud_.is_bigendian = false;
    unknown_cloud_.height = 1;
    unknown_cloud_.point_step = sizeof(pcl::PointXYZRGB);

    ground_cloud_.is_dense = true;
    ground_cloud_.is_bigendian = false;
    ground_cloud_.height = 1;
    ground_cloud_.point_step = sizeof(pcl::PointXYZRGB);

    obstacle_cloud_.is_dense = true;
    obstacle_cloud_.is_bigendian = false;
    obstacle_cloud_.height = 1;
    obstacle_cloud_.point_step = sizeof(pcl::PointXYZRGB);
}

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

    // Reserve memory
    initialize();

    // Choose segmentation algorithm
    if (processing_configuration_.segmentation.algorithm == "ransac")
    {
        segmenter_ptr_ = lidar_processing_lib::segmentation::ISegmenter::createUnique<
            lidar_processing_lib::segmentation::RansacSegmenter>(
            processing_configuration_.height_offset,
            processing_configuration_.segmentation.ransac.orthogonal_distance_threshold,
            processing_configuration_.segmentation.ransac.number_of_iterations,
            processing_configuration_.segmentation.ransac.thread_count);
    }
}

void LidarDataProcessorNode::run(const PointCloud2 &input_message)
{
    RCLCPP_INFO(this->get_logger(), "%s", "Received_message");

    // Transfer points from the message to the processing format
    pcl::PointXYZI point_cache;
    segmenter_input_cloud_.points.clear();
    for (std::uint32_t i = 0U; i < input_message.height; ++i)
    {
        const std::uint32_t row_offset = i * input_message.row_step;

        for (std::uint32_t j = 0U; j < input_message.width; ++j)
        {
            // Offset to the start of the point
            const std::uint32_t point_offset = row_offset + (j * input_message.point_step);

            // Copy the point data
            std::memcpy(&point_cache, &input_message.data[point_offset], input_message.point_step);
            segmenter_input_cloud_.points.push_back(point_cache);
        }
    }
    segmenter_input_cloud_.width = input_message.width;
    segmenter_input_cloud_.height = input_message.height;

    // Crop off point cloud to the bounding box

    // Segment
    segmenter_ptr_->run(segmenter_input_cloud_, segmentation_labels_);

    // Convert and publish
    pcl::PointXYZRGB unknown_point_cache{0.0F, 0.0F, 0.0F, 255U, 255U, 0U};  // Yellow
    pcl::PointXYZRGB ground_point_cache{0.0F, 0.0F, 0.0F, 220U, 220U, 220U}; // Ground
    pcl::PointXYZRGB obstacle_point_cache{0.0F, 0.0F, 0.0F, 0U, 255U, 0U};   // Obstacle

    unknown_cloud_.data.clear();
    ground_cloud_.data.clear();
    obstacle_cloud_.data.clear();

    std::uint32_t point_index = 0U;

    for (std::uint32_t i = 0U; i < input_message.height; ++i)
    {
        for (std::uint32_t j = 0U; j < input_message.width; ++j)
        {
            // Offset to the start of the point
            const auto &point = segmenter_input_cloud_.points[point_index];

            constexpr auto copyPoint = [](const pcl::PointXYZI &point, pcl::PointXYZRGB &point_cache,
                                          PointCloud2 &cloud) -> void {
                point_cache.x = point.x;
                point_cache.y = point.y;
                point_cache.z = point.z;
                const auto current_size = cloud.data.size();
                cloud.data.resize(current_size + sizeof(point_cache));
                std::memcpy(cloud.data.data() + current_size, &point_cache, sizeof(point_cache));
            };

            switch (segmentation_labels_[point_index])
            {
            case data_types_lib::SegmentationLabel::UNKNOWN: {
                copyPoint(point, unknown_point_cache, unknown_cloud_);
                break;
            }
            case data_types_lib::SegmentationLabel::GROUND: {
                copyPoint(point, ground_point_cache, ground_cloud_);
                break;
            }
            case data_types_lib::SegmentationLabel::OBSTACLE: {
                copyPoint(point, obstacle_point_cache, obstacle_cloud_);
                break;
            }
            case data_types_lib::SegmentationLabel::TRANSITIONAL_GROUND: {
                copyPoint(point, ground_point_cache, ground_cloud_);
                break;
            }
            case data_types_lib::SegmentationLabel::TRANSITIONAL_OBSTACLE: {
                copyPoint(point, obstacle_point_cache, obstacle_cloud_);
                break;
            }
            default: {
                RCLCPP_ERROR(this->get_logger(), "Provided invalid segmentation label: %u",
                             static_cast<std::uint32_t>(segmentation_labels_[point_index]));
                break;
            }
            }

            ++point_index;
        }
    }

    unknown_cloud_.width = unknown_cloud_.data.size() / sizeof(pcl::PointXYZRGB);
    unknown_cloud_.header = input_message.header;

    ground_cloud_.width = ground_cloud_.data.size() / sizeof(pcl::PointXYZRGB);
    ground_cloud_.header = input_message.header;

    obstacle_cloud_.width = obstacle_cloud_.data.size() / sizeof(pcl::PointXYZRGB);
    obstacle_cloud_.header = input_message.header;

    // Publish
    publisher_unknown_cloud_->publish(unknown_cloud_);
    publisher_ground_cloud_->publish(ground_cloud_);
    publisher_obstacle_cloud_->publish(obstacle_cloud_);
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
