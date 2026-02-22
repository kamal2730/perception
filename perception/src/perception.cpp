#include "perception/perception.hpp"

using std::placeholders::_1;

PerceptionNode::PerceptionNode()
    : Node("perception_node"),
      snapshot_available_(false)
{
    // Subscribers
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/zed/zed_node/point_cloud/cloud_registered",
        10,
        std::bind(&PerceptionNode::pointCloudCallback, this, _1));

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/zed/zed_node/rgb/color/rect/image",
        10,
        std::bind(&PerceptionNode::imageCallback, this, _1));

    trigger_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/perception/trigger",
        10,
        std::bind(&PerceptionNode::triggerCallback, this, _1));

    // Publishers (fixed spelling)
    debug_pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/perception/debug/pointcloud", 10);

    debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/perception/debug/rgb_image", 10);

    // Timer to continuously publish stored data (10 Hz)
    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&PerceptionNode::publishStoredData, this));

    RCLCPP_INFO(this->get_logger(), "Perception node initialized");
}

void PerceptionNode::pointCloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    stored_pointcloud_ = *msg;   // store latest
}

void PerceptionNode::imageCallback(
    const sensor_msgs::msg::Image::SharedPtr msg)
{
    stored_image_ = *msg;   // store latest
}

void PerceptionNode::triggerCallback(
    const std_msgs::msg::Bool::SharedPtr msg)
{
    if (!msg->data)
        return;

    if (stored_pointcloud_.data.empty() || stored_image_.data.empty())
    {
        RCLCPP_WARN(this->get_logger(), "No data received yet.");
        return;
    }

    snapshot_available_ = true;

    RCLCPP_INFO(this->get_logger(),
                "Snapshot stored. Now continuously publishing.");
}

void PerceptionNode::publishStoredData()
{
    if (!snapshot_available_)
        return;

    debug_pointcloud_pub_->publish(stored_pointcloud_);
    debug_image_pub_->publish(stored_image_);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PerceptionNode>());
    rclcpp::shutdown();
    return 0;
}