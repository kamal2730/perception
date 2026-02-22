#ifndef PERCEPTION_HPP_
#define PERCEPTION_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>

class PerceptionNode : public rclcpp::Node
{
public:
    PerceptionNode();

private:
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_sub_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_pointcloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;

    // Timer for continuous publishing
    rclcpp::TimerBase::SharedPtr publish_timer_;

    // Stored snapshot
    sensor_msgs::msg::PointCloud2 stored_pointcloud_;
    sensor_msgs::msg::Image stored_image_;

    bool snapshot_available_;

    // Callbacks
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void triggerCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void publishStoredData();
};

#endif