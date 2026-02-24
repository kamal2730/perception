#ifndef PERCEPTION_HPP_
#define PERCEPTION_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include "custom_interfaces/msg/rgb_detection.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>

class PerceptionNode : public rclcpp::Node
{
public:
    PerceptionNode();

private:
    using PointT = pcl::PointXYZ;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_sub_;
    rclcpp::Subscription<custom_interfaces::msg::RgbDetection>::SharedPtr rgb_detection_sub_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_pointcloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;

#ifdef PERCEPTION_DEBUG
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_debug_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr plane_debug_pub_;
#endif

    // Timer
    rclcpp::TimerBase::SharedPtr publish_timer_;

    // Stored snapshot
    sensor_msgs::msg::PointCloud2 stored_pointcloud_;
    sensor_msgs::msg::Image stored_image_;

    // Incoming data
    sensor_msgs::msg::PointCloud2 latest_pointcloud_;
    sensor_msgs::msg::Image latest_image_;
    custom_interfaces::msg::RgbDetection latest_rgb_detection_;
    
    bool rgb_available_;
    bool snapshot_available_;

    // Plane coefficients (a,b,c,d)
    pcl::ModelCoefficients::Ptr plane_coefficients_;

    // Processing
    void processPointCloud();
    void voxelDownsample(const pcl::PointCloud<PointT>::Ptr& input,
                         pcl::PointCloud<PointT>::Ptr& output);
    void segmentPlane(const pcl::PointCloud<PointT>::Ptr& input,
                      pcl::PointCloud<PointT>::Ptr& plane_cloud);

    // Callbacks
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void triggerCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void rgbDetectionCallback(const custom_interfaces::msg::RgbDetection::SharedPtr msg);
    void publishStoredData();
};

#endif