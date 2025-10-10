#ifndef DEPTH2POSE_NODE_HPP_
#define DEPTH2POSE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/common.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include "custom_interfaces/msg/rgb_detection.hpp"
#include "perception/msg/point_cloud2_array.hpp"

struct ClusterInfo
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    pcl::ModelCoefficients::Ptr plane_coeff;  // plane coefficients for 6DoF later
    Eigen::Vector3f plane_normal;             // plane normal vector
};

class Depth2PoseNode : public rclcpp::Node
{
public:
    Depth2PoseNode();

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void detectionCallback(const custom_interfaces::msg::RgbDetection::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
    rclcpp::Subscription<custom_interfaces::msg::RgbDetection>::SharedPtr det_sub_;

    rclcpp::Publisher<perception::msg::PointCloud2Array>::SharedPtr cluster_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    custom_interfaces::msg::RgbDetection::SharedPtr latest_detection_;
};

#endif  // DEPTH2POSE_NODE_HPP_
