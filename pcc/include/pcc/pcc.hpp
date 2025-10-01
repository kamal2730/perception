#ifndef OBJECT_CLUSTER_NODE_HPP_
#define OBJECT_CLUSTER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>

// Custom message for array of clouds
#include "pcc/msg/point_cloud2_array.hpp"

class ObjectClusterNode : public rclcpp::Node
{
public:
    ObjectClusterNode();

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_publisher_;   // old topic
    rclcpp::Publisher<pcc::msg::PointCloud2Array>::SharedPtr cluster_array_publisher_; // new topic
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
};

#endif  // OBJECT_CLUSTER_NODE_HPP_
