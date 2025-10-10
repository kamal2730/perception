#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "perception/msg/point_cloud2_array.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

class ClusterMergerNode : public rclcpp::Node
{
public:
    ClusterMergerNode();

private:
    void clusterArrayCallback(const perception::msg::PointCloud2Array::SharedPtr msg);

    rclcpp::Subscription<perception::msg::PointCloud2Array>::SharedPtr cluster_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_pub_;
};
