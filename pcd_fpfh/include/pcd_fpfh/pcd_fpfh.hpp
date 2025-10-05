#pragma once

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcc/msg/PointCloud2Array.hpp"
#include "custom_interfaces/msg/PcDetectionArray.hpp"


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/stl_io.h>
#include <pcl/features/fpfh.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

class PcdFpfhNode : public rclcpp::Node
{
public:
    PcdFpfhNode();

private:
    void clusterCallback(const pcc::msg::PointCloud2Array::SharedPtr msg);
    void computeFPFH(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                     pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfh);
    float compareFPFH(const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &model,
                      const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &cluster);

    rclcpp::Publisher<custom_interfaces::msg::PcDetectionArray>::SharedPtr publisher_;
    rclcpp::Subscription<pcc::msg::PointCloud2Array>::SharedPtr subscription_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud_;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr model_fpfh_;
};
