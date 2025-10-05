#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcc/msg/point_cloud2_array.hpp>
#include <custom_interfaces/msg/pc_detection_array.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_lib_io.h>        // For loadPolygonFileSTL
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>

#include <Eigen/Dense>
#include <string>

class PcdFpfhNode : public rclcpp::Node
{
public:
    PcdFpfhNode(const std::string &name = "pcd_fpfh_node");

private:
    void clusterCallback(const pcc::msg::PointCloud2Array::SharedPtr msg);
    void computeFPFH(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                     pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh);
    float compareFPFH(pcl::PointCloud<pcl::FPFHSignature33>::Ptr f1,
                      pcl::PointCloud<pcl::FPFHSignature33>::Ptr f2);

    rclcpp::Subscription<pcc::msg::PointCloud2Array>::SharedPtr sub_;
    rclcpp::Publisher<custom_interfaces::msg::PcDetectionArray>::SharedPtr pub_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud_;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr model_fpfh_;
};
