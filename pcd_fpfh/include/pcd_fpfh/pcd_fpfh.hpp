#ifndef PCD_FPFH_HPP
#define PCD_FPFH_HPP

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcc/msg/point_cloud2_array.hpp"
#include "custom_interfaces/msg/pc_detection_array.hpp"

class PcdFpfhNode : public rclcpp::Node
{
public:
    PcdFpfhNode();

private:
    void clusterCallback(const pcc::msg::PointCloud2Array::SharedPtr msg);

    rclcpp::Subscription<pcc::msg::PointCloud2Array>::SharedPtr subscription_;
    rclcpp::Publisher<custom_interfaces::msg::PcDetectionArray>::SharedPtr publisher_;
};

#endif  // PCD_FPFH_HPP
