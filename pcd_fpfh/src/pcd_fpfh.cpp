#include "pcd_fpfh/pcd_fpfh.hpp"

PcdFpfhNode::PcdFpfhNode() : Node("pcd_fpfh_node")
{
    subscription_ = this->create_subscription<pcc::msg::PointCloud2Array>(
        "/objects/cluster_array", 10,
        std::bind(&PcdFpfhNode::clusterCallback, this, std::placeholders::_1)
    );

    publisher_ = this->create_publisher<custom_interfaces::msg::PcDetectionArray>(
        "/pcd_fpfh/detections", 10
    );

    RCLCPP_INFO(this->get_logger(), "PcdFpfhNode started");
}

void PcdFpfhNode::clusterCallback(const pcc::msg::PointCloud2Array::SharedPtr msg)
{
    // Dummy implementation: just create empty PcDetectionArray and publish
    custom_interfaces::msg::PcDetectionArray detection_array;
    publisher_->publish(detection_array);

    RCLCPP_INFO(this->get_logger(), "Received %zu clusters, published dummy detection", msg->clouds.size());
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PcdFpfhNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
