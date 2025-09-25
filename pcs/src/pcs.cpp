#include "pcs/pcs_node.hpp"

PcsNode::PcsNode() : Node("pcs_node")
{
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/zed/zed_node/point_cloud/cloud_registered", 10,
        std::bind(&PcsNode::pointcloudCallback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/filtered/points", 10);

    RCLCPP_INFO(this->get_logger(), "PCS Node with VoxelGrid Filter (RGB) started");
}

void PcsNode::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // Convert ROS2 PointCloud2 -> PCL PointCloud with RGB
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*msg, *cloud);

    RCLCPP_INFO(this->get_logger(), "Received cloud with %lu points", cloud->size());

    // Apply VoxelGrid filter
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(0.01f, 0.01f, 0.01f);  // adjust resolution
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
    voxel.filter(*filtered);

    RCLCPP_INFO(this->get_logger(), "Filtered cloud has %lu points", filtered->size());

    // Convert PCL -> ROS2 PointCloud2
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*filtered, output_msg);
    output_msg.header = msg->header;

    publisher_->publish(output_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PcsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
