#include "perception/merge_clusters.hpp"

ClusterMergerNode::ClusterMergerNode() : Node("cluster_merger")
{
    cluster_sub_ = this->create_subscription<perception::msg::PointCloud2Array>(
        "/objects/cluster_array", 10,
        std::bind(&ClusterMergerNode::clusterArrayCallback, this, std::placeholders::_1));

    merged_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/objects/merged_cloud", 10);

    RCLCPP_INFO(this->get_logger(), "ClusterMerger Node Started");
}

void ClusterMergerNode::clusterArrayCallback(const perception::msg::PointCloud2Array::SharedPtr msg)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    for (const auto &cloud_msg : msg->clouds)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::fromROSMsg(cloud_msg, *cluster);
        *merged_cloud += *cluster;
    }

    if (merged_cloud->empty())
    {
        // RCLCPP_WARN(this->get_logger(), "Merged cloud is empty, skipping publish");
        return;
    }

    // Publish without downsampling
    sensor_msgs::msg::PointCloud2 merged_msg;
    pcl::toROSMsg(*merged_cloud, merged_msg);
    merged_msg.header = msg->clouds.front().header;

    merged_pub_->publish(merged_msg);

    // RCLCPP_INFO(this->get_logger(), "Published merged cloud with %lu points", merged_cloud->points.size());
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ClusterMergerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
