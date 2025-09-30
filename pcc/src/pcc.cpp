#include "pcc/pcc.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>

ObjectClusterNode::ObjectClusterNode() : Node("object_cluster_node")
{
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/objects/points", 10,
        std::bind(&ObjectClusterNode::pointCloudCallback, this, std::placeholders::_1));

    cluster_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/objects/clusters", 10);

    marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/objects/bounding_boxes", 10);

    RCLCPP_INFO(this->get_logger(), "Object Clustering Node Started");
}

void ObjectClusterNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // Convert ROS2 -> PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud);
    RCLCPP_INFO(this->get_logger(), "Received cloud with %lu points", cloud->size());

    // Remove NaN / Inf points
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    for (auto &pt : cloud->points)
        if (pcl::isFinite(pt))
            cloud_filtered->points.push_back(pt);
    cloud_filtered->width = cloud_filtered->points.size();
    cloud_filtered->height = 1;
    cloud_filtered->is_dense = true;

    // Downsample
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud_filtered);
    vg.setLeafSize(0.004f, 0.004f, 0.004f); // smaller leaf for small objects
    vg.filter(*cloud_downsampled);

    // Euclidean Clustering
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(cloud_downsampled);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.015); // tighter tolerance for small objects
    ec.setMinClusterSize(20);      // catch small objects
    ec.setMaxClusterSize(1000);   // ignore very big clusters
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_downsampled);
    ec.extract(cluster_indices);

    visualization_msgs::msg::MarkerArray marker_array;

    int cluster_id = 0;
    for (auto &indices : cluster_indices)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>());
        for (auto &idx : indices.indices)
            cluster->points.push_back(cloud_downsampled->points[idx]);

        cluster->width = cluster->points.size();
        cluster->height = 1;

        // Bounding box
        pcl::PointXYZ min_pt, max_pt;
        pcl::getMinMax3D(*cluster, min_pt, max_pt);

        visualization_msgs::msg::Marker marker;
        marker.header = msg->header;
        marker.ns = "clusters";
        marker.id = cluster_id;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = (min_pt.x + max_pt.x) / 2.0;
        marker.pose.position.y = (min_pt.y + max_pt.y) / 2.0;
        marker.pose.position.z = (min_pt.z + max_pt.z) / 2.0;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Increase bounding box size slightly
        marker.scale.x = (max_pt.x - min_pt.x) * 1.2f;
        marker.scale.y = (max_pt.y - min_pt.y) * 1.2f;
        marker.scale.z = (max_pt.z - min_pt.z) * 1.2f;

        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.5f;

        marker_array.markers.push_back(marker);
        cluster_id++;
    }

    // Publish bounding boxes
    marker_publisher_->publish(marker_array);

    RCLCPP_INFO(this->get_logger(), "Published %lu clusters", cluster_indices.size());
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObjectClusterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
