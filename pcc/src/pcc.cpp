#include "pcc/pcc.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>

ObjectClusterNode::ObjectClusterNode() : Node("object_cluster_node")
{
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/objects/points", 10,
        std::bind(&ObjectClusterNode::pointCloudCallback, this, std::placeholders::_1));

    cluster_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/objects/clusters_colored", 10);

    marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/objects/bounding_boxes", 10);

    RCLCPP_INFO(this->get_logger(), "Object Clustering Node with Bounding Boxes Started");
}

void ObjectClusterNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // Convert ROS2 -> PCL
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*msg, *cloud);
    RCLCPP_INFO(this->get_logger(), "Received cloud with %lu points", cloud->size());

    // --- Remove NaN / Inf points ---
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (auto &pt : cloud->points)
        if (pcl::isFinite(pt))
            cloud_filtered->points.push_back(pt);
    cloud_filtered->width = cloud_filtered->points.size();
    cloud_filtered->height = 1;
    cloud_filtered->is_dense = true;

    // --- Downsample ---
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud(cloud_filtered);
    vg.setLeafSize(0.005f, 0.005f, 0.005f);
    vg.filter(*cloud_downsampled);

    // --- Euclidean Clustering ---
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    tree->setInputCloud(cloud_downsampled);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.02);
    ec.setMinClusterSize(50);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_downsampled);
    ec.extract(cluster_indices);

    // --- Assign colors & bounding boxes ---
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored(new pcl::PointCloud<pcl::PointXYZRGB>());
    visualization_msgs::msg::MarkerArray marker_array;

    int cluster_id = 0;
    std::vector<uint8_t> colors = {255, 0, 0,   // red
                                   0, 255, 0,   // green
                                   0, 0, 255,   // blue
                                   255, 255, 0, // yellow
                                   255, 0, 255, // magenta
                                   0, 255, 255};// cyan

    for (auto &indices : cluster_indices)
    {
        // Color points
        uint8_t r = colors[(cluster_id*3)%colors.size()];
        uint8_t g = colors[(cluster_id*3+1)%colors.size()];
        uint8_t b = colors[(cluster_id*3+2)%colors.size()];

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>());
        for (auto &idx : indices.indices)
        {
            pcl::PointXYZRGB pt = cloud_downsampled->points[idx];
            pt.r = r;
            pt.g = g;
            pt.b = b;
            cloud_colored->points.push_back(pt);
            cluster->points.push_back(pt);
        }

        cluster->width = cluster->points.size();
        cluster->height = 1;

        // --- Compute bounding box ---
        pcl::PointXYZRGB min_pt, max_pt;
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

        marker.scale.x = (max_pt.x - min_pt.x);
        marker.scale.y = (max_pt.y - min_pt.y);
        marker.scale.z = (max_pt.z - min_pt.z);

        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.5f;

        marker_array.markers.push_back(marker);

        cluster_id++;
    }

    // Prepare cloud
    cloud_colored->width = cloud_colored->points.size();
    cloud_colored->height = 1;
    cloud_colored->is_dense = true;

    // Publish colored clusters
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud_colored, output_msg);
    output_msg.header = msg->header;
    cluster_publisher_->publish(output_msg);

    // Publish bounding boxes
    marker_publisher_->publish(marker_array);

    RCLCPP_INFO(this->get_logger(), "Published %lu clusters with bounding boxes", cluster_indices.size());
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObjectClusterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
