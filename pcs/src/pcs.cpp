#include "pcs/pcs_node.hpp"
#include <pcl/filters/statistical_outlier_removal.h>

PcsNode::PcsNode() : Node("pcs_node")
{
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/zed/zed_node/point_cloud/cloud_registered", 10,
        std::bind(&PcsNode::pointcloudCallback, this, std::placeholders::_1));

    plane_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/plane/points", 10);

    object_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/objects/points", 10);

    RCLCPP_INFO(this->get_logger(), "PCS Node with Full-Res Objects + SOR started");
}

void PcsNode::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // Convert ROS2 -> PCL PointCloud with RGB
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*msg, *cloud);
    RCLCPP_INFO(this->get_logger(), "Received cloud with %lu points", cloud->size());

    // Downsample cloud for plane segmentation only
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZRGB>());
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(0.01f, 0.01f, 0.01f);  // adjust resolution
    voxel.filter(*cloud_downsampled);
    RCLCPP_INFO(this->get_logger(), "Downsampled cloud has %lu points", cloud_downsampled->size());

    // --- RANSAC Plane Segmentation on downsampled cloud ---
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.005);  // adjust tolerance
    seg.setInputCloud(cloud_downsampled);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        RCLCPP_WARN(this->get_logger(), "No plane found in the cloud.");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Plane detected with %lu inliers", inliers->indices.size());

    // --- Extract plane and object from ORIGINAL cloud using plane equation ---
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr objectCloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    float a = coefficients->values[0];
    float b = coefficients->values[1];
    float c = coefficients->values[2];
    float d = coefficients->values[3];

    for (auto &p : cloud->points)
    {
        float dist = std::abs(a*p.x + b*p.y + c*p.z + d) / std::sqrt(a*a + b*b + c*c);
        if (dist <= 0.005) // plane threshold same as RANSAC
        {
            pcl::PointXYZRGB pt = p;
            pt.r = 255; pt.g = 255; pt.b = 255; // color plane white
            planeCloud->points.push_back(pt);
        }
        else
        {
            objectCloud->points.push_back(p);
        }
    }

    // --- Apply Statistical Outlier Removal to object points ---
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr objectCloudClean(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(objectCloud);
    sor.setMeanK(50);           // number of neighbors to analyze
    sor.setStddevMulThresh(1.0); // threshold multiplier
    sor.filter(*objectCloudClean);

    RCLCPP_INFO(this->get_logger(), "Cleaned object cloud has %lu points", objectCloudClean->size());

    // Convert to ROS2 messages and publish
    sensor_msgs::msg::PointCloud2 plane_msg;
    pcl::toROSMsg(*planeCloud, plane_msg);
    plane_msg.header = msg->header;
    plane_publisher_->publish(plane_msg);

    sensor_msgs::msg::PointCloud2 object_msg;
    pcl::toROSMsg(*objectCloudClean, object_msg);
    object_msg.header = msg->header;
    object_publisher_->publish(object_msg);

    RCLCPP_INFO(this->get_logger(), "Published plane (%lu pts) and objects (%lu pts)",
                planeCloud->size(), objectCloudClean->size());
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PcsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
