#define PERCEPTION_DEBUG   // Comment this line to disable all debug topics

#include "perception/perception.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/common.h>
#include <cmath>

using std::placeholders::_1;

PerceptionNode::PerceptionNode()
    : Node("perception_node"),
      rgb_available_(false),
      snapshot_available_(false)
{
    // Subscribers
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/zed/zed_node/point_cloud/cloud_registered",
        10,
        std::bind(&PerceptionNode::pointCloudCallback, this, _1));

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/zed/zed_node/rgb/color/rect/image",
        10,
        std::bind(&PerceptionNode::imageCallback, this, _1));

    rgb_detection_sub_ =this->create_subscription<custom_interfaces::msg::RgbDetection>(
        "/rgb_detections",
        10,
        std::bind(&PerceptionNode::rgbDetectionCallback, this, _1));

    trigger_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/perception/trigger",
        10,
        std::bind(&PerceptionNode::triggerCallback, this, _1));

    // Existing debug publishers
    debug_pointcloud_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/perception/debug/pointcloud", 10);

    debug_image_pub_ =
        this->create_publisher<sensor_msgs::msg::Image>(
            "/perception/debug/rgb_image", 10);

#ifdef PERCEPTION_DEBUG
    voxel_debug_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/perception/debug/voxel_cloud", 10);

    plane_debug_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/perception/debug/plane_cloud", 10);
#endif

    // Timer 10Hz
    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&PerceptionNode::publishStoredData, this));

    plane_coefficients_.reset(new pcl::ModelCoefficients);

    RCLCPP_INFO(this->get_logger(), "Perception node initialized");
}

void PerceptionNode::pointCloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    latest_pointcloud_ = *msg;
}

void PerceptionNode::imageCallback(
    const sensor_msgs::msg::Image::SharedPtr msg)
{
    latest_image_ = *msg;
}

void PerceptionNode::rgbDetectionCallback(
    const custom_interfaces::msg::RgbDetection::SharedPtr msg)
{
    latest_rgb_detection_ = *msg;
    rgb_available_ = true;

    //RCLCPP_INFO(this->get_logger(),"Received %ld RGB objects",msg->objects.size());
}

void PerceptionNode::triggerCallback(
    const std_msgs::msg::Bool::SharedPtr msg)
{
    if (!msg->data)
        return;

    if (latest_pointcloud_.data.empty() || latest_image_.data.empty())
    {
        RCLCPP_WARN(this->get_logger(), "No data received yet.");
        return;
    }

    stored_pointcloud_ = latest_pointcloud_;
    stored_image_ = latest_image_;

    snapshot_available_ = true;

    processPointCloud();

    RCLCPP_INFO(this->get_logger(),
                "Snapshot captured and processed.");
}

void PerceptionNode::publishStoredData()
{
    if (!snapshot_available_)
        return;

    debug_pointcloud_pub_->publish(stored_pointcloud_);
    debug_image_pub_->publish(stored_image_);
}

void PerceptionNode::processPointCloud()
{
    if (!rgb_available_)
    {
        RCLCPP_WARN(this->get_logger(), "No RGB detections available.");
        return;
    }
    pcl::PointCloud<PointT>::Ptr input(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(stored_pointcloud_, *input);

    // -------- Voxel Downsample --------
    pcl::PointCloud<PointT>::Ptr voxel_cloud(new pcl::PointCloud<PointT>);
    voxelDownsample(input, voxel_cloud);

#ifdef PERCEPTION_DEBUG
    sensor_msgs::msg::PointCloud2 voxel_msg;
    pcl::toROSMsg(*voxel_cloud, voxel_msg);
    voxel_msg.header = stored_pointcloud_.header;
    voxel_debug_pub_->publish(voxel_msg);
#endif

    // -------- Plane Segmentation --------
    pcl::PointCloud<PointT>::Ptr plane_cloud(new pcl::PointCloud<PointT>);
    segmentPlane(voxel_cloud, plane_cloud);

#ifdef PERCEPTION_DEBUG
    sensor_msgs::msg::PointCloud2 plane_msg;
    pcl::toROSMsg(*plane_cloud, plane_msg);
    plane_msg.header = stored_pointcloud_.header;
    plane_debug_pub_->publish(plane_msg);
#endif

    for (const auto & obj : latest_rgb_detection_.objects)
    {
        int x_min = static_cast<int>(obj.x);
        int y_min = static_cast<int>(obj.y);
        int width = static_cast<int>(obj.width);
        int height = static_cast<int>(obj.height);

        pcl::PointCloud<PointT>::Ptr cropped_cloud(new pcl::PointCloud<PointT>);

        // Use organized cloud (IMPORTANT)
        for (int v = y_min; v < y_min + height && v < static_cast<int>(stored_pointcloud_.height); v++)
        {
            if (v < 0) continue;
            for (int u = x_min; u < x_min + width && u < static_cast<int>(stored_pointcloud_.width); u++)
            {
                if (u < 0) continue;
                size_t idx = static_cast<size_t>(v) * stored_pointcloud_.width + u;
                if (idx >= input->points.size()) continue;
                const auto &pt = input->points[idx];
                if (!std::isfinite(pt.x) || 
                    !std::isfinite(pt.y) || 
                    !std::isfinite(pt.z))
                    continue;
                cropped_cloud->points.push_back(pt);
            }
        }

        if (cropped_cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "Cropped cloud empty for object %s", obj.name.c_str());
            continue;
        }

        RCLCPP_INFO(this->get_logger(),
            "Cropped %ld points for object %s",
            cropped_cloud->points.size(),
            obj.name.c_str());
    }
}

void PerceptionNode::voxelDownsample(
    const pcl::PointCloud<PointT>::Ptr& input,
    pcl::PointCloud<PointT>::Ptr& output)
{
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(input);
    vg.setLeafSize(0.005f, 0.005f, 0.005f);   // 5mm voxel
    vg.filter(*output);
}

void PerceptionNode::segmentPlane(
    const pcl::PointCloud<PointT>::Ptr& input,
    pcl::PointCloud<PointT>::Ptr& plane_cloud)
{
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(input);

    seg.segment(*inliers, *plane_coefficients_);

    if (inliers->indices.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Plane segmentation failed.");
        return;
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(input);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*plane_cloud);

    RCLCPP_INFO(this->get_logger(),
        "Plane equation: %.3f x + %.3f y + %.3f z + %.3f = 0",
        plane_coefficients_->values[0],
        plane_coefficients_->values[1],
        plane_coefficients_->values[2],
        plane_coefficients_->values[3]);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PerceptionNode>());
    rclcpp::shutdown();
    return 0;
}