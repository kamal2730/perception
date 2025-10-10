#include "perception/depth2pose.hpp"
#include <Eigen/Dense>

Depth2PoseNode::Depth2PoseNode() : Node("depth2pose")
{
    pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/zed/zed_node/point_cloud/cloud_registered", 10,
        std::bind(&Depth2PoseNode::pointCloudCallback, this, std::placeholders::_1));

    det_sub_ = this->create_subscription<custom_interfaces::msg::RgbDetection>(
        "/test_detections", 10,
        std::bind(&Depth2PoseNode::detectionCallback, this, std::placeholders::_1));

    cluster_pub_ = this->create_publisher<perception::msg::PointCloud2Array>("/objects/cluster_array", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/objects/bounding_boxes", 10);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/objects/poses", 10);

    RCLCPP_INFO(this->get_logger(), "Depth2Pose Node Started (6DoF enabled)");
}

void Depth2PoseNode::detectionCallback(custom_interfaces::msg::RgbDetection::SharedPtr msg)
{
    latest_detection_ = msg;
}

void Depth2PoseNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    if (!latest_detection_) {
        RCLCPP_WARN(this->get_logger(), "No detections yet, skipping pointcloud");
        return;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*msg, *cloud);

    perception::msg::PointCloud2Array cluster_array_msg;
    visualization_msgs::msg::MarkerArray marker_array;
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header = msg->header;

    int cluster_id = 0;

    for (auto &obj : latest_detection_->objects)
    {
        int x_min = static_cast<int>(obj.x);
        int y_min = static_cast<int>(obj.y);
        int width = static_cast<int>(obj.width);
        int height = static_cast<int>(obj.height);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>());

        // Extract points inside bbox
        for (int v = y_min; v < y_min + height; v++) {
            for (int u = x_min; u < x_min + width; u++) {
                int idx = v * msg->width + u;
                if (idx >= cloud->points.size()) continue;
                auto &pt = cloud->points[idx];
                if (!pcl::isFinite(pt)) continue;
                cluster->points.push_back(pt);
            }
        }

        if (cluster->empty()) continue;

        // --- Plane removal per cluster ---
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.005);
        seg.setInputCloud(cluster);

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
        seg.segment(*inliers, *coeff);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_only(new pcl::PointCloud<pcl::PointXYZRGB>());
        if (!inliers->indices.empty()) {
            pcl::ExtractIndices<pcl::PointXYZRGB> extract;
            extract.setInputCloud(cluster);
            extract.setIndices(inliers);
            extract.setNegative(true); // remove plane
            extract.filter(*object_only);
        } else {
            object_only = cluster;
        }

        // --- Noise removal ---
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_clean(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud(object_only);
        sor.setMeanK(20);
        sor.setStddevMulThresh(1.0);
        sor.filter(*cluster_clean);

        if (cluster_clean->empty()) continue;

        // --- Compute centroid ---
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cluster_clean, centroid);

        // --- Compute PCA for orientation ---
        pcl::PCA<pcl::PointXYZRGB> pca;
        pca.setInputCloud(cluster_clean);
        Eigen::Vector3f x_axis = pca.getEigenVectors().col(0); // principal direction in plane

        // --- Plane normal ---
        Eigen::Vector3f z_axis(0,0,1);
        if (coeff->values.size() >= 3)
            z_axis = Eigen::Vector3f(coeff->values[0], coeff->values[1], coeff->values[2]).normalized();

        // --- y-axis perpendicular to plane normal & x-axis ---
        Eigen::Vector3f y_axis = z_axis.cross(x_axis).normalized();
        x_axis = y_axis.cross(z_axis).normalized(); // re-orthogonalize

        // --- Build rotation matrix ---
        Eigen::Matrix3f R;
        R.col(0) = x_axis;
        R.col(1) = y_axis;
        R.col(2) = z_axis;

        Eigen::Quaternionf q(R);

        // --- Publish Pose ---
        geometry_msgs::msg::Pose pose;
        pose.position.x = centroid[0];
        pose.position.y = centroid[1];
        pose.position.z = centroid[2];
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();

        pose_array.poses.push_back(pose);

        // --- Convert cluster to ROS msg ---
        sensor_msgs::msg::PointCloud2 cluster_msg;
        pcl::toROSMsg(*cluster_clean, cluster_msg);
        cluster_msg.header = msg->header;
        cluster_array_msg.clouds.push_back(cluster_msg);

        // --- Visualization marker ---
        pcl::PointXYZRGB min_pt, max_pt;
        pcl::getMinMax3D(*cluster_clean, min_pt, max_pt);
        visualization_msgs::msg::Marker marker;
        marker.header = msg->header;
        marker.ns = "clusters";
        marker.id = cluster_id++;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = (min_pt.x + max_pt.x)/2.0;
        marker.pose.position.y = (min_pt.y + max_pt.y)/2.0;
        marker.pose.position.z = (min_pt.z + max_pt.z)/2.0;
        marker.scale.x = (max_pt.x - min_pt.x)*1.2f;
        marker.scale.y = (max_pt.y - min_pt.y)*1.2f;
        marker.scale.z = (max_pt.z - min_pt.z)*1.2f;
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.5f;
        marker_array.markers.push_back(marker);
    }

    cluster_pub_->publish(cluster_array_msg);
    marker_pub_->publish(marker_array);
    pose_pub_->publish(pose_array);

    RCLCPP_INFO(this->get_logger(), "Published %lu clusters and poses", cluster_array_msg.clouds.size());
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Depth2PoseNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
