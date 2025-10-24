#include "perception/depth2pose.hpp"
#include <Eigen/Dense>

Depth2PoseNode::Depth2PoseNode() : Node("depth2pose")
{
    pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/zed/zed_node/point_cloud/cloud_registered", 10,
        std::bind(&Depth2PoseNode::pointCloudCallback, this, std::placeholders::_1));

    det_sub_ = this->create_subscription<custom_interfaces::msg::RgbDetection>(
        "/rgb_detections", 10,
        std::bind(&Depth2PoseNode::detectionCallback, this, std::placeholders::_1));

    cluster_pub_ = this->create_publisher<perception::msg::PointCloud2Array>("/objects/cluster_array", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/objects/bounding_boxes", 10);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/objects/poses", 10);
    trigger_client_ = this->create_client<custom_interfaces::srv::Trigger>("/rgbd_trigger");


    RCLCPP_INFO(this->get_logger(), "Depth2Pose Node Started (6DoF enabled)");
}

void Depth2PoseNode::detectionCallback(custom_interfaces::msg::RgbDetection::SharedPtr msg)
{
    latest_detection_ = msg;
}

void Depth2PoseNode::callResetTrigger()
{
    if (!trigger_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(this->get_logger(), "Trigger service not available");
        return;
    }

    auto request = std::make_shared<custom_interfaces::srv::Trigger::Request>();
    request->reset = false;

    auto future = trigger_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future,
                                           std::chrono::seconds(1))
        == rclcpp::FutureReturnCode::SUCCESS) 
    {
        RCLCPP_INFO(this->get_logger(), "Trigger service responded successfully");
    } 
    else 
    {
        RCLCPP_WARN(this->get_logger(), "Trigger service call failed or timed out");
    }
}


void Depth2PoseNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    try {
        processPointCloud(msg);  // call the renamed function
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Exception in pointCloudCallback: %s", e.what());
        callResetTrigger();
    }
    catch (...) {
        RCLCPP_ERROR(this->get_logger(), "Unknown exception in pointCloudCallback");
        callResetTrigger();
    }
}


void Depth2PoseNode::processPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    if (!latest_detection_) {
        // RCLCPP_WARN(this->get_logger(), "No detections yet, skipping pointcloud");
        return;
    }

    if (msg->width == 0 || msg->height == 0) {
        RCLCPP_WARN(this->get_logger(), "Received empty point cloud, skipping");
        return;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    try {
        pcl::fromROSMsg(*msg, *cloud);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to convert point cloud: %s", e.what());
        callResetTrigger();
        return;
    }

    if (cloud->empty()) {
        RCLCPP_WARN(this->get_logger(), "Converted point cloud is empty, skipping");
        return;
    }

    perception::msg::PointCloud2Array cluster_array_msg;
    visualization_msgs::msg::MarkerArray marker_array;
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header = msg->header;

    int cluster_id = 0;
    bool any_valid_clusters = false;

    for (auto &obj : latest_detection_->objects)
    {
        int x_min = static_cast<int>(obj.x);
        int y_min = static_cast<int>(obj.y);
        int width = static_cast<int>(obj.width);
        int height = static_cast<int>(obj.height);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>());

        // Extract points inside bbox with bounds checking
        for (int v = y_min; v < y_min + height && v < static_cast<int>(msg->height); v++) {
            if (v < 0) continue;
            for (int u = x_min; u < x_min + width && u < static_cast<int>(msg->width); u++) {
                if (u < 0) continue;
                size_t idx = static_cast<size_t>(v) * msg->width + u;
                if (idx >= cloud->points.size()) break;
                const auto &pt = cloud->points[idx];
                if (!pcl::isFinite(pt) || !std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
                cluster->points.push_back(pt);
            }
        }

        if (cluster->empty()) continue;

        // --- Plane removal per cluster ---
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_only(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients());
        bool has_plane_coefficients = false;

        try {
            pcl::SACSegmentation<pcl::PointXYZRGB> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(0.005);
            seg.setInputCloud(cluster);

            pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
            seg.segment(*inliers, *plane_coefficients);

            if (!inliers->indices.empty()) {
                pcl::ExtractIndices<pcl::PointXYZRGB> extract;
                extract.setInputCloud(cluster);
                extract.setIndices(inliers);
                extract.setNegative(true); // remove plane
                extract.filter(*object_only);
                has_plane_coefficients = (plane_coefficients->values.size() >= 3);
            } else {
                object_only = cluster;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in plane segmentation: %s", e.what());
            callResetTrigger();
            continue;
        }

        if (object_only->empty()) continue;

        // --- Noise removal ---
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_clean(new pcl::PointCloud<pcl::PointXYZRGB>());
        try {
            pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
            sor.setInputCloud(object_only);
            sor.setMeanK(20);
            sor.setStddevMulThresh(1.0);
            sor.filter(*cluster_clean);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in statistical outlier removal: %s", e.what());
            callResetTrigger();
            continue;
        }

        if (cluster_clean->empty()) continue;

        // --- Compute centroid and orientation ---
        Eigen::Vector4f centroid;
        Eigen::Quaternionf q;
        
        try {
            // Compute centroid
            if (!pcl::compute3DCentroid(*cluster_clean, centroid)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to compute centroid for object %d", cluster_id);
                callResetTrigger();
                continue;
            }

            // Compute PCA for orientation
            pcl::PCA<pcl::PointXYZRGB> pca;
            pca.setInputCloud(cluster_clean);
            
            if (pca.getEigenVectors().rows() < 3 || pca.getEigenVectors().cols() < 1) {
                RCLCPP_ERROR(this->get_logger(), "Invalid PCA results for object %d", cluster_id);
                callResetTrigger();
                continue;
            }
            
            Eigen::Vector3f x_axis = pca.getEigenVectors().col(0); // principal direction in plane

            // Plane normal
            Eigen::Vector3f z_axis(0,0,1);
            if (has_plane_coefficients) {
                z_axis = Eigen::Vector3f(plane_coefficients->values[0], 
                                       plane_coefficients->values[1], 
                                       plane_coefficients->values[2]).normalized();
            }

            // y-axis perpendicular to plane normal & x-axis
            Eigen::Vector3f y_axis = z_axis.cross(x_axis).normalized();
            x_axis = y_axis.cross(z_axis).normalized(); // re-orthogonalize

            // Build rotation matrix
            Eigen::Matrix3f R;
            R.col(0) = x_axis;
            R.col(1) = y_axis;
            R.col(2) = z_axis;

            q = Eigen::Quaternionf(R);
            
            if (!q.coeffs().allFinite()) {
                RCLCPP_ERROR(this->get_logger(), "Invalid quaternion computed for object %d", cluster_id);
                continue;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in pose computation: %s", e.what());
            callResetTrigger();
            continue;
        }

        // --- Publish Pose ---
        geometry_msgs::msg::Pose pose;
        pose.position.x = centroid[0];
        pose.position.y = centroid[1];
        pose.position.z = centroid[2];
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();

        if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) || !std::isfinite(pose.position.z) ||
            !std::isfinite(pose.orientation.x) || !std::isfinite(pose.orientation.y) ||
            !std::isfinite(pose.orientation.z) || !std::isfinite(pose.orientation.w)) {
            RCLCPP_WARN(this->get_logger(), "Invalid pose values for object %d, skipping", cluster_id);
            continue;
        }

        // --- Convert cluster to ROS msg ---
        sensor_msgs::msg::PointCloud2 cluster_msg;
        try {
            pcl::toROSMsg(*cluster_clean, cluster_msg);
            cluster_msg.header = msg->header;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert cluster to ROS message: %s", e.what());
            continue;
        }

        // --- Compute bounding box ---
        pcl::PointXYZRGB min_pt, max_pt;
        try {
            pcl::getMinMax3D(*cluster_clean, min_pt, max_pt);
            
            // Verify the computed bounds are valid
            if (!std::isfinite(min_pt.x) || !std::isfinite(min_pt.y) || !std::isfinite(min_pt.z) ||
                !std::isfinite(max_pt.x) || !std::isfinite(max_pt.y) || !std::isfinite(max_pt.z)) {
                RCLCPP_ERROR(this->get_logger(), "Invalid bounding box values for cluster %d", cluster_id);
                continue;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error computing bounding box: %s", e.what());
            continue;
        }

        float center_x = (min_pt.x + max_pt.x)/2.0f;
        float center_y = (min_pt.y + max_pt.y)/2.0f;
        float center_z = (min_pt.z + max_pt.z)/2.0f;
        
        if (!std::isfinite(center_x) || !std::isfinite(center_y) || !std::isfinite(center_z)) {
            RCLCPP_WARN(this->get_logger(), "Invalid marker position for cluster %d, skipping", cluster_id);
            continue;
        }
        
        float scale_x = std::max((max_pt.x - min_pt.x)*1.2f, 0.001f);
        float scale_y = std::max((max_pt.y - min_pt.y)*1.2f, 0.001f);
        float scale_z = std::max((max_pt.z - min_pt.z)*1.2f, 0.001f);

        if (!std::isfinite(scale_x) || !std::isfinite(scale_y) || !std::isfinite(scale_z)) {
            RCLCPP_WARN(this->get_logger(), "Invalid marker scale for cluster %d, skipping", cluster_id);
            continue;
        }

        // Add to message arrays
        pose_array.poses.push_back(pose);
        cluster_array_msg.clouds.push_back(cluster_msg);

        // --- Create visualization marker ---
        visualization_msgs::msg::Marker marker;
        marker.header = msg->header;
        marker.ns = "clusters";
        marker.id = cluster_id++;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = center_x;
        marker.pose.position.y = center_y;
        marker.pose.position.z = center_z;
        marker.scale.x = scale_x;
        marker.scale.y = scale_y;
        marker.scale.z = scale_z;
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.5f;
        marker.lifetime = rclcpp::Duration::from_seconds(0.5);
        marker_array.markers.push_back(marker);
        
        any_valid_clusters = true;
    }

    // Publish results only if we have valid clusters
    if (any_valid_clusters) {
        cluster_pub_->publish(cluster_array_msg);
        marker_pub_->publish(marker_array);
        pose_pub_->publish(pose_array);
        RCLCPP_INFO(this->get_logger(), "Published %lu clusters and poses", cluster_array_msg.clouds.size());
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Depth2PoseNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
