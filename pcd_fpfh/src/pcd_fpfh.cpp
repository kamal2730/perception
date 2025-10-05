#include "pcd_fpfh/pcd_fpfh.hpp"

PcdFpfhNode::PcdFpfhNode(const std::string &name) : Node(name)
{
    // Load STL model
    std::string stl_path = "/home/rm/ws/src/perception/pcd_fpfh/models/F20_20.stl";
    pcl::PolygonMesh mesh;
    if (pcl::io::loadPolygonFileSTL(stl_path, mesh) == 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to load STL model: %s", stl_path.c_str());
        throw std::runtime_error("STL model load failed");
    }

    model_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromPCLPointCloud2(mesh.cloud, *model_cloud_);

    model_fpfh_ = pcl::PointCloud<pcl::FPFHSignature33>::Ptr(new pcl::PointCloud<pcl::FPFHSignature33>());
    computeFPFH(model_cloud_, model_fpfh_);

    // Subscribers
    sub_ = this->create_subscription<pcc::msg::PointCloud2Array>(
        "/objects/cluster_array", 10,
        std::bind(&PcdFpfhNode::clusterCallback, this, std::placeholders::_1));

    // Publishers
    pub_ = this->create_publisher<custom_interfaces::msg::PcDetectionArray>(
        "/objects/fpfh_detections", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/objects/fpfh_markers", 10);

    RCLCPP_INFO(this->get_logger(), "FPFH Node initialized");
}

void PcdFpfhNode::computeFPFH(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                              pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh)
{
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.02);
    ne.compute(*normals);

    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
    fpfh_est.setInputCloud(cloud);
    fpfh_est.setInputNormals(normals);
    fpfh_est.setSearchMethod(tree);
    fpfh_est.setRadiusSearch(0.05);
    fpfh_est.compute(*fpfh);
}

float PcdFpfhNode::compareFPFH(pcl::PointCloud<pcl::FPFHSignature33>::Ptr f1,
                               pcl::PointCloud<pcl::FPFHSignature33>::Ptr f2)
{
    Eigen::VectorXf hist1(33), hist2(33);
    hist1.setZero();
    hist2.setZero();

    for (auto &pt : f1->points)
        for (int i = 0; i < 33; ++i)
            hist1[i] += pt.histogram[i];
    if (!f1->points.empty()) hist1 /= f1->points.size();

    for (auto &pt : f2->points)
        for (int i = 0; i < 33; ++i)
            hist2[i] += pt.histogram[i];
    if (!f2->points.empty()) hist2 /= f2->points.size();

    float denom = hist1.norm() * hist2.norm();
    if (denom == 0.0f) return 0.0f;

    return hist1.dot(hist2) / denom; // cosine similarity
}

visualization_msgs::msg::Marker PcdFpfhNode::createBoundingBoxMarker(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int id)
{
    Eigen::Vector4f min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);

    float padding = 0.02f;
    min_pt.head<3>() -= Eigen::Vector3f(padding, padding, padding);
    max_pt.head<3>() += Eigen::Vector3f(padding, padding, padding);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "fpfh_bbox";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = (min_pt.x() + max_pt.x()) / 2.0;
    marker.pose.position.y = (min_pt.y() + max_pt.y()) / 2.0;
    marker.pose.position.z = (min_pt.z() + max_pt.z()) / 2.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = max_pt.x() - min_pt.x();
    marker.scale.y = max_pt.y() - min_pt.y();
    marker.scale.z = max_pt.z() - min_pt.z();
    marker.color.a = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.lifetime = rclcpp::Duration::from_seconds(1.0);

    return marker;
}

void PcdFpfhNode::clusterCallback(const pcc::msg::PointCloud2Array::SharedPtr msg)
{
    custom_interfaces::msg::PcDetectionArray detections;
    visualization_msgs::msg::MarkerArray marker_array;

    int similar_count = 0;
    const float similarity_threshold = 0.001f;

    int cluster_idx = 0;
    for (auto &cloud_msg : msg->clouds)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(cloud_msg, *cluster);

        pcl::PointCloud<pcl::FPFHSignature33>::Ptr cluster_fpfh(new pcl::PointCloud<pcl::FPFHSignature33>());
        computeFPFH(cluster, cluster_fpfh);

        float similarity = compareFPFH(model_fpfh_, cluster_fpfh);

        // Only count/publish clusters that pass threshold
        if (similarity >= similarity_threshold)
        {
            similar_count++;

            // Create marker for matched cluster
            marker_array.markers.push_back(createBoundingBoxMarker(cluster, cluster_idx));
        }

        // Always push detection (optional: could skip detections below threshold if desired)
        custom_interfaces::msg::PcDetection det;
        det.name = "model_object";
        det.probability = similarity;
        det.cloud = cloud_msg;
        detections.pc_detections.push_back(det);

        RCLCPP_INFO(this->get_logger(), "Cluster %d → similarity: %.6f", cluster_idx + 1, similarity);
        cluster_idx++;
    }

    pub_->publish(detections);

    // Only publish markers if any matched
    if (!marker_array.markers.empty())
        marker_pub_->publish(marker_array);

    RCLCPP_INFO(this->get_logger(),
                "Published %zu detections, %d clusters similar to model",
                detections.pc_detections.size(),
                similar_count);
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PcdFpfhNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
