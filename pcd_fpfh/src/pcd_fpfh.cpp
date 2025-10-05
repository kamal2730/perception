#include "pcd_fpfh/pcd_fpfh.hpp"

PcdFpfhNode::PcdFpfhNode()
: Node("pcd_fpfh_node")
{
    publisher_ = this->create_publisher<custom_interfaces::msg::PcDetectionArray>(
        "/pcd_fpfh/detections", 10);

    subscription_ = this->create_subscription<pcc::msg::PointCloud2Array>(
        "/objects/cluster_array", 10,
        std::bind(&PcdFpfhNode::clusterCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "PCD FPFH Node Started");

    // Load STL model
    std::string stl_path = "/home/rm/ws/src/perception/pcd_fpfh/models/F20_20.stl";
    pcl::PolygonMesh mesh;
    model_cloud_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    if (pcl::io::loadPolygonFileSTL(stl_path, mesh) == 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load STL model: %s", stl_path.c_str());
        return;
    }

    pcl::fromPCLPointCloud2(mesh.cloud, *model_cloud_);

    // Compute FPFH for the model
    model_fpfh_ = pcl::make_shared<pcl::PointCloud<pcl::FPFHSignature33>>();
    computeFPFH(model_cloud_, model_fpfh_);

    RCLCPP_INFO(this->get_logger(), "Loaded STL model and computed FPFH: %s", stl_path.c_str());
}

void PcdFpfhNode::computeFPFH(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                              pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfh)
{
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.05);
    ne.compute(*normals);

    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
    fpfh_est.setInputCloud(cloud);
    fpfh_est.setInputNormals(normals);
    fpfh_est.setSearchMethod(tree);
    fpfh_est.setRadiusSearch(0.05);
    fpfh_est.compute(*fpfh);
}

float PcdFpfhNode::compareFPFH(const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &model,
                               const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &cluster)
{
    // Simple comparison: mean L2 distance of descriptors
    float dist = 0.0f;
    size_t n = std::min(model->size(), cluster->size());
    for (size_t i = 0; i < n; ++i) {
        for (int j = 0; j < 33; ++j)
            dist += std::pow(model->points[i].histogram[j] - cluster->points[i].histogram[j], 2);
    }
    return dist / n;
}

void PcdFpfhNode::clusterCallback(const pcc::msg::PointCloud2Array::SharedPtr msg)
{
    custom_interfaces::msg::PcDetectionArray detections;
    int similar_count = 0;
    const float similarity_threshold = 0.5f;

    for (auto &cloud_msg : msg->clouds) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(cloud_msg, *cluster);

        pcl::PointCloud<pcl::FPFHSignature33>::Ptr cluster_fpfh(new pcl::PointCloud<pcl::FPFHSignature33>());
        computeFPFH(cluster, cluster_fpfh);

        float similarity = compareFPFH(model_fpfh_, cluster_fpfh);

        if (similarity < similarity_threshold) similar_count++;

        custom_interfaces::msg::PcDetection det;
        det.name = "model_object";
        det.probability = 1.0f / (1.0f + similarity);
        det.cloud = cloud_msg;
        detections.pc_detections.push_back(det);
    }

    publisher_->publish(detections);

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
