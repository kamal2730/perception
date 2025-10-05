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

    // Subscription
    sub_ = this->create_subscription<pcc::msg::PointCloud2Array>(
        "/objects/cluster_array", 10,
        std::bind(&PcdFpfhNode::clusterCallback, this, std::placeholders::_1));

    // Publisher
    pub_ = this->create_publisher<custom_interfaces::msg::PcDetectionArray>(
        "/objects/fpfh_detections", 10);

    RCLCPP_INFO(this->get_logger(), "FPFH Node initialized");
}

void PcdFpfhNode::computeFPFH(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                              pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh)
{
    // Estimate normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.02);
    ne.compute(*normals);

    // Compute FPFH
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
    hist1 /= f1->points.size();

    for (auto &pt : f2->points)
        for (int i = 0; i < 33; ++i)
            hist2[i] += pt.histogram[i];
    hist2 /= f2->points.size();

    return (hist1 - hist2).norm();
}

void PcdFpfhNode::clusterCallback(const pcc::msg::PointCloud2Array::SharedPtr msg)
{
    custom_interfaces::msg::PcDetectionArray detections;
    int similar_count = 0;
    const float similarity_threshold = 0.5f; // tweak this

    for (auto &cloud_msg : msg->clouds)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(cloud_msg, *cluster);

        pcl::PointCloud<pcl::FPFHSignature33>::Ptr cluster_fpfh(new pcl::PointCloud<pcl::FPFHSignature33>());
        computeFPFH(cluster, cluster_fpfh);

        float similarity = compareFPFH(model_fpfh_, cluster_fpfh);

        if (similarity < similarity_threshold)
            similar_count++;

        custom_interfaces::msg::PcDetection det;
        det.name = "model_object";
        det.probability = 1.0f / (1.0f + similarity);
        det.cloud = cloud_msg;

        detections.pc_detections.push_back(det);
    }

    pub_->publish(detections);

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
