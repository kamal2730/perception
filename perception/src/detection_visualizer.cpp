#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "custom_interfaces/msg/rgb_detection.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

using std::placeholders::_1;

class DetectionVisualizer : public rclcpp::Node
{
public:
    DetectionVisualizer()
    : Node("detection_visualizer")
    {
        // Subscribe to image topic
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/zed/zed_node/rgb/image_rect_color", 10,
            std::bind(&DetectionVisualizer::image_callback, this, _1)
        );

        // Subscribe to detection topic
        detection_sub_ = this->create_subscription<custom_interfaces::msg::RgbDetection>(
            "/test_detections", 10,
            std::bind(&DetectionVisualizer::detection_callback, this, _1)
        );

        // Publisher for annotated images
        annotated_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/annotated_image", 10
        );

        RCLCPP_INFO(this->get_logger(), "Detection Visualizer Node started.");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<custom_interfaces::msg::RgbDetection>::SharedPtr detection_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_image_pub_;

    cv::Mat latest_image_;
    std::mutex image_mutex_; // for thread-safe access to latest_image_

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            std::lock_guard<std::mutex> lock(image_mutex_);
            latest_image_ = cv_ptr->image.clone();  // store a copy of the latest image
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void detection_callback(const custom_interfaces::msg::RgbDetection::SharedPtr msg)
    {
        cv::Mat image_copy;

        {
            std::lock_guard<std::mutex> lock(image_mutex_);
            if (latest_image_.empty()) {
                RCLCPP_WARN(this->get_logger(), "No image received yet.");
                return;
            }
            image_copy = latest_image_.clone();
        }

        // Draw bounding boxes
        for (const auto& obj : msg->objects) {
            int x1 = static_cast<int>(obj.x);
            int y1 = static_cast<int>(obj.y);
            int x2 = static_cast<int>(obj.x + obj.width);
            int y2 = static_cast<int>(obj.y + obj.height);

            cv::rectangle(image_copy, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);
            std::ostringstream label_stream;
            label_stream << obj.name << " (" << std::fixed << std::setprecision(2) << obj.probability << ")";
            cv::putText(image_copy, label_stream.str(), cv::Point(x1, y1 - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
        }

        // Publish the annotated image
        auto annotated_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_copy).toImageMsg();
        annotated_image_pub_->publish(*annotated_msg);
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DetectionVisualizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
