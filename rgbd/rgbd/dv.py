import rclpy
from rclpy.node import Node
from custom_interfaces.msg import RgbDetection
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge
import numpy as np


class DetectionVisualizer(Node):
    def __init__(self):
        super().__init__('detection_visualizer')

        self.bridge = CvBridge()
        self.latest_image = None

        # Subscriber to image topic
        self.image_subscription = self.create_subscription(
            Image,
            '/zed/zed_node/rgb/image_rect_color',
            self.image_callback,
            10
        )

        # Subscriber to detection topic
        self.detection_subscription = self.create_subscription(
            RgbDetection,
            '/test_detections',
            self.detection_callback,
            10
        )

        # Publisher for annotated image
        self.annotated_image_publisher = self.create_publisher(
            Image,
            '/annotated_image',
            10
        )

        self.get_logger().info("Detection Visualizer Node started.")

    def image_callback(self, msg: Image):
        """Store the latest image."""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")

    def detection_callback(self, msg: RgbDetection):
        """Draw detections on the latest image, display and publish it."""
        if self.latest_image is None:
            self.get_logger().warn("No image received yet.")
            return

        # Make a copy to draw on
        cv_image = self.latest_image.copy()

        # Draw bounding boxes
        for obj in msg.objects:
            x1 = int(obj.x)
            y1 = int(obj.y)
            x2 = int(obj.x + obj.width)
            y2 = int(obj.y + obj.height)

            cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"{obj.name} ({obj.probability:.2f})"
            cv2.putText(cv_image, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Publish the annotated image
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            self.annotated_image_publisher.publish(annotated_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to convert and publish image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = DetectionVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
