#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from custom_interfaces.msg import RgbDetection, RgbObject
import cv2
from cv_bridge import CvBridge
import numpy as np
from ultralytics import YOLO
import argparse


class RgbDetNode(Node):
    def __init__(self, model_path, imgsz=640, visualize=False):
        super().__init__('rgb_det_node')
        self.get_logger().info("Initializing RGB Detection Node")

        # ROS subscriber & publisher
        self.subscription = self.create_subscription(
            Image,
            '/zed/zed_node/rgb/image_rect_color',
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(RgbDetection, 'rgb_detections', 10)

        self.bridge = CvBridge()
        self.imgsz = imgsz
        self.visualize = visualize

        # Load YOLO model (.pt or TensorRT)
        self.model = YOLO(model_path)
        self.get_logger().info(f"Loaded YOLO model from: {model_path}")

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if frame is None or frame.size == 0:
                self.get_logger().error("Empty frame received")
                return
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Ensure 3 channels
        if frame.shape[2] == 4:
            frame = frame[:, :, :3]

        frame = np.ascontiguousarray(frame)

        # Run YOLO inference
        results = self.model(frame, imgsz=self.imgsz)[0]

        # Prepare and publish custom message
        det_msg = RgbDetection()
        det_msg.image = msg  # original image
        for box, conf, cls in zip(results.boxes.xyxy, results.boxes.conf, results.boxes.cls):
            obj = RgbObject()
            obj.name = self.model.names[int(cls)]
            obj.probability = float(conf)
            obj.x = float(box[0])
            obj.y = float(box[1])
            obj.width = float(box[2] - box[0])
            obj.height = float(box[3] - box[1])
            det_msg.objects.append(obj)

        self.publisher.publish(det_msg)

        # Optional visualization
        if self.visualize:
            annotated = results.plot()
            cv2.imshow("RGB Detection", annotated)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description="ROS2 node for YOLO inference on ZED")
    parser.add_argument('--model', type=str, required=True, help="Path to YOLO model (.pt or .engine)")
    parser.add_argument('--imgsz', type=int, default=640, help="Input image size")
    parser.add_argument('--visualize', action='store_true', help="Enable visualization")
    parsed_args, _ = parser.parse_known_args()

    node = RgbDetNode(parsed_args.model, imgsz=parsed_args.imgsz, visualize=parsed_args.visualize)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down RGB Detection Node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
