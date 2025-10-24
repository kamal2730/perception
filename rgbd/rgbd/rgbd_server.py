#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image
from custom_interfaces.msg import RgbDetection, RgbObject
from custom_interfaces.srv import Trigger
import cv2
from cv_bridge import CvBridge
import numpy as np
from ultralytics import YOLO
import threading
import time
import os


class RGBDServer(Node):
    def __init__(self, model_path, imgsz=640, visualize=False, bbox_scale=1.2):
        super().__init__('rgbd_server')
        self.get_logger().info("Starting RGBD Server Node")

        qos = QoSProfile(depth=10)
        self.cb_group = ReentrantCallbackGroup()

        self.subscription = self.create_subscription(
            Image,
            '/zed/zed_node/rgb/image_rect_color',
            self.image_callback,
            qos
        )
        self.publisher = self.create_publisher(RgbDetection, '/rgb_detections', qos)
        self.srv = self.create_service(
            Trigger, '/rgbd_trigger', self.trigger_callback, callback_group=self.cb_group
        )

        self.bridge = CvBridge()
        self.imgsz = imgsz
        self.visualize = visualize
        self.model = YOLO(model_path, task='detect')
        self.bbox_scale = bbox_scale
        self.get_logger().info(f"Loaded YOLO model from: {model_path}")

        self.frame = None
        self.latest_detection = None
        self.triggered = False
        self.lock = threading.Lock()

        # Warmup GPU
        self.get_logger().info("Warming up model on dummy image...")
        self.model(np.ones((imgsz, imgsz, 3), dtype=np.uint8) * 255)
        self.get_logger().info("Model ready ✅")

        self.timer = self.create_timer(1.0 / 15.0, self.publish_detections)
        self.worker = threading.Thread(target=self.inference_loop, daemon=True)
        self.worker.start()

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if frame is not None and frame.size > 0:
                with self.lock:
                    self.frame = frame.copy()
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")

    def trigger_callback(self, request, response):
        response.success = True
        if request.reset:
            with self.lock:
                self.latest_detection = None
            self.get_logger().info("Trigger received — Resetting")
            return response

        with self.lock:
            self.triggered = True
        self.get_logger().info("Trigger received — next frame will run YOLO inference")
        return response


    def inference_loop(self):
        while rclpy.ok():
            run_now = False
            with self.lock:
                if self.triggered and self.frame is not None:
                    frame = self.frame.copy()
                    self.triggered = False
                    run_now = True
                else:
                    frame = None

            if run_now and frame is not None:
                results = self.model(frame, imgsz=self.imgsz)[0]
                height, width = frame.shape[:2]

                det_msg = RgbDetection()
                for box, conf, cls in zip(results.boxes.xyxy, results.boxes.conf, results.boxes.cls):
                    x1, y1, x2, y2 = map(float, box)
                    box_w = x2 - x1
                    box_h = y2 - y1

                    # Enlarge the bbox around its center
                    cx = x1 + box_w / 2
                    cy = y1 + box_h / 2
                    new_w = box_w * self.bbox_scale
                    new_h = box_h * self.bbox_scale

                    new_x1 = int(cx - new_w / 2)
                    new_y1 = int(cy - new_h / 2)
                    new_x2 = int(cx + new_w / 2)
                    new_y2 = int(cy + new_h / 2)

                    # Clamp to image boundaries
                    new_x1 = max(0, new_x1)
                    new_y1 = max(0, new_y1)
                    new_x2 = min(width - 1, new_x2)
                    new_y2 = min(height - 1, new_y2)

                    obj = RgbObject()
                    obj.name = self.model.names[int(cls)]
                    obj.probability = float(conf)
                    obj.x = float(new_x1)
                    obj.y = float(new_y1)
                    obj.width = float(new_x2 - new_x1)
                    obj.height = float(new_y2 - new_y1)
                    det_msg.objects.append(obj)

                with self.lock:
                    self.latest_detection = det_msg

                if self.visualize:
                    # Draw enlarged boxes for visualization
                    annotated = frame.copy()
                    for obj in det_msg.objects:
                        x1, y1 = int(obj.x), int(obj.y)
                        x2 = int(obj.x + obj.width)
                        y2 = int(obj.y + obj.height)
                        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(
                            annotated,
                            f"{obj.name} ({obj.probability:.2f})",
                            (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 255, 0),
                            1
                        )
                    cv2.imshow("RGBD Detection", annotated)
                    cv2.waitKey(1)
            else:
                time.sleep(0.01)

    def publish_detections(self):
        with self.lock:
            msg = self.latest_detection
        if msg is None:
            self.publisher.publish(RgbDetection())
        else:
            self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RGBDServer(
        os.path.expanduser("~/rgb_det.engine"),
        imgsz=640,
        visualize=True,
        bbox_scale=1.5  # enlarge boxes by 20%
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down RGBD Server...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
