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
    def __init__(self, model_path, imgsz=640, visualize=False):
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
        with self.lock:
            self.triggered = True
        self.get_logger().info("Trigger received — next frame will run YOLO inference")
        response.success = True
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

                det_msg = RgbDetection()
                for box, conf, cls in zip(results.boxes.xyxy, results.boxes.conf, results.boxes.cls):
                    obj = RgbObject()
                    obj.name = self.model.names[int(cls)]
                    obj.probability = float(conf)
                    obj.x = float(box[0])
                    obj.y = float(box[1])
                    obj.width = float(box[2] - box[0])
                    obj.height = float(box[3] - box[1])
                    det_msg.objects.append(obj)

                with self.lock:
                    self.latest_detection = det_msg

                if self.visualize:
                    annotated = results.plot()
                    cv2.imshow("RGBD Detection", annotated)
                    cv2.waitKey(1)
            else:
                time.sleep(0.01)

    def publish_detections(self):
        msg = None
        with self.lock:
            msg = self.latest_detection

        if msg is None:
            self.publisher.publish(RgbDetection())
        else:
            self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RGBDServer(os.path.expanduser("~/rgb_det.engine"), imgsz=640, visualize=True)
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
