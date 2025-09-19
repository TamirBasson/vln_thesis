#!/usr/bin/env python3
import os
import sys
import time

import cv2
import numpy as np
import pyrealsense2 as rs

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ultralytics import YOLO

class YoloRealsensePublisher(Node):
    def __init__(self):
        super().__init__('yolo_realsense_publisher')
        # Publisher for the detection image topic
        self.publisher_ = self.create_publisher(Image, '/image_detected', 10)
        self.bridge = CvBridge()

        # Hardcoded parameters
        model_path = '/host_root/home/tamir/yolo/yolo11n.pt'
        min_thresh = 0.5
        res_width = 640
        res_height = 480

        # Verify model exists
        if not os.path.exists(model_path):
            self.get_logger().error(f"Model file {model_path} not found!")
            sys.exit(1)

        # Load the YOLO model
        self.model = YOLO(model_path, task='detect')
        self.labels = self.model.names

        self.min_thresh = min_thresh
        self.res_width = res_width
        self.res_height = res_height

        # Setup the RealSense pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, res_width, res_height, rs.format.bgr8, 30)
        self.pipeline.start(config)

        # Define bounding box colors (Tableau 10 colors)
        self.bbox_colors = [
            (164, 120, 87), (68, 148, 228), (93, 97, 209),
            (178, 182, 133), (88, 159, 106), (96, 202, 231),
            (159, 124, 168), (169, 162, 241), (98, 118, 150),
            (172, 176, 184)
        ]

        # Create a timer to process frames at 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # Get a frame from the RealSense camera
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            self.get_logger().warning("No frame received from RealSense")
            return

        frame = np.asanyarray(color_frame.get_data())

        # Run YOLO inference on the frame
        results = self.model(frame, verbose=False)
        detections = results[0].boxes

        object_count = 0

        # Process each detection
        for i in range(len(detections)):
            # Convert detection tensor to numpy array
            xyxy_tensor = detections[i].xyxy.cpu()
            xyxy = xyxy_tensor.numpy().squeeze()
            if xyxy.size == 0:
                continue

            xmin, ymin, xmax, ymax = xyxy.astype(int)
            class_idx = int(detections[i].cls.item())
            classname = self.labels[class_idx]
            conf = detections[i].conf.item()

            # Draw bounding box if confidence exceeds threshold
            if conf > self.min_thresh:
                color = self.bbox_colors[class_idx % len(self.bbox_colors)]
                cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), color, 2)
                label = f'{classname}: {int(conf * 100)}%'
                labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                label_ymin = max(ymin, labelSize[1] + 10)
                cv2.rectangle(frame, (xmin, label_ymin - labelSize[1] - 10),
                              (xmin + labelSize[0], label_ymin + baseLine - 10),
                              color, cv2.FILLED)
                cv2.putText(frame, label, (xmin, label_ymin - 7),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
                object_count += 1

        # (Optional) Draw object count on the image
        cv2.putText(frame, f'Number of objects: {object_count}', (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # Convert the image to a ROS Image message and publish it
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(img_msg)
        self.get_logger().info("Published image with detections")

    def destroy_node(self):
        # Stop the RealSense pipeline on shutdown
        self.pipeline.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = YoloRealsensePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
