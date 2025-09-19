#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException

from scipy.spatial.transform import Rotation as R 
import math
import cv2
from cv_bridge import CvBridge

import csv
import os
import numpy as np
import shutil


class ImageTfSaver(Node):
    def __init__(self):
        super().__init__('image_tf_saver')

        # === Session Cleanup ===
        if os.path.exists('poses.csv'):
            os.remove('poses.csv')

        self.image_folder = 'image_data'
        if os.path.exists(self.image_folder):
            shutil.rmtree(self.image_folder)
        os.makedirs(self.image_folder, exist_ok=True)

        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Camera subscriber
        self.image_sub = self.create_subscription(
            Image,
            '/camera_sensor/image_raw',
            self.image_callback,
            5
        )

        self.cv_bridge = CvBridge()

        # CSV setup
        self.csv_file = open('poses.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["x", "y", "theta_deg"])

        # Track last position
        self.last_position = None
        self.min_distance = 1.0  # meters

    def image_callback(self, msg: Image):
        stamp = msg.header.stamp
        timestamp = stamp.sec + stamp.nanosec * 1e-9

        try:
            tf_time = Time.from_msg(stamp)
            transform = self.tf_buffer.lookup_transform(
                'map', 'camera_link', tf_time)

            # Translation
            x = transform.transform.translation.x
            y = transform.transform.translation.y

            # Rotation to yaw using scipy
            quat = transform.transform.rotation
            r = R.from_quat([quat.x, quat.y, quat.z, quat.w])
            yaw_rad = r.as_euler('xyz')[2]
            theta_deg = math.degrees(yaw_rad)

            # Check distance from last position
            if self.last_position is not None:
                dx = x - self.last_position[0]
                dy = y - self.last_position[1]
                dist = math.sqrt(dx**2 + dy**2)
                if dist < self.min_distance:
                    return  # skip if not moved enough

            # Save image
            rgb_img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            filename_base = f"{x:.2f}_{y:.2f}_{theta_deg:.1f}"
            img_path = os.path.join(self.image_folder, f"{filename_base}.png")
            cv2.imwrite(img_path, rgb_img)

            # Write to CSV
            self.csv_writer.writerow([f"{x:.2f}", f"{y:.2f}", f"{theta_deg:.1f}"])
            self.csv_file.flush()  # Ensure data is written immediately
            self.get_logger().info(f"Saved image to {img_path}")
            self.get_logger().info(f"Pose: x={x:.2f}, y={y:.2f}, theta={theta_deg:.1f}")
            # Update last position
            self.last_position = (x, y)

            self.get_logger().info(f"Saved {filename_base}.png")
        except (LookupException, ExtrapolationException) as e:
            self.get_logger().warn(f"TF lookup failed: {str(e)}")
        except cv2.error as e:
            self.get_logger().error(f"OpenCV error: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImageTfSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
