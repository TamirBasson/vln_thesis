import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np

class RealSenseRGBDPublisher(Node):
    def __init__(self):
        super().__init__('realsense_rgbd_publisher')

        self.bridge = CvBridge()
        self.rgb_pub = self.create_publisher(Image, 'camera/rgb/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, 'camera/depth/image_raw', 10)

        # 配置 RealSense 管道
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(config)

        # 定时发布
        self.timer = self.create_timer(1.0 / 200.0, self.publish_frames)

    def publish_frames(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            self.get_logger().warn("无法获取帧")
            return

        # 转换为 numpy
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # 转为 ROS2 Image 消息
        rgb_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
        depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding='16UC1')  # 注意深度格式

        # 同时发布
        self.rgb_pub.publish(rgb_msg)
        self.depth_pub.publish(depth_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RealSenseRGBDPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.pipeline.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
