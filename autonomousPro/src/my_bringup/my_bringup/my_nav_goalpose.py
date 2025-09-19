import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from builtin_interfaces.msg import Time
import math
import time
from tf2_ros import Buffer, TransformListener

from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import Quaternion,Point,Twist

# from my_bringup.scripts import calc_pose
from common_interface.msg import RectDepth, Camera2map
from std_msgs.msg import Int32MultiArray, Float32MultiArray,String


from rclpy.action import ActionClient
from nav2_msgs.action import FollowPath
class SimpleNav(Node):
    def __init__(self):
        super().__init__('simple_nav_client')
        self.diff_x, self.diff_y = [0,0]
        self.theta = 0
        self.camera_pose = None
        self.goal_handle = None
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_smoothed', 10)
        self.target_depth = self.create_subscription(RectDepth, 'task/rect_depth', self.exec_callback, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.state_pub = self.create_publisher(String, '/robot_state', 10)
        self.camera2map_coodinate_pub = self.create_publisher(Camera2map, '/camera2map', 10)
        self.timer = self.create_timer(0.5, self.update_pose_from_tf) 


    def send_goal(self, x, y, theta=0.0):
        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(theta / 2.0)

        self.nav_to_pose_client.wait_for_server()

        self.state_pub.publish(String(data="navigating"))

        future = self.nav_to_pose_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

        self.get_logger().info(f"Sent goal to: x={x}, y={y}, theta={theta}")


    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            # self.state_pub.publish(String(data="failed"))
            self.get_logger().warn("❌ Goal rejected")
            return

        self.goal_handle = goal_handle  # ✅ 保存用于后续 cancel
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if status == 4:  # STATUS_SUCCEEDED
            self.get_logger().info("✅ Reached!")
            self.state_pub.publish(String(data="reachGoal"))
            self.reset_nav()
            self.goal_handle = None  # ✅ reset and clean
        else:
            self.get_logger().warn(f"⚠️ Navigation failed with status: {status}")
            self.state_pub.publish(String(data="failed"))

    def reset_nav(self):
        # 1. cancle current goal
        if self.goal_handle:
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(lambda f: self.get_logger().info("🛑 Goal cancel requested."))

        # 2. clean inner statuss
        self.goal_handle = None
        # self.state_pub.publish(String(data="idle"))  # idle: free and waiting
        self.get_logger().info("♻️ Nav2 state reset.")
        time.sleep(5)


    def exec_callback(self, msg):
        self.diff_x, self.diff_y = msg.coordinate_diff.data
        self.theta = msg.theta

        if self.camera_pose is not None:
            cam_x = self.camera_pose.pose.position.x
            cam_y = self.camera_pose.pose.position.y
            cam_yaw = self.get_yaw_from_quaternion(self.camera_pose.pose.orientation)
            self.get_logger().info(
                f"obj in map base: X:{ (self.diff_x * math.cos(cam_yaw) + self.diff_y * math.sin(cam_yaw)) },Y:{(- self.diff_x * math.sin(cam_yaw) + self.diff_y * math.cos(cam_yaw))}"
            )
            target_x = cam_x + self.diff_x * math.cos(cam_yaw) - self.diff_y * math.sin(cam_yaw)
            target_y = cam_y + self.diff_x * math.sin(cam_yaw) + self.diff_y * math.cos(cam_yaw)
            target_yaw = cam_yaw + self.theta

            self.get_logger().info(
                f"📦 Object in map → x={target_x:.2f}, y={target_y:.2f}, θ={target_yaw:.2f}"
            )

            self.send_goal(target_x, target_y, target_yaw)
        else:
            self.get_logger().warn("⚠️ No camera pose yet.")

    # def pose_callback(self, msg):
    #     self.current_pose = msg.pose
    #     self.get_logger().info(f"Current position: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}")


    def update_pose_from_tf(self):
        try:
            if not self.tf_buffer.can_transform(
                'map', 'camera_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0)
            ):
                self.get_logger().warn("⚠️ 18 TF transform map → camera_link not available yet.")
                return

            from geometry_msgs.msg import PoseStamped, Point, Quaternion

            tf = self.tf_buffer.lookup_transform('map', 'camera_link', rclpy.time.Time())

            pose = PoseStamped()
            pose.header = tf.header
            pose.pose.position = Point(
                x=tf.transform.translation.x,
                y=tf.transform.translation.y,
                z=tf.transform.translation.z
            )
            pose.pose.orientation = Quaternion(
                x=tf.transform.rotation.x,
                y=tf.transform.rotation.y,
                z=tf.transform.rotation.z,
                w=tf.transform.rotation.w
            )

            self.camera_pose = pose
            self.get_logger().info(
                f"📍 Current pose (camera_link in map): x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f},yaw = {self.get_yaw_from_quaternion(pose.pose.orientation)}"
            )
            coordinate_msg = Camera2map()
            coordinate_msg.coordinate = Float32MultiArray()
            yaw = self.get_yaw_from_quaternion(pose.pose.orientation)
            coordinate_msg.coordinate.data = [pose.pose.position.x, pose.pose.position.y, yaw]
            self.camera2map_coodinate_pub.publish(coordinate_msg)

        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    import math
    rclpy.init(args=args)
    node = SimpleNav()
    
    # delay to send goal，waiting for system ready
    import time
    time.sleep(2)

    # send_goal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
