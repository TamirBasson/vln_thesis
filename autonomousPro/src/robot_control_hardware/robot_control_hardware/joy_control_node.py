import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from common_interface.msg import KeyCtrl
'''
# use for read rc input and translate to Control input
'''
class MyJoy(Node):
    DEADZONE = 0.05  # Set deadzone threshold
    def __init__(self):
        super().__init__('joy_control_node')
        # Create a subscription to the "/joy" topic for Joy messages
        self.subscription = self.create_subscription(Joy,'/joy',self.callback,10)
        # Create a publisher for Twist messages on the "/cmd_vel_joy" topic
        self.publisher = self.create_publisher(KeyCtrl, '/cmd_joy', 10)
        # self.publisher = self.create_publisher(KeyStatus, '/cmd_key',30)

        self.manual_mode = True  # Set manual mode flag
        self.get_logger().info("waitting for joy control")
    def callback(self, msg):
        KeyCtrlMsg = KeyCtrl()  # Create a KeyStatus message to hold key status
        if len(msg.buttons) > 0:
            KeyCtrlMsg.allow_nav = (msg.buttons[0] == 1)
            self.manual_mode = not KeyCtrlMsg.allow_nav
        else:
            self.manual_mode = True

        if self.manual_mode:
            KeyCtrlMsg.manual_spd.linear.x = self.apply_deadzone(msg.axes[1])
            KeyCtrlMsg.manual_spd.angular.z = self.apply_deadzone(msg.axes[0])

        self.publisher.publish(KeyCtrlMsg)
        # self.get_logger().info(f"{self.manual_mode},| {KeyCtrlMsg.manual_spd.linear.x}")
    # Handle joystick deadzone
    def apply_deadzone(self, value, deadzone=DEADZONE):
        if abs(value) < deadzone:
            return 0.0
        return value
    
def main(args=None):
    rclpy.init(args=args)
    node = MyJoy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()