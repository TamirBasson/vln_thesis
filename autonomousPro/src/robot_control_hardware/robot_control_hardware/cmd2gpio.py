#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from common_interface.msg import KeyCtrl
from gpiozero import PWMOutputDevice, DigitalOutputDevice
from builtin_interfaces.msg import Time

from action_msgs.msg import GoalStatusArray

class CmdVelToPwmGPIO(Node):
    LINEAR_SCALE = 0.8  # Linear velocity scale factor
    ANGULAR_SCALE = 3 # Angular velocity scale factor
    MAX_SPEED = 0.46
    MAX_REVERSE_SPEED = 0.42

    manualCtrl = True
    def __init__(self):
        super().__init__('cmd_vel_to_pwm_gpio')
        self.subscription_joy = self.create_subscription(KeyCtrl, '/cmd_joy', self.cmd_joy_callback, 10)
        self.subscription_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.subscription_navi_status= self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status',self.status_callback,10)
        # Robot parameters
        self.wheel_base = 0.475  # Wheel base (meters)
        self.linear_x = 0
        self.angular_z = 0
        self.v_left=0
        self.v_right=0
        self.navi_sta=0
        # GPIO settings
        try:
            self.pwm_l = PWMOutputDevice(18, frequency=500)
            self.pwm_r = PWMOutputDevice(13, frequency=500)
            self.is_reverse_l = DigitalOutputDevice(15)
            self.is_reverse_r = DigitalOutputDevice(19)
            self.gear_l = DigitalOutputDevice(14)
            self.gear_r = DigitalOutputDevice(26)
        except Exception as e:
            self.get_logger().error(f"GPIO initialization failed: {e}")
            return
        self.timer = self.create_timer(0.4, self.logSpeed)


    def logSpeed(self):
        # self.get_logger().info(f"vl:{self.v_left},vr:{self.v_right},vx:{self.linear_x},vz:{self.v_right}")
        self.get_logger().info(f"PWM -> Left: {self.pwm_l.value}, Right: {self.pwm_r.value} ")#| mode:{self.manualCtrl}")
    def status_callback(self, msg):
        for status in msg.status_list:
            self.get_logger().info(f'Status: {status.status}')
            self.navi_sta=status.status
    def cmd_joy_callback(self, msg):
        self.manualCtrl = not msg.allow_nav
        if self.manualCtrl:
            """ Process cmd_vel and control GPIO """
            linear_x = msg.manual_spd.linear.x * self.LINEAR_SCALE
            angular_z = msg.manual_spd.angular.z * self.ANGULAR_SCALE

            v_left = linear_x - (self.wheel_base / 2.0) * angular_z
            v_right = linear_x + (self.wheel_base / 2.0) * angular_z
            

            # Convert speed to PWM
            pwm_left = self.velocity_to_pwm(v_left)
            pwm_right = self.velocity_to_pwm(v_right)

            # Set direction
            self.is_reverse_l.value = pwm_left < 0
            self.is_reverse_r.value = pwm_right < 0

            # Set PWM range 0~1
            self.pwm_l.value = min(abs(pwm_left), self.MAX_SPEED)
            self.pwm_r.value = min(abs(pwm_right), self.MAX_SPEED)

            self.gear_l.value = False
            self.gear_r.value = False
        # self.get_logger().info(f"PWM -> Left: {self.pwm_l.value}, Right: {self.pwm_r.value} | mode:{self.manualCtrl}")
    def cmd_vel_callback(self,msg):
        if not self.manualCtrl:
            if not self.navi_sta == 4:
                self.linear_x = msg.linear.x * self.LINEAR_SCALE
                self.angular_z = msg.angular.z * self.ANGULAR_SCALE

            else:
                self.linear_x = 0
                self.angular_z = 0
                
            self.v_left = self.linear_x - (self.wheel_base / 2.0) * self.angular_z
            self.v_right = self.linear_x + (self.wheel_base / 2.0) * self.angular_z

            # self.get_logger().info(f"PWM -> Left: {self.pwm_l.value}, Right: {self.pwm_r.value}|mode{self.manualCtrl}")
            # v_left = v_left if v_left >= 0 else 0
            # v_right = v_right if v_right >= 0 else 0
            # Convert speed to PWM
            pwm_left = self.velocity_to_pwm(self.v_left)
            pwm_right = self.velocity_to_pwm(self.v_right)

            # Set direction
            self.is_reverse_l.value = pwm_left < 0
            self.is_reverse_r.value = pwm_right < 0

            # Set PWM range 0~1
            
            
            if self.is_reverse_l.value != self.is_reverse_r.value:
                self.pwm_l.value = min(abs(pwm_left), self.MAX_REVERSE_SPEED)
                self.pwm_r.value = min(abs(pwm_right), self.MAX_REVERSE_SPEED)
            else:
                self.pwm_l.value = min(abs(pwm_left), self.MAX_SPEED)
                self.pwm_r.value = min(abs(pwm_right), self.MAX_SPEED)
            # self.get_logger().info(f"PWM -> Left: {self.pwm_l.value}, Right: {self.pwm_r.value}|mode{self.manualCtrl}")

            self.gear_l.value = False
            self.gear_r.value = False
            
    def velocity_to_pwm(self, velocity):
        """ Convert speed to PWM value """
        threshold = 0.05
        if abs(velocity) < threshold:
            return 0  # Speed too low, stop
        pwm_value = (0.70 * abs(velocity) + 0.36)
        if velocity < 0:
            return -pwm_value 
        else:
            return pwm_value

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToPwmGPIO()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
