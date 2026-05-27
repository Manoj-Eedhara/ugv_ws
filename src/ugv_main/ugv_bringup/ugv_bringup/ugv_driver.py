#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import json
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float32MultiArray, String
import subprocess
import time

class UgvDriver(Node):
    def __init__(self, name):
        super().__init__(name)

        # Publish serial commands to ugv_bringup which owns the port exclusively
        self.serial_cmd_pub = self.create_publisher(String, 'ugv/serial_cmd', 10)

        self.cmd_vel_sub_ = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
        self.joint_states_sub = self.create_subscription(JointState, 'ugv/joint_states', self.joint_states_callback, 10)
        self.led_ctrl_sub = self.create_subscription(Float32MultiArray, 'ugv/led_ctrl', self.led_ctrl_callback, 10)
        self.voltage_sub = self.create_subscription(Float32, 'voltage', self.voltage_callback, 10)

    def cmd_vel_callback(self, msg):
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        # Apply minimum threshold to angular velocity if linear velocity is zero
        if linear_velocity == 0:
            if 0 < angular_velocity < 0.2:
                angular_velocity = 0.2
            elif -0.2 < angular_velocity < 0:
                angular_velocity = -0.2

        data = json.dumps({'T': '13', 'X': linear_velocity, 'Z': angular_velocity})
        self.serial_cmd_pub.publish(String(data=data))

    def joint_states_callback(self, msg):
        name = msg.name
        position = msg.position

        x_rad = position[name.index('pt_base_link_to_pt_link1')]
        y_rad = position[name.index('pt_link1_to_pt_link2')]

        x_degree = (180 * x_rad) / 3.1415926
        y_degree = (180 * y_rad) / 3.1415926

        joint_data = json.dumps({
            'T': 134,
            'X': x_degree,
            'Y': y_degree,
            "SX": 600,
            "SY": 600,
        })
        self.serial_cmd_pub.publish(String(data=joint_data))

    def led_ctrl_callback(self, msg):
        IO4 = msg.data[0]
        IO5 = msg.data[1]

        led_ctrl_data = json.dumps({
            'T': 132,
            "IO4": IO4,
            "IO5": IO5,
        })
        self.serial_cmd_pub.publish(String(data=led_ctrl_data))

    def voltage_callback(self, msg):
        voltage_value = msg.data
        if 0.1 < voltage_value < 9:
            subprocess.run(['aplay', '-D', 'plughw:3,0', '/home/ws/ugv_ws/src/ugv_main/ugv_bringup/ugv_bringup/low_battery.wav'])
            time.sleep(5)

def main(args=None):
    rclpy.init(args=args)
    node = UgvDriver("ugv_driver")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
