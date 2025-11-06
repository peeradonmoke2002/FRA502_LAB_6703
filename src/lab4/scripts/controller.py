#!/usr/bin/python3

import rclpy
from rclpy.node import Node

from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped, Twist,PoseStamped
from std_msgs.msg import String,Header
import numpy as np

import roboticstoolbox as rtb
from math import pi
from spatialmath import SE3
from scipy.spatial.transform import Rotation as R  # Import scipy Rotation
from sensor_msgs.msg import JointState  # Import JointState message

# DH Robot
from lab4.rrr_dh import RRRRobot



class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.robot = RRRRobot()
        self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        vz = msg.linear.z

        self.get_logger().info(f"Received cmd_vel: vx={vx}, vy={vy}, vz={vz}")

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
