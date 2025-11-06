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
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Define the frames for transform lookup
        self.source_frame = 'link_0'  # Base frame
        self.target_frame = 'end_effector'  # End effector frame

        self.create_subscription(JointState, "joint_states", self.joint_state_callback, 10)
        self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        vz = msg.linear.z

        # self.get_logger().info(f"Received cmd_vel: vx={vx}, vy={vy}, vz={vz}")

    def joint_state_callback(self, msg):
        joint_positions = msg.position
        self.robot.qz = np.array(joint_positions)
        # verify by logging
        self.get_logger().info(f"Received joint states: {self.robot.qz}")

        # Get transform once and use the result
        position, rotation = self.get_transform()
        if position is not None:
            self.get_logger().info(f"End Effector Position: {position}")
            self.get_logger().info(f"End Effector Rotation Matrix:\n{rotation}")


    def get_transform(self):
        # Get the latest available transform
        try:
            transform = self.tf_buffer.lookup_transform(
                self.source_frame, 
                self.target_frame, 
                rclpy.time.Time()  # Use Time() for latest available transform
            )
        except Exception as e:
            self.get_logger().error(f"Transform lookup failed: {e}")
            return None, None
        
        position = transform.transform.translation
        orientation = transform.transform.rotation
        
        x = position.x
        y = position.y
        z = position.z
        
        # Extract quaternion components
        qx = orientation.x
        qy = orientation.y
        qz = orientation.z
        qw = orientation.w

        rotation = R.from_quat([qx, qy, qz, qw])  # scipy expects [x, y, z, w]
        rotation_matrix = rotation.as_matrix()  # Get 3x3 rotation matrix

        return np.array([x, y, z]), rotation_matrix

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
