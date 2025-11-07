#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import numpy as np
import random
from spatialmath import SE3
from scipy.spatial.transform import Rotation as R  # Import scipy Rotation


# Import custom service
from controller_interfaces.srv import RandomTarget

# Import your robot
from lab4.rrr_dh import RRRRobot

class RandomPoseNode(Node):
    def __init__(self):
        super().__init__('random_pose_node')

        # Initialize robot for workspace calculation
        self.robot = RRRRobot()

        # Service server for Auto Mode to request new targets
        self.target_pub = self.create_publisher(PoseStamped, '/target', 10)
        self.random_target_service = self.create_service(RandomTarget, 'random_target', self.random_target_callback)

        # Calculate workspace bounds (spherical shell)
        # Based on link lengths: L2=0.12, L3=0.25, tool=0.28
        self.r_max = 0.12 + 0.25 + 0.28  # Maximum reach = 0.65m
        self.r_min = 0.10  # Minimum reach (conservative estimate)
        self.l = 0.2  # L1 offset (joint_1 height)

        # Debug: Print workspace info
        self.get_logger().info(f"Robot has {self.robot.n} joints")
        self.get_logger().info(f"Joint limits: {self.robot.qlim}")

        self.get_logger().info(f"Workspace: r_min={self.r_min}, r_max={self.r_max}, L1={self.l}")



    def publish_target(self, target_pos, q_solution):
        """Publish target position for RViz visualization with end effector orientation"""
        target_msg = PoseStamped()
        target_msg.header = Header()
        target_msg.header.stamp = self.get_clock().now().to_msg()
        target_msg.header.frame_id = "link_0"
        
        target_msg.pose.position.x = float(target_pos[0])
        target_msg.pose.position.y = float(target_pos[1])
        target_msg.pose.position.z = float(target_pos[2])
        
        # Get the orientation from the IK solution to match end effector
        T = self.robot.fkine(q_solution)
        # Extract rotation matrix
        rot_matrix = T.R  # 3x3 rotation matrix
          
        # Rotate by -90° around Y-axis so arrow (X-axis) points in direction of end effector Z-axis
        # This aligns the RViz arrow with the end effector's forward direction
        rotation_adjust = R.from_euler('y', -np.pi/2)
        r = R.from_matrix(rot_matrix) * rotation_adjust
        quat = r.as_quat()  # [x, y, z, w]
        
        target_msg.pose.orientation.x = float(quat[0])
        target_msg.pose.orientation.y = float(quat[1])
        target_msg.pose.orientation.z = float(quat[2])
        target_msg.pose.orientation.w = float(quat[3])
        
        self.target_pub.publish(target_msg)



    def inverse_kinematic(self, x, y, z):
        """
        Solve inverse kinematics for target position (x, y, z) with consistent orientation
        Returns joint angles if successful, None if IK fails
        """
        try:
            # Create SE3 transformation matrix for target position with consistent orientation
            # Use a consistent downward-pointing orientation for all targets
            T_target = SE3(x, y, z) @ SE3.Rx(np.pi/2)

            # Solve IK using Levenberg-Marquardt method - constrain position only
            sol = self.robot.ikine_LM(T_target, mask=[1, 1, 1, 0, 0, 0] ,limits=False)

            # Check if solution is valid
            if sol.success:
                return sol.q  # Return joint angles
            else:
                # Try with different seed
                sol = self.robot.ikine_LM(T_target, q0=np.zeros(self.robot.n), mask=[1, 1, 1, 0, 0, 0],limits=False)
                if sol.success:
                    return sol.q
                return None
        except:
            return None

    def random_target_callback(self, request, response):
        """Service callback for generating random targets in Auto Mode"""


        if request.request_new_target:
            position, q_solution = self.generate_random_pose()

            response.success = True
            response.target_position.x = float(position[0])
            response.target_position.y = float(position[1])
            response.target_position.z = float(position[2])
            response.message = f"Generated random target: [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]"
            self.publish_target(position, q_solution)
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.target_position.x = 0.0
            response.target_position.y = 0.0
            response.target_position.z = 0.0
            response.message = "No target requested"

        return response

    def generate_random_pose(self):
        """
        Generate a random valid pose within workspace using on-demand validation
        Keeps trying until finding a pose where IK succeeds
        Returns: tuple of (position, q_solution)
        """
        max_attempts = 100  # Safety limit to prevent infinite loop
        attempts = 0

        while attempts < max_attempts:
            # Generate random x, y, z within cubic bounds
            x = random.uniform(-self.r_max, self.r_max)
            y = random.uniform(-self.r_max, self.r_max)
            z = random.uniform(-self.r_max, self.r_max)

            # Check if point is within spherical shell
            # Distance from origin, accounting for L1 offset in z
            size_squared = x**2 + y**2 + (z - self.l)**2

            # Check if within reachable spherical shell
            if self.r_min**2 < size_squared < self.r_max**2:
                # Try inverse kinematics
                goal = self.inverse_kinematic(x, y, z)

                if goal is not None:
                    # IK succeeded! Return this pose and the solution
                    self.get_logger().info(f"Valid pose found after {attempts + 1} attempts")
                    return np.array([x, y, z]), goal

            attempts += 1

        # If all safe positions fail, return current robot position
        self.get_logger().error("All fallback positions failed, using current position")
        T_current = self.robot.fkine(self.robot.qz)
        current_pos = T_current.t
        return current_pos, self.robot.qz
    

def main(args=None):
    rclpy.init(args=args)
    node = RandomPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()