#!/usr/bin/python3

import rclpy
from rclpy.node import Node

from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped, Twist, PoseStamped
from std_msgs.msg import String, Header
import numpy as np

import roboticstoolbox as rtb
from spatialmath import SE3
from scipy.spatial.transform import Rotation as R  # Import scipy Rotation
from sensor_msgs.msg import JointState  # Import JointState message

# Custom service
from controller_interfaces.srv import SetMode, InverseKinematics, RandomTarget

# DH Robot
from lab4.rrr_dh import RRRRobot
from lab4.bt_mode import IsWhatMode
from lab4.bt_ipk_mode import IPKMode
from lab4.bt_to_mode import TOMode
from lab4.bt_am_mode import AMMode
import py_trees
import py_trees_ros

class ControllerBTNode(Node):
    def __init__(self):
        super().__init__('controller_bt_node')

        # Initialize robot
        self.robot = RRRRobot()
        self.initial_pose = np.array([0.0, 0.0, -np.pi/2])
        self.robot.qz = self.initial_pose.copy()

        # Initialize TF buffer for TO mode
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.target_pub = self.create_publisher(PoseStamped, '/target', 10)
        self.endeff_pub = self.create_publisher(PoseStamped, '/end_effector', 10)

        # Create services
        self.mode_service = self.create_service(SetMode, 'set_mode', self.set_mode_callback)
        self.ipk_service = self.create_service(InverseKinematics, 'inverse_kinematics', self.ipk_service_callback)

        # Create behavior tree
        self.tree = None
        self.create_behavior_tree()

        self.dt_loop = 1.0 / 100.0  # 100 Hz update rate
        self.timer = self.create_timer(self.dt_loop, self.tick_callback)
        self.prev_time = self.get_clock().now()


    def set_mode_callback(self, request, response):
        """Service callback to switch controller mode - updates blackboard"""
        requested_mode = request.mode.upper()
        valid_modes = ["IPK", "TO", "AM"]

        if requested_mode in valid_modes:
            # Update blackboard so IsWhatMode behaviors can see the change
            self.blackboard.mode = requested_mode
            response.success = True
            response.message = f"Mode switched to {requested_mode}"
            self.get_logger().info(response.message)

        else:
            response.success = False
            response.message = f"Invalid mode '{request.mode}'. Valid modes: {valid_modes}"
            self.get_logger().warn(response.message)

        return response

    def ipk_service_callback(self, request, response):
        """Service callback for Inverse Position Kinematics"""
        target_pos = np.array([
            request.target_position.x,
            request.target_position.y,
            request.target_position.z
        ])

        self.get_logger().info(f"IPK service called for target: {target_pos}")

        # Solve IK to check if target is reachable
        q_solution = self.inverse_kinematic(target_pos)

        if q_solution is not None:
            # Store target on blackboard for IPKMode behavior to move towards
            self.blackboard.target_position = target_pos
            # Publish target with the orientation from IK solution
            self.publish_target(target_pos, q_solution)
            response.success = True
            response.solution = q_solution.tolist()
            response.message = f"IK solved successfully - robot will move to target"
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.solution = []
            response.message = f"IK failed for target {target_pos} - target unreachable"
            self.get_logger().warn(response.message)

        return response

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

    def inverse_kinematic(self, target_pos):
        """Solve inverse kinematics for target position with consistent orientation"""
        try:
            # Create target transformation with position and desired orientation
            # Use a consistent downward-pointing orientation for all targets
            T_target = SE3(target_pos[0], target_pos[1], target_pos[2]) @ SE3.Rx(np.pi/2)

            # Method 1: LM with current position as seed - constrain position only
            sol = self.robot.ikine_LM(T_target, q0=self.robot.qz, mask=[1, 1, 1, 0, 0, 0])
            if sol.success:
                return sol.q

            # Method 2: LM with zero configuration
            sol = self.robot.ikine_LM(T_target, q0=np.zeros(self.robot.n), mask=[1, 1, 1, 0, 0, 0])
            if sol.success:
                return sol.q

            # Method 3: Random seed
            q_random = np.random.uniform(
                self.robot.qlim[0, :],
                self.robot.qlim[1, :],
                self.robot.n
            )
            sol = self.robot.ikine_LM(T_target, q0=q_random, mask=[1, 1, 1, 0, 0, 0])
            if sol.success:
                return sol.q

            return None

        except Exception as e:
            self.get_logger().error(f"IK exception: {e}")
            return None

    def create_behavior_tree(self):
            """Build and setup the behavior tree."""
            # Initialize blackboard BEFORE creating tree
            self.blackboard = py_trees.blackboard.Client(name="ControllerBTNode")
            self.blackboard.register_key(key="mode", access=py_trees.common.Access.WRITE)
            self.blackboard.register_key(key="target_position", access=py_trees.common.Access.WRITE)
            self.blackboard.mode = "TO"  # Default to Teleoperation mode
            self.blackboard.target_position = None
            self.tree = self.create_queue_tree()

    def create_queue_tree(self):
        # Root: Selector picks which mode to run
        root = py_trees.composites.Selector(name="Root", memory=False)

        # IPK Mode: Sequence of [Check → Execute]
        ipk_seq = py_trees.composites.Sequence(name="IPK_Mode", memory=False)
        ipk_seq.add_children([
            IsWhatMode(name="CheckIPK", mode="IPK"),
            IPKMode(name="RunIPK", node=self, robot=self.robot)
        ])

        # TO Mode: Sequence of [Check → Execute]
        to_seq = py_trees.composites.Sequence(name="TO_Mode", memory=False)
        to_seq.add_children([
            IsWhatMode(name="CheckTO", mode="TO"),
            TOMode(name="RunTO", node=self, robot=self.robot, tf_buffer=self.tf_buffer)
        ])

        # AM Mode: Sequence of [Check → Execute]
        am_seq = py_trees.composites.Sequence(name="AM_Mode", memory=False)
        am_seq.add_children([
            IsWhatMode(name="CheckAM", mode="AM"),
            AMMode(name="RunAM", node=self, robot=self.robot)
        ])

        # Root selector tries each mode sequence in order
        root.add_children([ipk_seq, to_seq, am_seq])


        # Create tree
        tree = py_trees_ros.trees.BehaviourTree(root, unicode_tree_debug=False)

        snapshot_visitor = py_trees.visitors.SnapshotVisitor()
        tree.visitors.append(snapshot_visitor)
        self.snapshot_visitor = snapshot_visitor

        tree.setup(timeout=15.0, node=self)
        return tree

    def post_tick_handler(self, snapshot_visitor, behaviour_tree):
        print(
            py_trees.display.unicode_tree(
                behaviour_tree.root,
                visited=snapshot_visitor.visited,
                previously_visited=snapshot_visitor.visited
            )
        )

    def get_transform(self):
        """Get current end-effector transform from TF"""
        try:
            import rclpy
            transform = self.tf_buffer.lookup_transform(
                'link_0',
                'end_effector',
                rclpy.time.Time()
            )
        except Exception:
            return None

        pos = transform.transform.translation
        ori = transform.transform.rotation
        quat = [ori.x, ori.y, ori.z, ori.w]

        # Convert quaternion to rotation matrix
        rot = R.from_quat(quat).as_matrix()

        T = np.eye(4)
        T[0:3, 0:3] = rot
        T[0:3, 3] = [pos.x, pos.y, pos.z]
        return T

    def publish_end_effector(self):
        """Publish current end-effector position for RViz visualization"""
        T = self.get_transform()
        if T is None:
            return

        position = T[0:3, 3]
        rot_matrix = T[0:3, 0:3]

        endeff_msg = PoseStamped()
        endeff_msg.header = Header()
        endeff_msg.header.stamp = self.get_clock().now().to_msg()
        endeff_msg.header.frame_id = "link_0"
        endeff_msg.pose.position.x = float(position[0])
        endeff_msg.pose.position.y = float(position[1])
        endeff_msg.pose.position.z = float(position[2])

        rotation_adjust = R.from_euler('y', -np.pi/2)
        r = R.from_matrix(rot_matrix) * rotation_adjust
        quat = r.as_quat()  # [x, y, z, w]

        endeff_msg.pose.orientation.x = float(quat[0])
        endeff_msg.pose.orientation.y = float(quat[1])
        endeff_msg.pose.orientation.z = float(quat[2])
        endeff_msg.pose.orientation.w = float(quat[3])

        self.endeff_pub.publish(endeff_msg)

    def tick_callback(self):
        self.tree.tick()
        self.publish_end_effector()


def main(args=None):
    rclpy.init(args=args)
    node = ControllerBTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Controller stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
