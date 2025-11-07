import py_trees
import py_trees_ros
from controller_interfaces.srv import SetMode, InverseKinematics, RandomTarget
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
import numpy as np
from spatialmath import SE3
from scipy.spatial.transform import Rotation as R

# TO Mode Behaviors

class TOMode(py_trees.behaviour.Behaviour):
    """
    Teleoperation mode behavior that controls robot using cmd_vel.
    Supports both world frame and tool frame control.
    """

    def __init__(self, name, node, robot, tf_buffer):
        super(TOMode, self).__init__(name)
        self.node = node
        self.robot = robot
        self.tf_buffer = tf_buffer

        # Publishers and subscribers
        self.joint_publisher = None
        self.cmd_vel_sub = None
        self.teleop_frame_sub = None
        self.reset_pose_sub = None
        self.singularity_pub = None

        # State variables
        self.cmd_vel = np.array([0.0, 0.0, 0.0])
        self.teleop_frame = "tool"  # Default is end-effector/tool frame
        self.initial_pose = np.array([0.0, 0.0, -np.pi/2])

        # Control parameters
        self.dt = 0.01  # 100Hz update rate
        self.source_frame = 'link_0'
        self.target_frame = 'end_effector'

    def setup(self, **kwargs):
        """Create publishers and subscribers once"""
        if self.joint_publisher is None:
            self.joint_publisher = self.node.create_publisher(JointState, 'joint_states', 10)

        if self.cmd_vel_sub is None:
            self.cmd_vel_sub = self.node.create_subscription(
                Twist, 'cmd_vel', self.cmd_vel_callback, 10
            )

        if self.teleop_frame_sub is None:
            self.teleop_frame_sub = self.node.create_subscription(
                String, '/teleop_frame', self.teleop_frame_callback, 10
            )

        if self.reset_pose_sub is None:
            self.reset_pose_sub = self.node.create_subscription(
                String, '/reset_pose', self.reset_pose_callback, 10
            )

        if self.singularity_pub is None:
            self.singularity_pub = self.node.create_publisher(String, '/singularity_warning', 10)

    def cmd_vel_callback(self, msg):
        """Callback for cmd_vel topic"""
        self.cmd_vel = np.array([msg.linear.x, msg.linear.y, msg.linear.z])

    def teleop_frame_callback(self, msg):
        """Callback to switch teleoperation frame (world or tool)"""
        frame = msg.data.lower()
        if frame in ["world", "tool"]:
            self.teleop_frame = frame
            self.node.get_logger().info(f"Teleop frame switched to: {self.teleop_frame}")
        else:
            self.node.get_logger().warn(f"Invalid teleop frame '{msg.data}'. Use 'world' or 'tool'")

    def reset_pose_callback(self, msg):
        """Callback to reset robot to initial pose"""
        if msg.data.lower() == "reset":
            self.robot.qz = self.initial_pose.copy()
            self.cmd_vel = np.array([0.0, 0.0, 0.0])
            self.node.get_logger().info("Robot reset to initial pose")
            # Immediately publish the reset position
            self.publish_joint_states()

    def get_transform(self):
        """Get current transform from TF buffer"""
        try:
            import rclpy
            transform = self.tf_buffer.lookup_transform(
                self.source_frame,
                self.target_frame,
                rclpy.time.Time()
            )
        except Exception as e:
            self.node.get_logger().warn(f"TF not ready: {e}", throttle_duration_sec=5.0)
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

    def update(self):
        """Teleoperation control loop"""
        # Get current end-effector pose
        T_current = self.get_transform()
        if T_current is None:
            # TF not ready, just return and try again next tick
            return py_trees.common.Status.RUNNING

        # Extract rotation matrix
        T_rot = T_current[0:3, 0:3]

        # Compute desired velocity based on frame
        if self.teleop_frame == "tool":
            # Tool/end-effector frame: Transform cmd_vel to base frame
            p_dot = T_rot @ self.cmd_vel
        else:
            # World/base frame: Use cmd_vel directly
            p_dot = self.cmd_vel

        # Compute Jacobian
        J = self.robot.jacob0(self.robot.qz)
        J_pos = J[:3, :]  # Position part only

        # Singularity check
        condJ = np.linalg.cond(J_pos)
        if condJ > 1e3:
            # Near singularity - stop motion
            warning_msg = String()
            warning_msg.data = f"SINGULARITY DETECTED! Condition number: {condJ:.2e}"
            self.singularity_pub.publish(warning_msg)
            self.node.get_logger().warn(warning_msg.data, throttle_duration_sec=2.0)

            # Stop all motion
            self.cmd_vel = np.array([0.0, 0.0, 0.0])
            self.publish_joint_states()
            return py_trees.common.Status.RUNNING

        # Compute joint velocities using pseudo-inverse
        dq = np.linalg.pinv(J_pos) @ p_dot

        # Update robot position
        self.robot.qz = self.robot.qz + dq * self.dt

        # Log status
        if np.linalg.norm(p_dot) > 0.001:  # Only log when moving
            self.node.get_logger().info(
                f"TO: Frame={self.teleop_frame}, v={p_dot}, dq={dq}",
                throttle_duration_sec=1.0
            )

        # Publish joint states
        self.publish_joint_states()

        return py_trees.common.Status.RUNNING

    def publish_joint_states(self):
        """Publish current joint states"""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = ['joint_1', 'joint_2', 'joint_3']
        msg.position = [float(self.robot.qz[0]), float(self.robot.qz[1]), float(self.robot.qz[2])]
        self.joint_publisher.publish(msg)
