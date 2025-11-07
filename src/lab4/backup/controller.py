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


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.robot = RRRRobot()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Define the frames for transform lookup
        self.source_frame = 'link_0'  # Base frame
        self.target_frame = 'end_effector'  # End effector frame
        self.joint_names = ['joint_1', 'joint_2', 'joint_3']
        self.cmd_vel = np.array([0.0, 0.0, 0.0])  # Initialize command velocity

        # Controller mode: "IPK", "TO", "AM"
        self.mode = "TO"  # Default mode is Teleoperation
        self.target_position = None  # Target position for IPK and AM modes

        # Teleoperation frame mode: "world" or "tool"
        self.teleop_frame = "tool"  # Default is end-effector/tool frame

        # Initialize joint positions
        self.initial_pose = np.array([0.0, 0.0, -np.pi/2])
        self.robot.qz = self.initial_pose.copy()

        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
        self.create_subscription(PoseStamped, "/target", self.target_callback, 10)
        self.create_subscription(String, "/teleop_frame", self.teleop_frame_callback, 10)
        self.create_subscription(String, "/reset_pose", self.reset_pose_callback, 10)

        # pub target and end_effector
        self.target_pub = self.create_publisher(PoseStamped, "/target", 10)
        self.endeff_pub = self.create_publisher(PoseStamped, "/end_effector", 10)

        # Publisher for singularity warning
        self.singularity_pub = self.create_publisher(String, "/singularity_warning", 10)

        # Service server for mode switching
        self.mode_service = self.create_service(SetMode, 'set_mode', self.set_mode_callback)

        # Service server for IPK (Inverse Position Kinematics)
        self.ipk_service = self.create_service(InverseKinematics, 'inverse_kinematics', self.ipk_service_callback)

        # Service client for Auto Mode - request random targets
        self.random_target_client = self.create_client(RandomTarget, 'random_target')

        # Auto Mode state
        self.am_target_reached = False
        self.am_wait_start_time = None
        self.am_wait_duration = 10.0  # 10 seconds wait
        self.am_requesting_target = False
        self.am_failed = False  # Flag to indicate Auto Mode has failed

        self.dt_loop = 1.0 / 100.0  # 100 Hz update rate
        self.timer = self.create_timer(self.dt_loop, self.timer_callback)
        self.prev_time = self.get_clock().now()

        self.get_logger().info(f"Controller initialized. Current mode: {self.mode}")

    def target_publisher(self, position):
        target_msg = PoseStamped()
        target_msg.header = Header()
        target_msg.header.stamp = self.get_clock().now().to_msg()
        target_msg.header.frame_id = self.source_frame          # <-- set frame_id
        target_msg.pose.position.x = float(position[0])
        target_msg.pose.position.y = float(position[1])
        target_msg.pose.position.z = float(position[2])
        self.target_pub.publish(target_msg)

    def endeff_publisher(self, position):
        endeff_msg = PoseStamped()
        endeff_msg.header = Header()
        endeff_msg.header.stamp = self.get_clock().now().to_msg()
        endeff_msg.header.frame_id = self.source_frame
        endeff_msg.pose.position.x = float(position[0])
        endeff_msg.pose.position.y = float(position[1])
        endeff_msg.pose.position.z = float(position[2])
        self.endeff_pub.publish(endeff_msg)

    def cmd_vel_callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        vz = msg.linear.z

        self.cmd_vel = np.array([vx, vy, vz])

    def teleop_frame_callback(self, msg):
        """Callback to switch teleoperation frame (world or tool)"""
        frame = msg.data.lower()
        if frame in ["world", "tool"]:
            self.teleop_frame = frame
            self.get_logger().info(f"Teleop frame switched to: {self.teleop_frame}")
        else:
            self.get_logger().warn(f"Invalid teleop frame '{msg.data}'. Use 'world' or 'tool'")

    def reset_pose_callback(self, msg):
        """Callback to reset robot to initial pose"""
        if msg.data.lower() == "reset":
            self.robot.qz = self.initial_pose.copy()
            self.cmd_vel = np.array([0.0, 0.0, 0.0])
            self.get_logger().info("Robot reset to initial pose")
            # Immediately publish the reset position
            self.pub_joint_states(self.robot.qz[0], self.robot.qz[1], self.robot.qz[2])

    def target_callback(self, msg):
        """Callback for /target topic - stores target position for IPK and AM modes"""
        self.target_position = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
        self.get_logger().info(f"New target received: {self.target_position}")

    def set_mode_callback(self, request, response):
        """Service callback to switch controller mode"""
        requested_mode = request.mode.upper()
        valid_modes = ["IPK", "TO", "AM"]

        if requested_mode in valid_modes:
            self.mode = requested_mode
            response.success = True
            response.message = f"Mode switched to {self.mode}"
            self.get_logger().info(response.message)

            # Reset velocities when switching modes
            self.cmd_vel = np.array([0.0, 0.0, 0.0])

            # Reset Auto Mode state when switching
            if requested_mode == "AM":
                self.am_target_reached = False
                self.am_wait_start_time = None
                self.am_requesting_target = False
                self.am_failed = False  # Reset failure flag
                self.target_position = None  # Force request of new target

        else:
            response.success = False
            response.message = f"Invalid mode '{request.mode}'. Valid modes: {valid_modes}"
            self.get_logger().warn(response.message)

        return response

    def request_random_target(self):
        """Request a new random target from random_pos node"""
        if not self.random_target_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Random target service not available")
            return False

        request = RandomTarget.Request()
        request.request_new_target = True

        self.get_logger().info(f"[SERVICE CALL] Calling /random_target with request_new_target={request.request_new_target}")

        future = self.random_target_client.call_async(request)
        future.add_done_callback(self.random_target_response_callback)
        self.am_requesting_target = True
        return True

    def random_target_response_callback(self, future):
        """Callback for random target service response"""
        try:
            response = future.result()
            self.get_logger().info(f"[SERVICE RESPONSE] Received response: success={response.success}, message='{response.message}'")
            self.get_logger().info(f"[SERVICE RESPONSE] Target position: ({response.target_position.x:.3f}, {response.target_position.y:.3f}, {response.target_position.z:.3f})")

            if response.success:
                self.target_position = np.array([
                    response.target_position.x,
                    response.target_position.y,
                    response.target_position.z
                ])
                self.am_target_reached = False
                self.am_wait_start_time = None
                self.get_logger().info(f"New random target received: {self.target_position}")
            else:
                # Service failed - stop Auto Mode operation completely
                self.get_logger().error(f"[FAILURE] Random target service failed: {response.message}")
                self.get_logger().error("[FAILURE] Setting am_failed=True to stop Auto Mode")
                self.get_logger().error("[FAILURE] Auto Mode STOPPED. Please switch modes.")
                # Set failure flag to completely stop Auto Mode
                self.am_failed = True
                self.get_logger().error(f"[FAILURE] am_failed is now: {self.am_failed}")
                # Keep old target_position if it exists, or set to current position to stop movement
                if self.target_position is None:
                    # No previous target, use current end-effector position
                    T_current = self.get_transform()
                    if T_current is not None:
                        self.target_position = T_current[0:3, 3]
                # Mark as reached and clear wait timer to fully stop Auto Mode
                self.am_target_reached = True
                self.am_wait_start_time = None  # Clear wait timer to prevent retry
        except Exception as e:
            self.get_logger().error(f"Service call exception: {e}")
            self.get_logger().error("Auto Mode STOPPED due to exception. Please switch modes.")
            # Set failure flag to completely stop Auto Mode
            self.am_failed = True
            # On exception, stop Auto Mode completely
            if self.target_position is None:
                T_current = self.get_transform()
                if T_current is not None:
                    self.target_position = T_current[0:3, 3]
            self.am_target_reached = True
            self.am_wait_start_time = None  # Clear wait timer to prevent retry
        finally:
            self.am_requesting_target = False

    def ipk_service_callback(self, request, response):
        """Service callback for Inverse Position Kinematics"""
        target_pos = np.array([
            request.target_position.x,
            request.target_position.y,
            request.target_position.z
        ])

        self.get_logger().info(f"IPK service called for target: {target_pos}")

        # Solve IK
        q_solution = self.inverse_kinematic(target_pos)

        if q_solution is not None:
            # IK succeeded - move robot to target
            self.robot.qz = q_solution
            self.pub_joint_states(self.robot.qz[0], self.robot.qz[1], self.robot.qz[2])

            response.success = True
            response.solution = q_solution.tolist()
            response.message = f"IK solved successfully: q={q_solution}"
            self.get_logger().info(response.message)

            # Also publish target position for visualization
            self.target_publisher(target_pos)
        else:
            # IK failed - robot stays at current position
            response.success = False
            response.solution = []
            response.message = f"IK failed for target {target_pos}. Target may be unreachable."
            self.get_logger().warn(response.message)

        return response


    def inverse_kinematic(self, target_pos):
        """
        Solve inverse kinematics for target position
        Returns joint angles if successful, None if IK fails
        """
        try:
            # Create SE3 transformation matrix for target position
            T_target = SE3(target_pos[0], target_pos[1], target_pos[2])

            # Try multiple IK solvers for better success rate
            # Method 1: Levenberg-Marquardt with current position as seed
            sol = self.robot.ikine_LM(T_target, q0=self.robot.qz, mask=[1, 1, 1, 0, 0, 0])

            if sol.success:
                self.get_logger().info(f"IK solved with LM (current seed)", throttle_duration_sec=2.0)
                return sol.q

            # Method 2: Try with zero configuration as seed
            sol = self.robot.ikine_LM(T_target, q0=np.zeros(self.robot.n), mask=[1, 1, 1, 0, 0, 0])

            if sol.success:
                self.get_logger().info(f"IK solved with LM (zero seed)", throttle_duration_sec=2.0)
                return sol.q

            # Method 3: Try with random seed
            q_random = np.random.uniform(
                self.robot.qlim[0, :],
                self.robot.qlim[1, :],
                self.robot.n
            )
            sol = self.robot.ikine_LM(T_target, q0=q_random, mask=[1, 1, 1, 0, 0, 0])

            if sol.success:
                self.get_logger().info(f"IK solved with LM (random seed)", throttle_duration_sec=2.0)
                return sol.q

            # All methods failed
            self.get_logger().warn(f"IK failed for target {target_pos}. Target may be unreachable.", throttle_duration_sec=3.0)
            return None

        except Exception as e:
            self.get_logger().error(f"IK exception: {e}")
            return None

    def get_transform(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.source_frame,
                self.target_frame,
                rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f"TF not ready: {e}")
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



    def pub_joint_states(self, joint1, joint2, joint3):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [float(joint1), float(joint2), float(joint3)]
        self.joint_state_publisher.publish(msg)


    def timer_callback(self):
        # Correct time calculation
        time_diff = self.get_clock().now() - self.prev_time
        dt = time_diff.nanoseconds * 1.0e-9

        # Avoid issues with first iteration
        if dt <= 0 or dt > 1.0:
            dt = self.dt_loop

        # Get current end-effector pose
        T_current = self.get_transform()
        if T_current is None:
            return  # Exit if transform not available

        # Extract translation and rotation
        T_translation = T_current[0:3, 3]
        T_rot = T_current[0:3, 0:3]

        # Publish end-effector position
        self.endeff_publisher(T_translation)

        # Mode-specific control logic
        if self.mode == "TO":
            # TELEOPERATION MODE: Use velocity control from cmd_vel
            if self.teleop_frame == "tool":
                # Tool/end-effector frame: Transform cmd_vel to base frame
                p_dot = T_rot @ self.cmd_vel
            else:
                # World/base frame: Use cmd_vel directly
                p_dot = self.cmd_vel

        elif self.mode == "IPK":
            # INVERSE POSITION KINEMATICS MODE: Service-based only
            # Robot does NOT move automatically in IPK mode
            # Movement only happens when /inverse_kinematics service is called
            # The service callback (ipk_service_callback) handles movement

            # Just publish current joint states (no movement)
            self.pub_joint_states(self.robot.qz[0], self.robot.qz[1], self.robot.qz[2])
            self.prev_time = self.get_clock().now()
            return

        elif self.mode == "AM":
            # AUTO MODE: Service-based loop with 10-second timeout
            # State machine: Request target -> Move to target -> Wait 10s -> Repeat

            # If Auto Mode has failed, stop all operations
            if self.am_failed:
                self.get_logger().error(f"[AM CHECK] am_failed={self.am_failed} - Auto Mode is in FAILED state. Switch modes to recover.", throttle_duration_sec=5.0)
                self.pub_joint_states(self.robot.qz[0], self.robot.qz[1], self.robot.qz[2])
                self.prev_time = self.get_clock().now()
                return

            # If we're waiting for service response, skip
            if self.am_requesting_target:
                self.prev_time = self.get_clock().now()
                return

            # If no target yet, request one
            if self.target_position is None:
                self.get_logger().info("[AM] No target position - requesting initial target", throttle_duration_sec=2.0)
                self.request_random_target()
                self.prev_time = self.get_clock().now()
                return

            # If target not reached yet, move to target using IK
            if not self.am_target_reached:
                q_target = self.inverse_kinematic(self.target_position)

                if q_target is not None:
                    # Check if we're close enough to target
                    T_fk = self.robot.fkine(q_target)
                    target_pos_fk = T_fk.t  # Extract position from SE3

                    error = np.linalg.norm(target_pos_fk - self.target_position)

                    if error < 0.01:  # 1cm threshold
                        # Target reached!
                        self.robot.qz = q_target
                        self.am_target_reached = True
                        self.am_wait_start_time = self.get_clock().now()
                        self.get_logger().info("AM: Target reached! Starting 10-second wait...")
                    else:
                        # Still moving to target
                        self.robot.qz = q_target
                        self.get_logger().info(f"AM: Moving to target (error: {error:.4f}m)", throttle_duration_sec=1.0)
                else:
                    self.get_logger().warn("AM: IK failed for target", throttle_duration_sec=2.0)

                self.pub_joint_states(self.robot.qz[0], self.robot.qz[1], self.robot.qz[2])
                self.prev_time = self.get_clock().now()
                return

            # If target reached, wait 10 seconds
            if self.am_wait_start_time is not None:
                elapsed = (self.get_clock().now() - self.am_wait_start_time).nanoseconds * 1e-9

                if elapsed < self.am_wait_duration:
                    # Still waiting
                    self.get_logger().info(f"AM: Waiting... ({elapsed:.1f}/{self.am_wait_duration}s)", throttle_duration_sec=1.0)
                else:
                    # Wait complete, request new target
                    self.get_logger().info("[AM] Wait complete! Requesting new target...")
                    self.get_logger().info(f"[AM] Current state - am_failed={self.am_failed}, am_target_reached={self.am_target_reached}")
                    self.request_random_target()

            self.prev_time = self.get_clock().now()
            return

        else:
            # Unknown mode
            self.get_logger().error(f"Unknown mode: {self.mode}")
            p_dot = np.array([0.0, 0.0, 0.0])

        # Compute Jacobian and convert velocity to joint velocities
        J = self.robot.jacob0(self.robot.qz)
        J_pos = J[:3, :]

        # Singularity check
        condJ = np.linalg.cond(J_pos)
        if condJ > 1e3:
            # STOP robot (don't reset)
            warning_msg = String()
            warning_msg.data = f"SINGULARITY DETECTED! Condition number: {condJ:.2e}"
            self.singularity_pub.publish(warning_msg)
            self.get_logger().warn(warning_msg.data)

            # Stop all motion
            self.cmd_vel = np.array([0.0, 0.0, 0.0])
            # Keep current position (don't update joint states)
            self.prev_time = self.get_clock().now()
            return

        # Compute joint velocities
        dq = np.linalg.pinv(J_pos) @ p_dot
        self.robot.qz = self.robot.qz + dq * dt

        # Debug logging
        self.get_logger().info(f"Mode: {self.mode}, p_dot: {p_dot}, dq: {dq}", throttle_duration_sec=1.0)

        # Publish joint states
        self.pub_joint_states(self.robot.qz[0], self.robot.qz[1], self.robot.qz[2])

        self.prev_time = self.get_clock().now()




def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()