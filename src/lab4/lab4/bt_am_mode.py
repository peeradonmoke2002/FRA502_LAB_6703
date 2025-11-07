import py_trees
import py_trees_ros
from controller_interfaces.srv import SetMode, InverseKinematics, RandomTarget
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as np
from spatialmath import SE3

# AM Mode Behaviors

class AMMode(py_trees.behaviour.Behaviour):

    def __init__(self, name, node, robot):
        super(AMMode, self).__init__(name)
        self.node = node
        self.robot = robot
        self.publisher = None
        self.random_target_client = None

        # Blackboard for shared state
        self.blackboard = self.attach_blackboard_client(name=self.__class__.__name__)
        self.blackboard.register_key(key="am_target_position", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="am_target_reached", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="am_wait_start_time", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="am_requesting_target", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="am_failed", access=py_trees.common.Access.WRITE)

        # Control parameters
        self.dt = 0.01  # 100Hz update rate
        self.kp = 0.5  # Velocity gain for smooth movement
        self.wait_duration = 10.0  # 10 seconds wait at target
        self.target_threshold = 0.01  # 1cm threshold

    def setup(self, **kwargs):
        """Create publisher and service client once"""
        if self.publisher is None:
            self.publisher = self.node.create_publisher(JointState, 'joint_states', 10)

        if self.random_target_client is None:
            self.random_target_client = self.node.create_client(RandomTarget, 'random_target')

        # Initialize blackboard state (always set to ensure clean state)
        self.blackboard.am_target_position = None
        self.blackboard.am_target_reached = False
        self.blackboard.am_wait_start_time = None
        self.blackboard.am_requesting_target = False
        self.blackboard.am_failed = False

    def update(self):
        """Auto Mode state machine"""

        # If Auto Mode has failed, stop all operations
        if self.blackboard.am_failed:
            self.node.get_logger().error(
                "[AM] Auto Mode is in FAILED state. Switch modes to recover.",
                throttle_duration_sec=5.0
            )
            self.publish_joint_states()
            return py_trees.common.Status.RUNNING

        # If we're waiting for service response, skip
        if self.blackboard.am_requesting_target:
            return py_trees.common.Status.RUNNING

        # If no target yet, request one
        if self.blackboard.am_target_position is None:
            self.node.get_logger().info(
                "[AM] No target position - requesting initial target",
                throttle_duration_sec=2.0
            )
            self.request_random_target()
            return py_trees.common.Status.RUNNING

        # If target not reached yet, move to target
        if not self.blackboard.am_target_reached:
            self.move_to_target()
            return py_trees.common.Status.RUNNING

        # If target reached, wait 10 seconds
        if self.blackboard.am_wait_start_time is not None:
            elapsed = (self.node.get_clock().now() - self.blackboard.am_wait_start_time).nanoseconds * 1e-9

            if elapsed < self.wait_duration:
                # Still waiting
                self.node.get_logger().info(
                    f"[AM] Waiting... ({elapsed:.1f}/{self.wait_duration}s)",
                    throttle_duration_sec=1.0
                )
                self.publish_joint_states()
            else:
                # Wait complete, request new target
                self.node.get_logger().info("[AM] Wait complete! Requesting new target...")
                self.request_random_target()

        return py_trees.common.Status.RUNNING

    def move_to_target(self):
        """Move smoothly towards target using velocity control"""
        target_pos = self.blackboard.am_target_position

        # Get current end-effector position from forward kinematics
        T_current = self.robot.fkine(self.robot.qz)
        current_pos = T_current.t  # [x, y, z]

        # Compute position error
        error = target_pos - current_pos
        error_norm = np.linalg.norm(error)

        # Check if target reached (within 1cm)
        if error_norm < self.target_threshold:
            self.blackboard.am_target_reached = True
            self.blackboard.am_wait_start_time = self.node.get_clock().now()
            self.node.get_logger().info("[AM] Target reached! Starting 10-second wait...")
            self.publish_joint_states()
            return

        # Compute desired velocity towards target
        p_dot = self.kp * error  # Proportional control

        # Compute Jacobian
        J = self.robot.jacob0(self.robot.qz)
        J_pos = J[:3, :]  # Position part only

        # Singularity check
        condJ = np.linalg.cond(J_pos)
        if condJ > 1e3:
            self.node.get_logger().warn(
                f"[AM] Near singularity (cond={condJ:.2e})",
                throttle_duration_sec=2.0
            )
            self.publish_joint_states()
            return

        # Compute joint velocities using pseudo-inverse
        dq = np.linalg.pinv(J_pos) @ p_dot

        # Update robot position
        self.robot.qz = self.robot.qz + dq * self.dt

        # Log progress
        self.node.get_logger().info(
            f"[AM] Moving to target (error: {error_norm:.4f}m)",
            throttle_duration_sec=1.0
        )

        # Publish updated joint states
        self.publish_joint_states()

    def request_random_target(self):
        """Request a new random target from random_pos node"""
        if not self.random_target_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn("[AM] Random target service not available")
            return

        request = RandomTarget.Request()
        request.request_new_target = True

        self.node.get_logger().info(
            f"[AM SERVICE CALL] Calling /random_target with request_new_target={request.request_new_target}"
        )

        future = self.random_target_client.call_async(request)
        future.add_done_callback(self.random_target_response_callback)
        self.blackboard.am_requesting_target = True

    def random_target_response_callback(self, future):
        """Callback for random target service response"""
        try:
            response = future.result()
            self.node.get_logger().info(
                f"[AM SERVICE RESPONSE] Received response: success={response.success}, message='{response.message}'"
            )
            self.node.get_logger().info(
                f"[AM SERVICE RESPONSE] Target position: ({response.target_position.x:.3f}, "
                f"{response.target_position.y:.3f}, {response.target_position.z:.3f})"
            )

            if response.success:
                self.blackboard.am_target_position = np.array([
                    response.target_position.x,
                    response.target_position.y,
                    response.target_position.z
                ])
                self.blackboard.am_target_reached = False
                self.blackboard.am_wait_start_time = None
                self.node.get_logger().info(f"[AM] New random target received: {self.blackboard.am_target_position}")
            else:
                # Service failed - stop Auto Mode operation completely
                self.node.get_logger().error(f"[AM FAILURE] Random target service failed: {response.message}")
                self.node.get_logger().error("[AM FAILURE] Setting am_failed=True to stop Auto Mode")
                self.node.get_logger().error("[AM FAILURE] Auto Mode STOPPED. Please switch modes.")

                # Set failure flag to completely stop Auto Mode
                self.blackboard.am_failed = True

                # Keep old target_position if it exists, or use current position to stop movement
                if self.blackboard.am_target_position is None:
                    T_current = self.robot.fkine(self.robot.qz)
                    self.blackboard.am_target_position = T_current.t

                # Mark as reached to fully stop Auto Mode
                self.blackboard.am_target_reached = True

        except Exception as e:
            self.node.get_logger().error(f"[AM] Service call exception: {e}")
            self.node.get_logger().error("[AM] Auto Mode STOPPED due to exception. Please switch modes.")

            # Set failure flag to completely stop Auto Mode
            self.blackboard.am_failed = True

            # On exception, stop Auto Mode completely
            if self.blackboard.am_target_position is None:
                T_current = self.robot.fkine(self.robot.qz)
                self.blackboard.am_target_position = T_current.t

            self.blackboard.am_target_reached = True

        finally:
            self.blackboard.am_requesting_target = False

    def publish_joint_states(self):
        """Publish current joint states"""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = ['joint_1', 'joint_2', 'joint_3']
        msg.position = [float(self.robot.qz[0]), float(self.robot.qz[1]), float(self.robot.qz[2])]
        self.publisher.publish(msg)

    def terminate(self, new_status):
        """Reset AM state when behavior is terminated (mode switch)"""
        self.node.get_logger().info("[AM] Terminating - resetting state")
        self.blackboard.am_target_position = None
        self.blackboard.am_target_reached = False
        self.blackboard.am_wait_start_time = None
        self.blackboard.am_requesting_target = False
        self.blackboard.am_failed = False
        return super().terminate(new_status)
