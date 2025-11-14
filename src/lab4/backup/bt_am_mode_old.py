import py_trees
from controller_interfaces.srv import RandomTarget
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as np

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
        self.blackboard.register_key(key="am_target_start_time", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="am_requesting_target", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="am_failed", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="am_last_status", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="am_singularity_start_time", access=py_trees.common.Access.WRITE)

        # Control parameters
        self.dt = 0.01  # 100Hz update rate
        self.kp = 0.5  # Velocity gain for smooth movement
        self.target_threshold = 0.01  # 1cm threshold
        self.travel_timeout = 10.0  # seconds
        self.singularity_timeout = 3.0  # seconds - max time stuck at singularity before giving up

    def setup(self, **kwargs):
        """Create publisher and service client once"""
        if self.publisher is None:
            self.publisher = self.node.create_publisher(JointState, 'joint_states', 10)

        if self.random_target_client is None:
            self.random_target_client = self.node.create_client(RandomTarget, 'random_target')

        # Initialize blackboard state (always set to ensure clean state)
        self.blackboard.am_target_position = None
        self.blackboard.am_target_start_time = None
        self.blackboard.am_requesting_target = False
        self.blackboard.am_failed = False
        self.blackboard.am_last_status = "IDLE"
        self.blackboard.am_singularity_start_time = None

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

        # Active target: move until success/timeout
        self.move_to_target()
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

        # Initialize target timer if needed
        if self.blackboard.am_target_start_time is None:
            self.blackboard.am_target_start_time = self.node.get_clock().now()

        elapsed = (self.node.get_clock().now() - self.blackboard.am_target_start_time).nanoseconds * 1e-9

        # Check if target reached (within 1cm)
        if error_norm < self.target_threshold:
            self.handle_target_completion(success=True, travel_time=elapsed)
            return

        # Travel time exceeded
        if elapsed > self.travel_timeout:
            self.handle_target_completion(success=False, travel_time=elapsed, error_norm=error_norm)
            return

        # Compute desired velocity towards target
        p_dot = self.kp * error  # Proportional control

        # Compute Jacobian
        J = self.robot.jacob0(self.robot.qz)
        J_pos = J[:3, :]  # Position part only

        # Singularity check
        condJ = np.linalg.cond(J_pos)
        if condJ > 1e3:
            # Start or continue singularity timer
            if self.blackboard.am_singularity_start_time is None:
                self.blackboard.am_singularity_start_time = self.node.get_clock().now()
                self.node.get_logger().warn(
                    f"[AM] Entered singularity region (cond={condJ:.2e})",
                    throttle_duration_sec=2.0
                )

            # Check how long we've been stuck at singularity
            singularity_elapsed = (self.node.get_clock().now() - self.blackboard.am_singularity_start_time).nanoseconds * 1e-9

            if singularity_elapsed > self.singularity_timeout:
                # Stuck at singularity for too long - abandon this target
                self.node.get_logger().error(
                    f"[AM] Stuck at singularity for {singularity_elapsed:.2f}s - abandoning target"
                )
                self.handle_target_completion(success=False, travel_time=elapsed, error_norm=error_norm, reason="SINGULARITY")
                return
            else:
                self.node.get_logger().warn(
                    f"[AM] Near singularity (cond={condJ:.2e}, stuck for {singularity_elapsed:.1f}s)",
                    throttle_duration_sec=1.0
                )

            self.publish_joint_states()
            return
        else:
            # Not at singularity - reset singularity timer
            if self.blackboard.am_singularity_start_time is not None:
                self.node.get_logger().info("[AM] Exited singularity region")
                self.blackboard.am_singularity_start_time = None

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

    def handle_target_completion(self, success, travel_time, error_norm=0.0, reason="TIMEOUT"):
        """Handle success or timeout when attempting to reach a target"""
        if success:
            status_text = "SUCCESS"
        else:
            status_text = reason  # "TIMEOUT" or "SINGULARITY"

        target = self.blackboard.am_target_position
        if target is None:
            T_current = self.robot.fkine(self.robot.qz)
            target = T_current.t

        if success:
            self.node.get_logger().info(
                f"[AM] Target reached in {travel_time:.2f}s (<= {self.travel_timeout}s)",
                throttle_duration_sec=1.0
            )
        else:
            if reason == "SINGULARITY":
                self.node.get_logger().error(
                    f"[AM] Failed to reach target - path blocked by singularity "
                    f"(elapsed {travel_time:.2f}s, error {error_norm:.3f}m)"
                )
            else:
                self.node.get_logger().error(
                    f"[AM] Failed to reach target within {self.travel_timeout}s "
                    f"(elapsed {travel_time:.2f}s, error {error_norm:.3f}m)"
                )

        self.blackboard.am_last_status = status_text
        self.blackboard.am_target_position = None
        self.blackboard.am_target_start_time = None
        self.blackboard.am_singularity_start_time = None  # Reset singularity timer
        self.publish_joint_states()

        # Request the next target and report outcome
        self.request_random_target(
            target_reached=success,
            reference_position=target
        )

    def request_random_target(self, target_reached=False, reference_position=None):
        """Request a new random target from random_pos node while reporting current status"""
        if not self.random_target_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn("[AM] Random target service not available")
            return

        request = RandomTarget.Request()
        request.request_new_target = True
        request.target_reached = target_reached

        if reference_position is None:
            T_current = self.robot.fkine(self.robot.qz)
            reference_position = T_current.t

        request.current_position.x = float(reference_position[0])
        request.current_position.y = float(reference_position[1])
        request.current_position.z = float(reference_position[2])

        self.node.get_logger().info(
            "[AM SERVICE CALL] Requesting random target "
            f"(reported_status={self.blackboard.am_last_status})"
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
                self.blackboard.am_target_start_time = self.node.get_clock().now()
                self.blackboard.am_last_status = "TRACKING"
                self.node.get_logger().info(
                    f"[AM] New random target received: {self.blackboard.am_target_position}"
                )
            else:
                # Service failed - stop Auto Mode operation completely
                self.node.get_logger().error(f"[AM FAILURE] Random target service failed: {response.message}")
                self.node.get_logger().error("[AM FAILURE] Setting am_failed=True to stop Auto Mode")
                self.node.get_logger().error("[AM FAILURE] Auto Mode STOPPED. Please switch modes.")
                self.blackboard.am_last_status = "FAILED_SERVICE"

                # Set failure flag to completely stop Auto Mode
                self.blackboard.am_failed = True

                # Keep old target_position if it exists, or use current position to stop movement
                if self.blackboard.am_target_position is None:
                    T_current = self.robot.fkine(self.robot.qz)
                    self.blackboard.am_target_position = T_current.t

        except Exception as e:
            self.node.get_logger().error(f"[AM] Service call exception: {e}")
            self.node.get_logger().error("[AM] Auto Mode STOPPED due to exception. Please switch modes.")
            self.blackboard.am_last_status = "FAILED_SERVICE"

            # Set failure flag to completely stop Auto Mode
            self.blackboard.am_failed = True

            # On exception, stop Auto Mode completely
            if self.blackboard.am_target_position is None:
                T_current = self.robot.fkine(self.robot.qz)
                self.blackboard.am_target_position = T_current.t

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
        self.blackboard.am_target_start_time = None
        self.blackboard.am_requesting_target = False
        self.blackboard.am_failed = False
        self.blackboard.am_last_status = "IDLE"
        self.blackboard.am_singularity_start_time = None
        return super().terminate(new_status)
