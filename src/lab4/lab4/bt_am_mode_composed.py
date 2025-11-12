import py_trees
from controller_interfaces.srv import RandomTarget
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as np


class HaveTarget(py_trees.behaviour.Behaviour):
    """Check if we have an active target"""
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.__class__.__name__)
        self.blackboard.register_key(key="am_target_position", access=py_trees.common.Access.READ)

    def update(self):
        if self.blackboard.am_target_position is not None:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class MoveToTarget(py_trees.behaviour.Behaviour):


    def __init__(self, name, node, robot):
        super().__init__(name)
        self.node = node
        self.robot = robot
        self.publisher = None

        self.blackboard = self.attach_blackboard_client(name=self.__class__.__name__)
        self.blackboard.register_key(key="am_target_position", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="am_target_start_time", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="am_singularity_start_time", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="am_last_target_reached", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="am_last_target_position", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="am_in_singularity", access=py_trees.common.Access.WRITE)

        # Control parameters
        self.dt = 0.01
        self.kp = 0.5
        self.target_threshold = 0.01
        self.travel_timeout = 10.0
        self.singularity_timeout = 3.0

    def setup(self, **kwargs):
        if self.publisher is None:
            self.publisher = self.node.create_publisher(JointState, 'joint_states', 10)

    def update(self):
        """Main processing logic"""
        target_pos = self.blackboard.am_target_position
        if target_pos is None:
            return py_trees.common.Status.FAILURE

        # Initialize timer
        if self.blackboard.am_target_start_time is None:
            self.blackboard.am_target_start_time = self.node.get_clock().now()

        # Get current position
        T_current = self.robot.fkine(self.robot.qz)
        current_pos = T_current.t

        # Compute error
        error = target_pos - current_pos
        error_norm = np.linalg.norm(error)

        # Calculate elapsed time
        elapsed = (self.node.get_clock().now() - self.blackboard.am_target_start_time).nanoseconds * 1e-9

        # Check if target reached
        if error_norm < self.target_threshold:
            self.node.get_logger().info(f"[AM] Target reached in {elapsed:.2f}s")
            # Save info for RequestTarget to report
            self.blackboard.am_last_target_reached = True
            self.blackboard.am_last_target_position = target_pos.copy()
            # Clear current target so RequestTarget runs next tick
            self.blackboard.am_target_position = None
            self.blackboard.am_target_start_time = None
            self.blackboard.am_singularity_start_time = None
            return py_trees.common.Status.SUCCESS

        # Check global timeout
        if elapsed > self.travel_timeout:
            self.node.get_logger().error(f"[AM] Timeout after {elapsed:.2f}s")
            # Save info for RequestTarget to report
            self.blackboard.am_last_target_reached = False
            self.blackboard.am_last_target_position = target_pos.copy()
            # Clear target and return FAILED so RequestTarget runs
            self.blackboard.am_target_position = None
            self.blackboard.am_target_start_time = None
            self.blackboard.am_singularity_start_time = None
            return py_trees.common.Status.FAILURE

        # Compute Jacobian and check singularity
        J = self.robot.jacob0(self.robot.qz)
        J_pos = J[:3, :]
        condJ = np.linalg.cond(J_pos)

        if condJ <= 1e3 and self.blackboard.am_singularity_start_time is not None:
            self.node.get_logger().info("[AM] Exited singularity region")
            self.blackboard.am_singularity_start_time = None

        if condJ > 1e3:
            # At singularity - track time
            if self.blackboard.am_singularity_start_time is None:
                self.blackboard.am_singularity_start_time = self.node.get_clock().now()
                self.node.get_logger().warn(f"[AM] Entered singularity (cond={condJ:.2e})")

            singularity_elapsed = (self.node.get_clock().now() - self.blackboard.am_singularity_start_time).nanoseconds * 1e-9

            if singularity_elapsed > self.singularity_timeout:
                self.node.get_logger().error(f"[AM] Stuck at singularity for {singularity_elapsed:.2f}s")

                # mark singular state on blackboard
                self.blackboard.am_in_singularity = True

                # Save info for RequestTarget to report (optional)
                self.blackboard.am_last_target_reached = False
                self.blackboard.am_last_target_position = target_pos.copy()

                # Clear target so BT doesn’t try to move this one again
                self.blackboard.am_target_position = None
                self.blackboard.am_target_start_time = None
                self.blackboard.am_singularity_start_time = None

                return py_trees.common.Status.FAILURE

            # still in singular, but not timed out yet
            self.node.get_logger().warn(
                f"[AM] Near singularity (cond={condJ:.2e}, {singularity_elapsed:.1f}s)",
                throttle_duration_sec=1.0
            )
            self.publish_joint_states()
            return py_trees.common.Status.RUNNING

        # Move to target
        p_dot = self.kp * error
        dq = np.linalg.pinv(J_pos) @ p_dot
        self.robot.qz = self.robot.qz + dq * self.dt

        self.node.get_logger().info(
            f"[AM] Moving to target (error: {error_norm:.4f}m)",
            throttle_duration_sec=1.0
        )

        self.publish_joint_states()
        return py_trees.common.Status.RUNNING

    def publish_joint_states(self):
        """Publish current joint states"""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = ['joint_1', 'joint_2', 'joint_3']
        msg.position = [float(self.robot.qz[0]), float(self.robot.qz[1]), float(self.robot.qz[2])]
        self.publisher.publish(msg)

class IsNotSingularity(py_trees.behaviour.Behaviour):
    """
    Condition: SUCCESS if NOT in singularity.
    If in singularity, return FAILURE and log an error.
    """
    def __init__(self, name, node):
        super().__init__(name)
        self.node = node
        self.blackboard = self.attach_blackboard_client(name=self.__class__.__name__)
        self.blackboard.register_key(
            key="am_in_singularity",
            access=py_trees.common.Access.READ
        )

    def update(self):
        in_singularity = getattr(self.blackboard, "am_in_singularity", False)
        if in_singularity:
            # Here you can log once or with throttle; user sees it's stuck
            self.node.get_logger().error(
                "[AM] Behavior Tree blocked: robot is in singularity. Please restart the program."
            )
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.SUCCESS

class RequestTarget(py_trees.behaviour.Behaviour):
    """
    Request new target from service.
    Reads previous target result from blackboard (am_last_target_*) and reports it.
    """

    def __init__(self, name, node, robot):
        super().__init__(name)
        self.node = node
        self.robot = robot
        self.random_target_client = None

        # Blackboard
        self.blackboard = self.attach_blackboard_client(name=self.__class__.__name__)
        self.blackboard.register_key(key="am_target_position",        access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="am_target_start_time",      access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="am_last_target_reached",    access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="am_last_target_position",   access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="am_singularity_start_time", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="am_in_singularity",         access=py_trees.common.Access.READ)


        # Async request state
        self.requesting = False      # True while waiting for service reply
        self.future = None           # rclpy future
        self.last_request_success = None  # True / False / None

    def setup(self, **kwargs):
        if self.random_target_client is None:
            self.random_target_client = self.node.create_client(RandomTarget, 'random_target')

        if not hasattr(self.blackboard, 'am_last_target_position'):
            self.blackboard.am_last_target_position = None
        if not hasattr(self.blackboard, 'am_last_target_reached'):
            self.blackboard.am_last_target_reached = False

    def update(self):
        """Request new target (non-blocking)."""
        # in_singularity = getattr(self.blackboard, "am_in_singularity", False)
        # if in_singularity:
        #     self.node.get_logger().error(
        #         "[AM] Cannot request new target: robot in singularity. Please restart the program."
        #     )
        #     # also clean async state
        #     self.requesting = False
        #     self.future = None
        #     self.last_request_success = None
        #     return py_trees.common.Status.FAILURE
        
        # If we already have an active target, nothing to do
        if self.blackboard.am_target_position is not None:
            # ensure async state is clean
            self.requesting = False
            self.future = None
            self.last_request_success = None
            return py_trees.common.Status.SUCCESS

        # Service not available
        if not self.random_target_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn("[AM] Random target service not available")
            return py_trees.common.Status.FAILURE

        # We already sent a request and are waiting for the reply
        if self.requesting:
            # Future finished?
            if self.future is not None and self.future.done():
                # target_response_callback has set last_request_success
                if self.last_request_success:
                    # Got a valid target (blackboard.am_target_position is now set)
                    self.node.get_logger().info("[AM] Target request completed")
                    self.requesting = False
                    self.future = None
                    self.last_request_success = None
                    return py_trees.common.Status.SUCCESS
                else:
                    # Service replied but failed
                    self.node.get_logger().error("[AM] Target request failed")
                    self.requesting = False
                    self.future = None
                    self.last_request_success = None
                    return py_trees.common.Status.FAILURE

            # still waiting → RUNNING
            return py_trees.common.Status.RUNNING

        # No active request and no active target → send a new request

        # Choose reference position & previous status
        if self.blackboard.am_last_target_position is not None:
            reference_pos = self.blackboard.am_last_target_position
            target_reached = bool(self.blackboard.am_last_target_reached)
            status = "reached" if target_reached else "failed"
            self.node.get_logger().info(f"[AM] Requesting new target (previous: {status})")
        else:
            # First ever request → use current pose as reference
            T_current = self.robot.fkine(self.robot.qz)
            reference_pos = T_current.t
            target_reached = False
            self.node.get_logger().info("[AM] Requesting initial target")

        # Request
        request = RandomTarget.Request()
        request.request_new_target = True
        request.target_reached = target_reached
        request.current_position.x = float(reference_pos[0])
        request.current_position.y = float(reference_pos[1])
        request.current_position.z = float(reference_pos[2])

        # Send async request
        self.future = self.random_target_client.call_async(request)
        self.future.add_done_callback(self.target_response_callback)

        self.requesting = True
        self.last_request_success = None

        # We are now waiting for the response
        return py_trees.common.Status.RUNNING

    def target_response_callback(self, future):
        """Handle service response (runs in executor thread)."""
        try:
            response = future.result()
            if response.success:
                # Write new target to blackboard
                self.blackboard.am_target_position = np.array([
                    response.target_position.x,
                    response.target_position.y,
                    response.target_position.z
                ])
                self.blackboard.am_target_start_time = self.node.get_clock().now()
                self.node.get_logger().info(
                    f"[AM] New target: [{response.target_position.x:.3f}, "
                    f"{response.target_position.y:.3f}, {response.target_position.z:.3f}]"
                )
                self.last_request_success = True
            else:
                self.node.get_logger().error(f"[AM] Service failed: {response.message}")
                self.last_request_success = False
        except Exception as e:
            self.node.get_logger().error(f"[AM] Service exception: {e}")
            self.last_request_success = False

def create_am_mode_tree(node, robot):

    # Initialize blackboard
    blackboard = py_trees.blackboard.Client(name="AM_Mode_Init")
    blackboard.register_key(key="am_target_position", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="am_target_start_time", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="am_singularity_start_time", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="am_last_target_reached", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="am_last_target_position", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="am_in_singularity", access=py_trees.common.Access.WRITE)
    # Initialize with default values
    blackboard.am_target_position = None
    blackboard.am_target_start_time = None
    blackboard.am_singularity_start_time = None
    blackboard.am_last_target_reached = None
    blackboard.am_last_target_position = None
    blackboard.am_in_singularity = False

    process_seq = py_trees.composites.Sequence(name="ProcessTarget", memory=False)  # memory=False (default)
    process_seq.add_children([
        HaveTarget(name="HaveTarget?"),
        IsNotSingularity(name="IsNotSingularity", node=node),
        MoveToTarget(name="MoveToTarget", node=node, robot=robot)
    ])


    request_target = RequestTarget(name="RequestTarget", node=node, robot=robot)

    root = py_trees.composites.Selector(name="AM_Root", memory=False)  # no memory
    root.add_children([
        process_seq,
        request_target
    ])

    return root