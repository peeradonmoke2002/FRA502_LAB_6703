import py_trees
from controller_interfaces.srv import RandomTarget
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as np

class HaveTarget(py_trees.behaviour.Behaviour):
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
        target_pos = self.blackboard.am_target_position
        if target_pos is None:
            return py_trees.common.Status.FAILURE

        if self.blackboard.am_target_start_time is None:
            self.blackboard.am_target_start_time = self.node.get_clock().now()

        T_current = self.robot.fkine(self.robot.qz)
        current_pos = T_current.t

        error = target_pos - current_pos
        error_norm = np.linalg.norm(error)

        elapsed = (self.node.get_clock().now() - self.blackboard.am_target_start_time).nanoseconds * 1e-9

        if error_norm < self.target_threshold:
            self.node.get_logger().info(f"[AM] Target reached in {elapsed:.2f}s")
            self.blackboard.am_last_target_reached = True
            self.blackboard.am_last_target_position = target_pos.copy()
            self.blackboard.am_target_position = None
            self.blackboard.am_target_start_time = None
            self.blackboard.am_singularity_start_time = None
            return py_trees.common.Status.SUCCESS

        if elapsed > self.travel_timeout:
            self.node.get_logger().error(f"[AM] Timeout after {elapsed:.2f}s")
            self.blackboard.am_last_target_reached = False
            self.blackboard.am_last_target_position = target_pos.copy()
            self.blackboard.am_target_position = None
            self.blackboard.am_target_start_time = None
            self.blackboard.am_singularity_start_time = None
            return py_trees.common.Status.FAILURE

        J = self.robot.jacob0(self.robot.qz)
        J_pos = J[:3, :]
        condJ = np.linalg.cond(J_pos)

        if condJ <= 1e3 and self.blackboard.am_singularity_start_time is not None:
            self.node.get_logger().info("[AM] Exited singularity region")
            self.blackboard.am_singularity_start_time = None

        if condJ > 1e3:
            if self.blackboard.am_singularity_start_time is None:
                self.blackboard.am_singularity_start_time = self.node.get_clock().now()
                self.node.get_logger().warn(f"[AM] Entered singularity (cond={condJ:.2e})")

            singularity_elapsed = (self.node.get_clock().now() - self.blackboard.am_singularity_start_time).nanoseconds * 1e-9

            if singularity_elapsed > self.singularity_timeout:
                self.node.get_logger().error(f"[AM] Stuck at singularity for {singularity_elapsed:.2f}s")

                self.blackboard.am_in_singularity = True

                self.blackboard.am_last_target_reached = False
                self.blackboard.am_last_target_position = target_pos.copy()

                self.blackboard.am_target_position = None
                self.blackboard.am_target_start_time = None
                self.blackboard.am_singularity_start_time = None

                return py_trees.common.Status.FAILURE

            self.node.get_logger().warn(
                f"[AM] Near singularity (cond={condJ:.2e}, {singularity_elapsed:.1f}s)",
                throttle_duration_sec=1.0
            )
            self.publish_joint_states()
            return py_trees.common.Status.RUNNING
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
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = ['joint_1', 'joint_2', 'joint_3']
        msg.position = [float(self.robot.qz[0]), float(self.robot.qz[1]), float(self.robot.qz[2])]
        self.publisher.publish(msg)

class IsNotSingularity(py_trees.behaviour.Behaviour):

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
            self.node.get_logger().error(
                "[AM] Behavior Tree blocked: robot is in singularity. Please restart the program."
            )
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.SUCCESS

class RequestTarget(py_trees.behaviour.Behaviour):
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
        self.requesting = False    
        self.future = None         
        self.last_request_success = None  

    def setup(self, **kwargs):
        if self.random_target_client is None:
            self.random_target_client = self.node.create_client(RandomTarget, 'random_target')

        if not hasattr(self.blackboard, 'am_last_target_position'):
            self.blackboard.am_last_target_position = None
        if not hasattr(self.blackboard, 'am_last_target_reached'):
            self.blackboard.am_last_target_reached = False

    def update(self):
        if self.blackboard.am_target_position is not None:
            self.requesting = False
            self.future = None
            self.last_request_success = None
            return py_trees.common.Status.SUCCESS
        
        if not self.random_target_client.service_is_ready():
            self.node.get_logger().warn("[AM] Random target service not available", throttle_duration_sec=5.0)
            return py_trees.common.Status.RUNNING

        if self.requesting:
            if self.future is not None and self.future.done():
                if self.last_request_success:
                    self.node.get_logger().info("[AM] Target request completed")
                    self.requesting = False
                    self.future = None
                    self.last_request_success = None
                    return py_trees.common.Status.SUCCESS
                else:
                    self.node.get_logger().error("[AM] Target request failed")
                    self.requesting = False
                    self.future = None
                    self.last_request_success = None
                    return py_trees.common.Status.FAILURE

            return py_trees.common.Status.RUNNING


        if self.blackboard.am_last_target_position is not None:
            reference_pos = self.blackboard.am_last_target_position
            target_reached = bool(self.blackboard.am_last_target_reached)
            status = "reached" if target_reached else "failed"
            self.node.get_logger().info(f"[AM] Requesting new target (previous: {status})")
        else:
            T_current = self.robot.fkine(self.robot.qz)
            reference_pos = T_current.t
            target_reached = False
            self.node.get_logger().info("[AM] Requesting initial target")

        request = RandomTarget.Request()
        request.request_new_target = True
        request.target_reached = target_reached
        request.current_position.x = float(reference_pos[0])
        request.current_position.y = float(reference_pos[1])
        request.current_position.z = float(reference_pos[2])

        self.future = self.random_target_client.call_async(request)
        self.future.add_done_callback(self.target_response_callback)

        self.requesting = True
        self.last_request_success = None

        return py_trees.common.Status.RUNNING

    def target_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
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

def create_am_mode_tree(node, robot):# no memory

    blackboard = py_trees.blackboard.Client(name="AM_Mode_Init")
    blackboard.register_key(key="am_target_position", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="am_target_start_time", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="am_singularity_start_time", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="am_last_target_reached", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="am_last_target_position", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="am_in_singularity", access=py_trees.common.Access.WRITE)
    blackboard.am_target_position = None
    blackboard.am_target_start_time = None
    blackboard.am_singularity_start_time = None
    blackboard.am_last_target_reached = None
    blackboard.am_last_target_position = None
    blackboard.am_in_singularity = False

    process_seq = py_trees.composites.Sequence(name="ProcessTarget", memory=False)  
    process_seq.add_children([
        HaveTarget(name="HaveTarget?"),
        IsNotSingularity(name="IsNotSingularity", node=node),
        MoveToTarget(name="MoveToTarget", node=node, robot=robot)
    ])


    request_target = RequestTarget(name="RequestTarget", node=node, robot=robot)

    root = py_trees.composites.Selector(name="AM_Root", memory=False)  
    root.add_children([
        process_seq,
        request_target
    ])

    return root
