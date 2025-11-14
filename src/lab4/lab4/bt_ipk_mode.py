import py_trees
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as np


class HaveTarget(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.__class__.__name__)
        self.blackboard.register_key(key="ipk_target_reachable", access=py_trees.common.Access.READ)

    def update(self):
        service_response = getattr(self.blackboard, "ipk_target_reachable", None)
        if service_response is not None:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class IsNotSingularity(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super().__init__(name)
        self.node = node
        self.blackboard = self.attach_blackboard_client(name=self.__class__.__name__)
        self.blackboard.register_key(key="ipk_target_reachable", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="ipk_in_singularity", access=py_trees.common.Access.READ)

    def update(self):
        target_reachable = getattr(self.blackboard, "ipk_target_reachable", None)
        if target_reachable is False:
            self.node.get_logger().error(
                "[IPK] Target unreachable - IK solution not found. Please change target.",
                throttle_duration_sec=5.0
            )
            return py_trees.common.Status.FAILURE

        robot_stuck = getattr(self.blackboard, "ipk_in_singularity", False)
        if robot_stuck:
            self.node.get_logger().error(
                "[IPK] Robot stuck in singularity. Please change target.",
                throttle_duration_sec=5.0
            )
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.SUCCESS

class MoveToTarget(py_trees.behaviour.Behaviour):
    def __init__(self, name, node, robot):
        super().__init__(name)
        self.node = node
        self.robot = robot
        self.publisher = None

        self.blackboard = self.attach_blackboard_client(name=self.__class__.__name__)
        self.blackboard.register_key(key="target_position", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="ipk_singularity_start_time", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="ipk_in_singularity", access=py_trees.common.Access.WRITE)

        self.dt = 0.01
        self.kp = 0.5
        self.target_threshold = 0.01 
        self.singularity_timeout = 3.0 

    def setup(self, **kwargs):
        if self.publisher is None:
            self.publisher = self.node.create_publisher(JointState, 'joint_states', 10)

    def update(self):
        target_pos = self.blackboard.target_position
        if target_pos is None:
            return py_trees.common.Status.FAILURE

        T_current = self.robot.fkine(self.robot.qz)
        current_pos = T_current.t

        error = target_pos - current_pos
        error_norm = np.linalg.norm(error)

        if error_norm < self.target_threshold:
            self.node.get_logger().info(
                f"[IPK] Target reached (error: {error_norm:.4f}m)"
            )
            self.publish_joint_states()
            return py_trees.common.Status.SUCCESS

        J = self.robot.jacob0(self.robot.qz)
        J_pos = J[:3, :]
        condJ = np.linalg.cond(J_pos)

        if condJ <= 1e3 and self.blackboard.ipk_singularity_start_time is not None:
            self.node.get_logger().info("[IPK] Exited singularity region")
            self.blackboard.ipk_singularity_start_time = None

        if condJ > 1e3:
            if self.blackboard.ipk_singularity_start_time is None:
                self.blackboard.ipk_singularity_start_time = self.node.get_clock().now()
                self.node.get_logger().warn(f"[IPK] Entered singularity (cond={condJ:.2e})")

            singularity_elapsed = (self.node.get_clock().now() - self.blackboard.ipk_singularity_start_time).nanoseconds * 1e-9

            if singularity_elapsed > self.singularity_timeout:
                self.node.get_logger().error(f"[IPK] Stuck at singularity for {singularity_elapsed:.2f}s")
                self.blackboard.ipk_in_singularity = True
                self.publish_joint_states()
                return py_trees.common.Status.RUNNING

            self.node.get_logger().warn(
                f"[IPK] Near singularity (cond={condJ:.2e}, {singularity_elapsed:.1f}s)",
                throttle_duration_sec=1.0
            )
            self.publish_joint_states()
            return py_trees.common.Status.RUNNING

        p_dot = self.kp * error
        dq = np.linalg.pinv(J_pos) @ p_dot
        self.robot.qz = self.robot.qz + dq * self.dt

        self.node.get_logger().info(
            f"[IPK] Moving to target (error: {error_norm:.4f}m)",
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

class WaitForTarget(py_trees.behaviour.Behaviour):
    def __init__(self, name, node, robot):
        super().__init__(name)
        self.node = node
        self.robot = robot
        self.publisher = None

    def setup(self, **kwargs):
        if self.publisher is None:
            self.publisher = self.node.create_publisher(JointState, 'joint_states', 10)

    def update(self):
        self.node.get_logger().info(
            "[IPK] Waiting for target from /target topic",
            throttle_duration_sec=3.0
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

def create_ipk_mode_tree(node, robot):

    blackboard = py_trees.blackboard.Client(name="IPK_Mode_Init")
    blackboard.register_key(key="ipk_singularity_start_time", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="ipk_in_singularity", access=py_trees.common.Access.WRITE)

    blackboard.ipk_singularity_start_time = None
    blackboard.ipk_in_singularity = False

    root = py_trees.composites.Selector(name="IPK_Root", memory=False)

    process_seq = py_trees.composites.Sequence(name="ProcessTarget", memory=False)
    process_seq.add_children([
        HaveTarget(name="HaveTarget?"),
        IsNotSingularity(name="IsNotSingularity?", node=node),
        MoveToTarget(name="MoveToTarget", node=node, robot=robot)
    ])

    wait_target = WaitForTarget(name="WaitForTarget", node=node, robot=robot)

    root.add_children([
        process_seq,
        wait_target
    ])

    return root
