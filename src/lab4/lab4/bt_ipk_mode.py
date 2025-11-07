import py_trees
import py_trees_ros
from controller_interfaces.srv import SetMode, InverseKinematics, RandomTarget
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as np
from spatialmath import SE3

# IPK Mode Behaviors

class IPKServiceBehavior(py_trees.behaviour.Behaviour):
    """
    Behavior that creates the IPK service and handles requests.
    Stores incoming requests on blackboard for processing.
    """

    def __init__(self, name, node, robot):
        super(IPKServiceBehavior, self).__init__(name)
        self.node = node
        self.robot = robot
        self.service = None
        self.blackboard = self.attach_blackboard_client(name=self.__class__.__name__)
        self.blackboard.register_key(key="pose_request", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="ipk_response", access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        """Create IPK service only once"""
        if self.service is None:
            self.service = self.node.create_service(
                InverseKinematics,
                'inverse_kinematics',
                self.ipk_service_callback
            )
            # Initialize blackboard
            self.blackboard.pose_request = None
            self.blackboard.ipk_response = None
            self.node.get_logger().info("IPK service created")

    def ipk_service_callback(self, request, response):
        """Store request on blackboard for BT processing"""
        target_pos = np.array([
            request.target_position.x,
            request.target_position.y,
            request.target_position.z
        ])

        self.node.get_logger().info(f"IPK service called for target: {target_pos}")

        # Store request on blackboard
        self.blackboard.pose_request = target_pos

        # Wait briefly for BT to process (synchronous service)
        # In a real implementation, you might want async processing
        # For now, solve IK directly in callback
        q_solution = self.solve_ik(target_pos)

        if q_solution is not None:
            # Update robot position
            self.robot.qz = q_solution

            response.success = True
            response.solution = q_solution.tolist()
            response.message = f"IK solved successfully"
            self.node.get_logger().info(response.message)
        else:
            response.success = False
            response.solution = []
            response.message = f"IK failed for target {target_pos}"
            self.node.get_logger().warn(response.message)

        # Clear request
        self.blackboard.pose_request = None

        return response

    def solve_ik(self, target_pos):
        """Solve inverse kinematics for target position"""
        try:
            T_target = SE3(target_pos[0], target_pos[1], target_pos[2])

            # Method 1: LM with current position as seed
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
            self.node.get_logger().error(f"IK exception: {e}")
            return None

    def update(self):
        """Service is running, always return SUCCESS"""
        return py_trees.common.Status.SUCCESS


class HavePoseRequest(py_trees.behaviour.Behaviour):
    """Check if a pose request is available on the blackboard."""

    def __init__(self, name):
        super(HavePoseRequest, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.__class__.__name__)
        self.blackboard.register_key(
            key="pose_request",
            access=py_trees.common.Access.READ
        )

    def update(self):
        if self.blackboard.pose_request is not None:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class SolveIPK(py_trees.behaviour.Behaviour):
    """Solve inverse kinematics for the requested pose."""

    def __init__(self, name, robot):
        super(SolveIPK, self).__init__(name)
        self.robot = robot
        self.blackboard = self.attach_blackboard_client(name=self.__class__.__name__)
        self.blackboard.register_key(key="pose_request", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="ik_solution", access=py_trees.common.Access.WRITE)

    def update(self):
        target_pos = self.blackboard.pose_request

        if target_pos is None:
            return py_trees.common.Status.FAILURE

        try:
            T_target = SE3(target_pos[0], target_pos[1], target_pos[2])

            # Try multiple IK methods
            sol = self.robot.ikine_LM(T_target, q0=self.robot.qz, mask=[1, 1, 1, 0, 0, 0])

            if sol.success:
                self.blackboard.ik_solution = sol.q
                return py_trees.common.Status.SUCCESS

            # Try with zero seed
            sol = self.robot.ikine_LM(T_target, q0=np.zeros(self.robot.n), mask=[1, 1, 1, 0, 0, 0])

            if sol.success:
                self.blackboard.ik_solution = sol.q
                return py_trees.common.Status.SUCCESS

            return py_trees.common.Status.FAILURE

        except Exception as e:
            return py_trees.common.Status.FAILURE


class UpdateRobotPosition(py_trees.behaviour.Behaviour):
    """Update robot joint positions with IK solution."""

    def __init__(self, name, robot):
        super(UpdateRobotPosition, self).__init__(name)
        self.robot = robot
        self.blackboard = self.attach_blackboard_client(name=self.__class__.__name__)
        self.blackboard.register_key(key="ik_solution", access=py_trees.common.Access.READ)

    def update(self):
        ik_solution = self.blackboard.ik_solution

        if ik_solution is not None:
            self.robot.qz = ik_solution
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.FAILURE


class PublishJointStates(py_trees.behaviour.Behaviour):
    """Publish current joint states."""

    def __init__(self, name, node, robot):
        super(PublishJointStates, self).__init__(name)
        self.node = node
        self.robot = robot
        self.publisher = None

    def setup(self, **kwargs):
        """Create publisher once"""
        if self.publisher is None:
            self.publisher = self.node.create_publisher(JointState, 'joint_states', 10)

    def update(self):
        """Publish current joint states"""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = ['joint_1', 'joint_2', 'joint_3']
        msg.position = [float(self.robot.qz[0]), float(self.robot.qz[1]), float(self.robot.qz[2])]
        self.publisher.publish(msg)
        return py_trees.common.Status.SUCCESS


class ClearPoseRequest(py_trees.behaviour.Behaviour):
    """Clear the pose request from blackboard after processing."""

    def __init__(self, name):
        super(ClearPoseRequest, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.__class__.__name__)
        self.blackboard.register_key(key="pose_request", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="ik_solution", access=py_trees.common.Access.WRITE)

    def update(self):
        self.blackboard.pose_request = None
        self.blackboard.ik_solution = None
        return py_trees.common.Status.SUCCESS


class IPKMode(py_trees.behaviour.Behaviour):
    """
    IPK mode behavior that moves robot smoothly towards target using velocity control.
    """

    def __init__(self, name, node, robot):
        super(IPKMode, self).__init__(name)
        self.node = node
        self.robot = robot
        self.publisher = None
        self.blackboard = self.attach_blackboard_client(name=self.__class__.__name__)
        self.blackboard.register_key(key="target_position", access=py_trees.common.Access.READ)
        self.dt = 0.01  # 100Hz update rate
        self.kp = 0.5  # Velocity gain for smooth movement

    def setup(self, **kwargs):
        if self.publisher is None:
            self.publisher = self.node.create_publisher(JointState, 'joint_states', 10)

    def update(self):
        """Move smoothly towards target using velocity control"""
        target_pos = self.blackboard.target_position

        if target_pos is None:
            # No target - just publish current position
            self.publish_joint_states()
            return py_trees.common.Status.RUNNING

        # Get current end-effector position from forward kinematics
        T_current = self.robot.fkine(self.robot.qz)
        current_pos = T_current.t  # [x, y, z]

        # Compute position error
        error = target_pos - current_pos
        error_norm = np.linalg.norm(error)

        # Check if target reached (within 1cm)
        if error_norm < 0.01:
            self.node.get_logger().info(f"IPK: Target reached! (error: {error_norm:.4f}m)", throttle_duration_sec=2.0)
            self.publish_joint_states()
            return py_trees.common.Status.RUNNING

        # Compute desired velocity towards target
        p_dot = self.kp * error  # Proportional control

        # Compute Jacobian
        J = self.robot.jacob0(self.robot.qz)
        J_pos = J[:3, :]  # Position part only

        # Singularity check
        condJ = np.linalg.cond(J_pos)
        if condJ > 1e3:
            self.node.get_logger().warn(f"IPK: Near singularity (cond={condJ:.2e})", throttle_duration_sec=2.0)
            self.publish_joint_states()
            return py_trees.common.Status.RUNNING

        # Compute joint velocities using pseudo-inverse
        dq = np.linalg.pinv(J_pos) @ p_dot

        # Update robot position
        self.robot.qz = self.robot.qz + dq * self.dt

        # Log progress
        self.node.get_logger().info(
            f"IPK: Moving to target (error: {error_norm:.4f}m)",
            throttle_duration_sec=1.0
        )

        # Publish updated joint states
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
