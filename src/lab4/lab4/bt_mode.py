import py_trees
from controller_interfaces.srv import SetMode

class SetModeService(py_trees.behaviour.Behaviour):

    def __init__(self, name, node):
        super(SetModeService, self).__init__(name)
        self.node = node
        self.mode_service = None
        self.mode = "TO"
        self.blackboard = self.attach_blackboard_client(name=self.__class__.__name__)
        self.blackboard.register_key(
            key="mode",
            access=py_trees.common.Access.WRITE
        )
    def setup(self, **kwargs):
        if self.mode_service is None:
            self.mode_service = self.node.create_service(
                SetMode, 'set_mode', self.set_mode_callback
            )
            self.blackboard.mode = self.mode
            self.node.get_logger().info("SetMode service created and blackboard initialized")

    def set_mode_callback(self, request, response):
        requested_mode = request.mode.upper()
        valid_modes = ["IPK", "TO", "AM"]

        if requested_mode in valid_modes:
            # Update blackboard mode (shared across all behaviors)
            self.blackboard.mode = requested_mode
            response.success = True
            response.message = f"Mode switched to {self.blackboard.mode}"
            self.node.get_logger().info(response .message)

        else:
            response.success = False
            response.message = f"Invalid mode '{request.mode}'. Valid modes: {valid_modes}"
            self.node.get_logger().warn(response.message)

        return response

    def update(self):
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        return super().terminate(new_status)

class IsWhatMode(py_trees.behaviour.Behaviour):

    def __init__(self, name, mode):
        super(IsWhatMode, self).__init__(name)
        self.mode = mode
        self.blackboard = self.attach_blackboard_client(name=self.__class__.__name__)
        self.blackboard.register_key(
            key="mode",
            access=py_trees.common.Access.READ
        )

    def update(self):
        if self.blackboard.mode == self.mode:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

