import py_trees
from controller_interfaces.srv import SetMode

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

