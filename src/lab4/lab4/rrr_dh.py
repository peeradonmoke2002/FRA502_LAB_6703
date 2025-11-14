import numpy as np
from roboticstoolbox.robot.ERobot import ERobot
from ament_index_python import get_package_share_directory
import os

class RRRRobot(ERobot):
    def __init__(self):
        pkg_path = os.path.join(get_package_share_directory('robot_description'))
        xacro_file = os.path.join(pkg_path,'urdf','my-robot.xacro')
        links, name, urdf_string, urdf_filepath = self.URDF_read(
            xacro_file
        )
        super().__init__(
            links,
            name=name.upper(),
            manufacturer="RRR Robot",
            urdf_string=urdf_string,
            urdf_filepath=urdf_filepath,
        )

        self.qz = np.zeros(self.n)
        self.addconfiguration("qz", self.qz)


