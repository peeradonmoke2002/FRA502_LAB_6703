#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import numpy as np

def generate_launch_description():
    # Package names
    robot_description_pkg = "robot_description"
    lab4_pkg = "lab4"

    # Paths
    robot_description_share = get_package_share_directory(robot_description_pkg)
    lab4_share = get_package_share_directory(lab4_pkg)

    rviz_config_path = os.path.join(lab4_share, "rviz", "lab4.rviz")

    # Include Robot State Publisher from robot_description
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_description_share, "launch", "rsp.launch.py")
        ),
        launch_arguments={"use_sim_time": "false"}.items()
    )

        # Set initial joint states to not face singularity
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{
            'source_list': ['joint_states'],
            'zeros': {
                'joint_1': 0.0,
                'joint_2': 0.0,  
                'joint_3': -np.pi/2    
            }
        }]
    )

    # Controller Node - handles IPK, TO, and AM modes
    controller_node = Node(
        package=lab4_pkg,
        executable='controller_bt.py',
        name='controller_node',
        output='screen',
    )

    # Random Pose Node - generates random valid targets within workspace
    random_pose_node = Node(
        package=lab4_pkg,
        executable='random_pos.py',
        name='random_pose_node',
        output='screen',
    )

    # RViz2 with LAB4 configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # Create LaunchDescription
    ld = LaunchDescription()

    # Add all nodes
    ld.add_action(rsp)
    ld.add_action(controller_node)
    ld.add_action(random_pose_node)
    ld.add_action(joint_state_publisher)
    ld.add_action(rviz_node)

    return ld
