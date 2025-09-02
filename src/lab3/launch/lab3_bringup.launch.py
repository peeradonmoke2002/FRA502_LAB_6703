from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.event_handlers import OnProcessStart
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction


def generate_launch_description():
    # Launch arguments for the turtle names
    turtle_eater_name = DeclareLaunchArgument(
        'eater_name',
        default_value='eater_turtle',
    )
    
    turtle_killer_name = DeclareLaunchArgument(
        'killer_name',
        default_value='killer_turtle',
    )
    
    sampling_frequency_data = DeclareLaunchArgument(
        'sampling_frequency_data',
        default_value='100',
    )
    
    max_pizza = DeclareLaunchArgument(
        'max_pizza',
        default_value='10',
    )

    turtlesim_plus = Node(
        package='turtlesim_plus',
        executable='turtlesim_plus_node.py',  
        name='turtlesim_plus',
        output='screen'
    )
    
    # Create the eater node with the turtle_name parameter
    eater_node = Node(
        package='lab3',
        executable='eater.py',
        name='eater_node',
        parameters=[{
            'max_pizza': LaunchConfiguration('max_pizza'),
            'eater_name': LaunchConfiguration('eater_name'),
            'sampling_frequency': LaunchConfiguration('sampling_frequency_data')
        }],
        output='screen'
    )
    
    killer_node = Node(
        package='lab3',
        executable='killer.py',
        name='killer_node',
        parameters=[{
            'eater_name':  LaunchConfiguration('eater_name'),
            'killer_name': LaunchConfiguration('killer_name'),
            'sampling_frequency': LaunchConfiguration('sampling_frequency_data')

        }],
        output='screen'
    )
    
    kill_default = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/remove_turtle', 'turtlesim/srv/Kill', '{name: "turtle1"}'],
        output='screen'
    )
  

    spawn_eater = ExecuteProcess(
        cmd=[[
            'ros2 service call ',
            '/spawn_turtle ',                              
            'turtlesim/srv/Spawn ',                        
            '"{x: 0.1, y: 0.1, theta: 0.0, name: \\"',
            LaunchConfiguration('eater_name'),
            '\\"}"'
        ]],
        shell=True,
        output='screen'
    )

    spawn_killer = ExecuteProcess(
        cmd=[[
            'ros2 service call ',
            '/spawn_turtle ',
            'turtlesim/srv/Spawn ',
            '"{x: 0.1, y: 0.1, theta: 0.0, name: \\"',
            LaunchConfiguration('killer_name'),
            '\\"}"'
        ]],
        shell=True,
        output='screen'
    )
    
    ld = LaunchDescription()
    



    ld.add_action(turtle_eater_name)
    ld.add_action(turtle_killer_name)
    ld.add_action(max_pizza)
    ld.add_action(sampling_frequency_data)
    ld.add_action(turtlesim_plus)
    ld.add_action(spawn_eater)
    ld.add_action(spawn_killer)
    ld.add_action(kill_default)
    ld.add_action(eater_node)
    ld.add_action(killer_node)
    
    return ld
