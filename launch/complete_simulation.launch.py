import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    pkg_name = 'quadruped_gait_controller'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Include Gazebo simulation launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'gazebo_simulation.launch.py')
        )
    )
    
    # Gait Controller Node (delayed start to ensure Gazebo is ready)
    gait_controller_node = TimerAction(
        period=5.0,  # Wait 5 seconds for Gazebo to fully start
        actions=[
            Node(
                package=pkg_name,
                executable='gait_controller',
                name='gait_controller',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'step_length': 0.2,
                    'step_height': 0.03,
                    'robot_height': 0.33
                }]
            )
        ]
    )
    
    # RViz2 for visualization (optional)
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'robot_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        condition=lambda context: os.path.exists(rviz_config_file)
    )
    
    # Teleop keyboard for manual control
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        output='screen',
        prefix='xterm -e',  # Opens in separate terminal
        remappings=[('/cmd_vel', '/cmd_vel')]
    )
    
    return LaunchDescription([
        gazebo_launch,
        gait_controller_node,
        # rviz_node,  # Décommentez pour lancer RViz
        # teleop_node,  # Décommentez pour contrôle clavier
    ])
