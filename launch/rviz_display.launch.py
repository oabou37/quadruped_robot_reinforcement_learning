import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    pkg_name = 'quadruped_gait_control'
    pkg_share = get_package_share_directory(pkg_name)
    
    # URDF file path
    urdf_file = os.path.join(pkg_share, 'urdf', 'testudog.urdf')
    
    # Read URDF content
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': False
        }]
    )
    
    # Joint State Publisher (pour tester manuellement)
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    # RViz2
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'robot_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    # Gait Controller Node
    gait_controller_node = Node(
        package='quadruped_gait_control',
        executable='gait_controller',
        name='gait_controller',
        output='screen'
    )
    
    # Joint Command Relay - converts joint_commands to joint_states
    joint_relay_node = Node(
        package='quadruped_gait_control',
        executable='joint_command_relay',
        name='joint_command_relay',
        output='screen'
    )
    
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        gait_controller_node,
        joint_relay_node,
        rviz_node
    ])
