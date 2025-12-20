import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    pkg_name = 'quadruped_gait_controller'
    # Sécurité si le package n'est pas trouvé
    try:
        pkg_share = get_package_share_directory(pkg_name)
    except KeyError:
        return LaunchDescription([LogInfo(msg=f"Package {pkg_name} not found!")])

    # 1. Gazebo Simulation
    gazebo_launch_path = os.path.join(pkg_share, 'launch', 'gazebo_simulation.launch.py')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path)
    )
    
    # 2. Gait Controller (avec délai de 5s)
    gait_controller_node = TimerAction(
        period=5.0,
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
    
    # 3. RViz2 (Logique corrigée : on vérifie le fichier AVANT)
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'robot_view.rviz')
    launch_rviz = False # Changez à True si vous voulez RViz
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='log'
    )
    
    # 4. Teleop (Clavier)
    launch_teleop = False # Changez à True si vous voulez le clavier
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        output='screen',
        prefix='xterm -e', # Nécessite: sudo apt install xterm
        remappings=[('/cmd_vel', '/cmd_vel')]
    )

    # Création de la liste des choses à lancer
    ld_list = [
        gazebo_launch,
        gait_controller_node
    ]

    # Ajout conditionnel propre
    if launch_rviz and os.path.exists(rviz_config_file):
        ld_list.append(rviz_node)
    
    if launch_teleop:
        ld_list.append(teleop_node)

    return LaunchDescription(ld_list)
