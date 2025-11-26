import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Package names
    pkg_name = 'quadruped_gait_controller'
    pkg_gazebo_ros = FindPackageShare('gazebo_ros').find('gazebo_ros')
    
    # Paths
    pkg_share = get_package_share_directory(pkg_name)
    urdf_file = os.path.join(pkg_share, 'urdf', 'testudog.urdf')
    world_file = os.path.join(pkg_share, 'worlds', 'empty.world')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_x_position = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='X position of the robot'
    )
    
    declare_y_position = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Y position of the robot'
    )
    
    declare_z_position = DeclareLaunchArgument(
        'z_pose',
        default_value='0.5',
        description='Z position of the robot (height)'
    )
    
    # Read URDF file
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }]
    )
    
    # Joint State Publisher Node (for manual control if needed)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Gazebo server
    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )
    
    # Gazebo client
    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )
    
    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'quadruped_robot',
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose
        ],
        output='screen'
    )
    
    # ROS2 Control Node (for effort controllers)
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'use_sim_time': use_sim_time},
            os.path.join(pkg_share, 'config', 'controllers.yaml')
        ],
        output='screen'
    )
    
    # Load joint state broadcaster
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )
    
    # Load effort controllers for each joint
    load_effort_controllers = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'effort_controllers'],
        output='screen'
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_x_position,
        declare_y_position,
        declare_z_position,
        robot_state_publisher_node,
        start_gazebo_server,
        start_gazebo_client,
        spawn_robot,
        # controller_manager,  # Décommentez si vous utilisez ros2_control
        # load_joint_state_broadcaster,
        # load_effort_controllers,
    ])
