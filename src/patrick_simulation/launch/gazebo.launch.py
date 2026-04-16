import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_main = FindPackageShare('patrick_main')
    pkg_simulation = FindPackageShare('patrick_simulation')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')

    # ── Launch arguments ─────────────────────────────────────────────────
    world_arg = DeclareLaunchArgument(
        'world_file',
        default_value=os.path.join(
            get_package_share_directory('aws_robomaker_small_house_world'),
            'worlds', 'small_house.world'),
        description='Path to the Gazebo world file (small_house, bookstore, warehouse, etc.)',
    )
    gui_arg = DeclareLaunchArgument('gui', default_value='true')

    # ── Environment variables for Gazebo model/resource paths ────────────
    kobuki_models = os.path.join(get_package_share_directory('kobuki'), 'models')
    ament_prefix = os.environ.get('AMENT_PREFIX_PATH', '')
    gz_resource_paths = [
        os.path.join(p, 'share') for p in ament_prefix.split(':') if p
    ]
    gz_resource_paths.append(kobuki_models)
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if existing_gz_path:
        gz_resource_paths.append(existing_gz_path)
    gz_resource_path_value = ':'.join(gz_resource_paths)

    # ── Robot description (URDF via xacro, simulation mode) ──────────────
    xacro_file = PathJoinSubstitution([pkg_main, 'urdf', 'patrick.urdf.xacro'])
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file, ' use_sim:=true']),
        value_type=str,
    )

    # ── Robot State Publisher ────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        output='screen',
    )

    # ── Gazebo server ───────────────────────────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': ['-r -v 4 ', LaunchConfiguration('world_file')],
        }.items(),
    )

    # ── Spawn Patrick robot into Gazebo ─────────────────────────────────
    gz_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'patrick',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.05',
            '--ros-args', '--log-level', 'warn',
        ],
        output='screen',
    )

    # ── Gazebo ↔ ROS bridge (clock) ─────────────────────────────────────
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # ── Static TF: world → odom ─────────────────────────────────────────
    static_tf_world_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom'],
        output='screen',
    )

    # ── Controller spawners ─────────────────────────────────────────────
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Chain arm/gripper/diff after joint_state_broadcaster is up
    delay_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                diff_drive_spawner,
                arm_controller_spawner,
                gripper_controller_spawner,
            ],
        ),
    )

    return LaunchDescription([
        # Environment
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=gz_resource_path_value),
        # Arguments
        world_arg,
        gui_arg,
        # Core
        robot_state_publisher,
        static_tf_world_odom,
        # Gazebo
        gz_sim,
        gz_spawn,
        gz_bridge,
        # Controllers
        joint_state_broadcaster_spawner,
        delay_controllers,
    ])
