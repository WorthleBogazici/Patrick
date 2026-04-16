import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── Package paths ────────────────────────────────────────────────────────
    pkg_main = FindPackageShare('patrick_main')
    pkg_kobuki = get_package_share_directory('kobuki')

    # ── Launch arguments ─────────────────────────────────────────────────────
    gui_arg = DeclareLaunchArgument('gui', default_value='true')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
    vision_arg = DeclareLaunchArgument('vision', default_value='true')
    simulation_arg = DeclareLaunchArgument(
        'simulation', default_value='false',
        description='true = Gazebo simulation, false = real hardware (Arduino + Kobuki)')
    kinect_arg = DeclareLaunchArgument(
        'kinect', default_value='false',
        description='true = use Kinect camera, false = use ZED2 camera')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            get_package_share_directory('aws_robomaker_small_house_world'),
            'worlds', 'small_house.world'),
        description='Path to Gazebo world file')
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_kobuki, 'maps', 'aws_house.yaml'),
        description='Path to map YAML for nav2 / RViz')

    use_sim = LaunchConfiguration('simulation')
    use_kinect = LaunchConfiguration('kinect')

    # ── Paths ────────────────────────────────────────────────────────────────
    xacro_file = PathJoinSubstitution([pkg_main, 'urdf', 'patrick.urdf.xacro'])
    controllers_yaml = PathJoinSubstitution([pkg_main, 'config', 'patrick_controllers.yaml'])
    params_yaml = PathJoinSubstitution([pkg_main, 'config', 'patrick_params.yaml'])
    rviz_config = PathJoinSubstitution([pkg_main, 'rviz', 'robot.rviz'])
    world_file = LaunchConfiguration('world')

    # ── Robot description ────────────────────────────────────────────────────
    camera_type = PythonExpression([
        "'kinect' if '", use_kinect, "' == 'true' else 'zed2'"
    ])

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', xacro_file,
            ' use_sim:=', use_sim,
            ' camera_type:=', camera_type,
        ]),
        value_type=str,
    )

    # ── Robot State Publisher ────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim,
        }],
        output='screen',
    )

    # ══════════════════════════════════════════════════════════════════════════
    #  SIMULATION MODE
    # ══════════════════════════════════════════════════════════════════════════

    # Gazebo Harmonic
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py',
            ])
        ),
        launch_arguments={'gz_args': ['-r -v 4 ', world_file]}.items(),
        condition=IfCondition(use_sim),
    )

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
        condition=IfCondition(use_sim),
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/rgbd_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/rgbd_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/rgbd_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        output='screen',
        condition=IfCondition(use_sim),
    )

    # Controller spawners (simulation — diff-drive + arm + gripper via gz_ros2_control)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=IfCondition(use_sim),
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=IfCondition(use_sim),
    )

    arm_controller_spawner_sim = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=IfCondition(use_sim),
    )

    delay_sim_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                diff_drive_spawner,
                arm_controller_spawner_sim,
            ],
        ),
        condition=IfCondition(use_sim),
    )

    # ══════════════════════════════════════════════════════════════════════════
    #  REAL HARDWARE MODE
    # ══════════════════════════════════════════════════════════════════════════

    # Kobuki base driver (third-party kobuki_node package)
    kobuki_ros_node = Node(
        package='kobuki_node',
        executable='kobuki_ros_node',
        name='kobuki_ros_node',
        output='screen',
        parameters=[params_yaml],
        remappings=[('/commands/velocity', '/cmd_vel')],
        condition=UnlessCondition(use_sim),
    )

    # Arm controller spawners (real HW — ArduinoHardwareInterface loaded from URDF)
    hw_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=UnlessCondition(use_sim),
    )

    hw_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=UnlessCondition(use_sim),
    )

    delay_hw_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=hw_joint_state_broadcaster,
            on_exit=[
                hw_arm_controller_spawner,
            ],
        ),
        condition=UnlessCondition(use_sim),
    )

    # ══════════════════════════════════════════════════════════════════════════
    #  COMMON
    # ══════════════════════════════════════════════════════════════════════════

    static_tf_world_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom'],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': use_sim}],
    )

    # Vision pipeline (delayed start)
    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('patrick_vision'), 'launch', 'vision.launch.py',
            ])
        ),
        condition=IfCondition(LaunchConfiguration('vision')),
    )

    delayed_vision = TimerAction(
        period=5.0,
        actions=[vision_launch],
        condition=IfCondition(LaunchConfiguration('vision')),
    )

    # ── Environment variable for Gazebo model paths ─────────────────────
    kobuki_models = os.path.join(pkg_kobuki, 'models')
    ament_prefix = os.environ.get('AMENT_PREFIX_PATH', '')
    gz_resource_paths = [
        os.path.join(p, 'share') for p in ament_prefix.split(':') if p
    ]
    gz_resource_paths.append(kobuki_models)
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if existing_gz_path:
        gz_resource_paths.append(existing_gz_path)
    gz_resource_path_value = ':'.join(gz_resource_paths)

    return LaunchDescription([
        # Environment
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=gz_resource_path_value),
        # Arguments
        gui_arg,
        rviz_arg,
        vision_arg,
        simulation_arg,
        kinect_arg,
        world_arg,
        map_arg,
        # Core
        robot_state_publisher,
        static_tf_world_odom,
        # Simulation
        gz_sim,
        gz_spawn,
        gz_bridge,
        joint_state_broadcaster_spawner,
        delay_sim_controllers,
        # Real hardware
        kobuki_ros_node,
        hw_joint_state_broadcaster,
        delay_hw_controllers,
        # Visualization
        rviz_node,
        # Vision (delayed)
        delayed_vision,
    ])
