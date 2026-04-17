#!/usr/bin/env python3
"""Patrick MoveIt2 bringup — launches move_group + RViz MotionPlanning.

Attaches to an already-running patrick_bringup (robot_state_publisher +
ros2_control + controllers). Selects the sim or real-HW MoveIt controller
bridge YAML via the `simulation` launch argument.

Usage (two shells):
    # shell 1 — robot + controllers
    ros2 launch patrick_main patrick_bringup.launch.py simulation:=true

    # shell 2 — MoveIt
    ros2 launch patrick_moveit_config patrick_moveit.launch.py \
        simulation:=true use_sim_time:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    rviz = LaunchConfiguration('rviz').perform(context)
    simulation = LaunchConfiguration('simulation').perform(context).lower() == 'true'

    patrick_main_share = get_package_share_directory('patrick_main')
    xacro_path = os.path.join(patrick_main_share, 'urdf', 'patrick.urdf.xacro')

    controllers_file = ('config/moveit_controllers.yaml'
                        if simulation else 'config/moveit_controllers_hw.yaml')

    moveit_config = (
        MoveItConfigsBuilder('patrick', package_name='patrick_moveit_config')
        .robot_description(
            file_path=xacro_path,
            mappings={'use_sim': 'true' if simulation else 'false'},
        )
        .robot_description_semantic(file_path='config/patrick.srdf')
        .trajectory_execution(file_path=controllers_file)
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .planning_pipelines(
            pipelines=['ompl', 'pilz_industrial_motion_planner', 'chomp', 'stomp'],
            default_planning_pipeline='ompl',
        )
        .planning_scene_monitor(
            publish_robot_description=False,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
        )
        .to_moveit_configs()
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': use_sim_time == 'true'},
            {'publish_robot_description_semantic': True},
        ],
    )

    rviz_config = os.path.join(
        get_package_share_directory('patrick_moveit_config'),
        'config', 'moveit.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='moveit_rviz',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {'use_sim_time': use_sim_time == 'true'},
        ],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return [move_group_node, rviz_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='true when running against Gazebo'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Launch RViz with MotionPlanning plugin'),
        DeclareLaunchArgument(
            'simulation', default_value='true',
            description=('true → moveit_controllers.yaml (arm_controller, JTC); '
                         'false → moveit_controllers_hw.yaml '
                         '(scaled_arm_controller, Scaled JTC)')),
        OpaqueFunction(function=_launch_setup),
    ])
