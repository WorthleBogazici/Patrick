import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_main = FindPackageShare('patrick_main')

    xacro_file = PathJoinSubstitution([pkg_main, 'urdf', 'patrick.urdf.xacro'])
    rviz_config = PathJoinSubstitution([pkg_main, 'rviz', 'robot.rviz'])

    use_sim_arg = DeclareLaunchArgument('use_sim', default_value='false')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')

    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' use_sim:=', LaunchConfiguration('use_sim'),
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        use_sim_arg,
        rviz_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ])
