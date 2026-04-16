from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare('patrick_main'), 'config', 'patrick_params.yaml'
    ])

    fastsam = Node(
        package='patrick_vision',
        executable='fastsam_color_detector.py',
        name='fastsam_color_detector',
        output='screen',
        parameters=[params],
    )

    frame_pub = Node(
        package='patrick_vision',
        executable='object_frame_publisher',
        name='object_frame_publisher',
        output='screen',
        parameters=[params],
    )

    marker_pub = Node(
        package='patrick_vision',
        executable='object_marker_publisher',
        name='object_marker_publisher',
        output='screen',
        parameters=[params],
    )

    return LaunchDescription([fastsam, frame_pub, marker_pub])
