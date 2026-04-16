from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare('patrick_main'), 'config', 'patrick_params.yaml'
    ])

    navigation = LifecycleNode(
        package='patrick_navigation',
        executable='navigation_node',
        name='navigation_node',
        namespace='',
        output='screen',
        parameters=[params],
    )

    return LaunchDescription([navigation])
