from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare('patrick_main'), 'config', 'patrick_params.yaml'
    ])

    manipulation = LifecycleNode(
        package='patrick_manipulation',
        executable='manipulation_node',
        name='manipulation_node',
        namespace='',
        output='screen',
        parameters=[params],
    )

    return LaunchDescription([manipulation])
