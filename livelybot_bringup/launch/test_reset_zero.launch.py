from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('livelybot_description'),
                'launch',
                'livelybot_description.launch.py'
            )
        )
    )

    test_reset_zero_node = Node(
        package='livelybot_serial',
        executable='test_reset_zero',
        name='test_reset_zero',
        output='screen',
        parameters=[LaunchConfiguration('robot_param_file')],
    )

    return LaunchDescription([
        description_launch,
        test_reset_zero_node
    ])
