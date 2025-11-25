from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('drive_arbitration')
    config = os.path.join(pkg_share, 'config', 'drive_arbitration_params.yaml')

    return LaunchDescription([
        Node(
            package='drive_arbitration',
            executable='drive_arbitrator_node',
            name='drive_arbitrator_node',
            output='screen',
            parameters=[config]
        )
    ])
