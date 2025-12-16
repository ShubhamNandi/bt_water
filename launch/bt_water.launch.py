from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_share = get_package_share_directory('bt_water')
    config_file = os.path.join(package_share, 'config', 'config.yaml')
    
    return LaunchDescription([
        Node(
            package='bt_water',
            executable='bt_executor_node',
            name='bt_executor_node',
            output='screen',
            parameters=[config_file],
            emulate_tty=True,
        ),
    ])

