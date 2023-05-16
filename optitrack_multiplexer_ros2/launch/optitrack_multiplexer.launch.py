from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()
    # Create the NatNet client node
    config = os.path.join(
        get_package_share_directory('optitrack_multiplexer_ros2'),
        'config',
        'optitrack_multiplexer_config.yaml'
    )
    natnet_client = Node(
        package='optitrack_multiplexer_ros2',
        executable='multiplexer_node',
        name='optitrack_multiplexer_node',
        parameters=[config],
        # prefix=['xterm -e gdb -ex run --args'],
        output='screen',
        emulate_tty=True
    )

    ld.add_action(natnet_client)
    return ld
