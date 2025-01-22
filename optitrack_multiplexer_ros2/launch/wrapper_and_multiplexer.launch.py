from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()
    # Create the NatNet client node
    config_wrapper = os.path.join(
        get_package_share_directory('optitrack_wrapper_ros2'),
        'config',
        'optitrack_wrapper_config.yaml'
    )
    optitrack_wrapper = Node(
        package='optitrack_wrapper_ros2',
        executable='wrapper_node',
        name='optitrack_wrapper_node',
        parameters=[config_wrapper],
        # prefix=['xterm -e gdb -ex run --args'],
        output='screen',
        emulate_tty=True
    )

    config_multiplexer = os.path.join(
        get_package_share_directory('optitrack_multiplexer_ros2'),
        'config',
        'optitrack_multiplexer_config.yaml'
    )
    optitrack_multiplexer = Node(
        package='optitrack_multiplexer_ros2',
        executable='multiplexer_node',
        name='optitrack_multiplexer_node',
        parameters=[config_multiplexer],
        # prefix=['xterm -e gdb -ex run --args'],
        output='screen',
        emulate_tty=True
    )

    ld.add_action(optitrack_wrapper)
    ld.add_action(optitrack_multiplexer)
    return ld
