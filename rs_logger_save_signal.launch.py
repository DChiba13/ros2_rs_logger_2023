from inspect import Parameter
from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_prefix = get_package_share_directory('ros2_rs_miyauchi_logger')

    container = Node(
	    package='rclcpp_components',
		executable='component_container',
		name = 'logger_save_signal_container',
		emulate_tty = True,
		output = 'screen'
	)

    components = LoadComposableNodes(
        target_container="logger_save_signal_container",
        composable_node_descriptions=[
            ComposableNode(
                package='ros2_rs_miyauchi_logger',
                plugin='project_ryusei::ROS2LoggerSaveSignal',
                name='rs_logger_save',
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ]
    )
    return LaunchDescription([
        container,
        components
    ])