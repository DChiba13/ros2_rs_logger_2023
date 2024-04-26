from inspect import Parameter
from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_prefix = get_package_share_directory('ros2_rs_logger_2023')
    components = LoadComposableNodes(
        target_container="rs_container",
        composable_node_descriptions=[
            ComposableNode(
                package='ros2_rs_logger_2023',
                plugin='project_ryusei::ROS2Logger',
                name='rs_logger',
                parameters=[join(pkg_prefix, "cfg/logger.yaml")],
                remappings=[
                    # subscriber
                    ("/camera/image",            "/camera1/image"),
                    ("/top/points",              "/c32/points"),
                    ("/middle/points",           "/mrs1000/points"),
                    ("/bottom/points",           "/tim551/points"),
                    ("/mercury/state",           "/mercury/state"),
                    # ("/sonar/ranges",            "/mercury_sonar/ranges"),
                    ("/location/corrected_pose", "/locator/corrected_pose"),

                    # publisher
                    ("~/is_active", "~/is_active"),
                    ("~/file_name", "~/filename"),

                    # service
                    ("~/set_active",   "~/set_active"),
                    ("~/trigger_save", "~/trigger_save"),
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ]
    )
    return LaunchDescription([
        components
    ])
