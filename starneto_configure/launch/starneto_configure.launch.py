from sys import executable

from numpy import outer
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    container = ComposableNodeContainer(
        name='starneto_configure_container',
        namespace='starneto_configure',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package="starneto_configure",
                plugin="starneto_configure::StarnetoConfigureNode",
                name="starneto_configure_node",
                parameters=[
                    {"configure_server_address" : "192.168.1.9"},
                    {"udp_server_port" : 7001},
                    {"configure_client_address" : "192.168.1.19"},
                    {"udp_client_port" : 8001},
                ],
            )
        ],
        output='screen',
    )

    return LaunchDescription([container])