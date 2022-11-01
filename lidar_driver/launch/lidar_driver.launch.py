from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import LoadComposableNodes
from launch_ros.substitutions import FindPackageShare
import launch
import yaml

def generate_launch_description():
    launchArguments = []

    def addLaunchArguments(name: str, defaultValue=None):
        launchArguments.append(DeclareLaunchArgument(name, default_value=defaultValue))

    addLaunchArguments("use_intra_process", "True")
    addLaunchArguments("container_name", "lslidar_container")
    addLaunchArguments("use_lidar_container", "False")

    addLaunchArguments("frame_id", "lidar_link")
    addLaunchArguments("device_ip", "192.168.1.200")
    addLaunchArguments("msop_port", "2368")
    addLaunchArguments("difop_port", "2369")
    addLaunchArguments("model", "LSC16")
    addLaunchArguments("rpm", "600.0")
    addLaunchArguments("return_mode", "1")
    addLaunchArguments("pcap", "")
    addLaunchArguments("time_synchronization", "false")

    addLaunchArguments("output_topic_msop", "lslidar/msop_packet")
    addLaunchArguments("output_topic_difop", "lslidar/difop_packet")
    addLaunchArguments("output_topic_sync", "lslidar/sync_header")

    return launch.LaunchDescription(
        launchArguments
        + [OpaqueFunction(function=launchSetup)]
    )


def launchSetup(context, *args, **kwargs):
    lidar_control_component = ComposableNode(
        package="lidar_driver",
        plugin="lidar_driver::LidarDriver",
        name="lidar_driver",
        namespace="",
        remappings=[
            ("output/msop_packet", [f"/sensing/lidar/top/", LaunchConfiguration("output_topic_msop")]),
            ("output/difop_packet",[f"/sensing/lidar/top/",  LaunchConfiguration("output_topic_difop")]),
            ("output/sync_header", [f"/sensing/lidar/top/", LaunchConfiguration("output_topic_sync")])
        ],
        parameters=[{
            "frame_id": LaunchConfiguration("frame_id"),
            "device_ip": LaunchConfiguration("device_ip"),
            "msop_port": LaunchConfiguration("msop_port"),
            "difop_port": LaunchConfiguration("difop_port"),
            "model": LaunchConfiguration("model"),
            "rpm": LaunchConfiguration("rpm"),
            "return_mode": LaunchConfiguration("return_mode"),
            "pcap": LaunchConfiguration("pcap"),
            "time_synchronization": LaunchConfiguration("time_synchronization"),
        }],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )
    # 如果是单线程模式就直接加载容器
    container = ComposableNodeContainer(
        name=LaunchConfiguration("container_name"),
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[],
        condition=UnlessCondition(LaunchConfiguration("use_lidar_container")),
        output="screen",
    )
    targetContainer = (
        container
        if UnlessCondition(LaunchConfiguration("use_lidar_container")).evaluate(context)
        else LaunchConfiguration("container_name")
    )
    composableLoader = LoadComposableNodes(
        composable_node_descriptions=[lidar_control_component],
        target_container=targetContainer
    )
    return [
        container,
        composableLoader
    ]