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
    addLaunchArguments("container_name", "starneto_autoware_interface_container")
    addLaunchArguments("use_ins_container", "False")

    addLaunchArguments("gnss_receiver", "starneto")
    addLaunchArguments("coordinate_system", "1")
    addLaunchArguments("base_frame", "base_link")

    addLaunchArguments("input_topic_gpfpd", "gnss/raw_data")
    addLaunchArguments("input_topic_gtimu", "imu/raw_data")

    addLaunchArguments("output_topic_fix", "/fix")
    addLaunchArguments("output_topic_navpvt", "/navpvt")
    addLaunchArguments("output_topic_imu", "imu_raw")

    return launch.LaunchDescription(
        launchArguments
        + [OpaqueFunction(function=launchSetup)]
    )


def launchSetup(context, *args, **kwargs):
    starneto_autoware_interface_component = ComposableNode(
        package="starneto_autoware_interface",
        plugin="starneto_autoware_interface::StarnetoAutowareInterfaceNode",
        name="starneto_autoware_interface",
        namespace="",
        remappings=[
            ("input_topic_gpfpd", [f"/sensing/ins/starneto/", LaunchConfiguration("input_topic_gpfpd")]),
            ("input_topic_gtimu", [f"/sensing/ins/starneto/", LaunchConfiguration("input_topic_gtimu")]),
            ("fix", LaunchConfiguration("output_topic_fix")),
            ("navpvt", LaunchConfiguration("output_topic_navpvt")),
            ("imu/data_raw", LaunchConfiguration("output_topic_imu"))
        ],
        parameters=[{
            "coordinate_system": LaunchConfiguration("coordinate_system"),
            "base_frame": LaunchConfiguration("base_frame")
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
        condition=UnlessCondition(LaunchConfiguration("use_ins_container")),
        output="screen"
    )
    targetContainer = (
        container
        if UnlessCondition(LaunchConfiguration("use_ins_container")).evaluate(context)
        else LaunchConfiguration("container_name")
    )
    composableLoader = LoadComposableNodes(
        composable_node_descriptions=[starneto_autoware_interface_component],
        target_container=targetContainer
    )
    return [
        container,
        composableLoader
    ]