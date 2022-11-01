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
    addLaunchArguments("container_name", "joy2_container")
    addLaunchArguments("use_joy_container", "False")

    addLaunchArguments("autorepeat_rate", "20.0")
    addLaunchArguments("sticky_buttons", "False")
    addLaunchArguments("coalesce_interval_ms", "1")
    addLaunchArguments("device_id", "0")
    addLaunchArguments("device_name", "")
    addLaunchArguments("deadzone", "0.05")

    addLaunchArguments("input_joy_feedback", "/joy/set_feedback")
    addLaunchArguments("output_joy", "/joy/status")

    return launch.LaunchDescription(
        launchArguments
        + [OpaqueFunction(function=launchSetup)]
    )


def launchSetup(context, *args, **kwargs):
    baidu_joy_controller_component = ComposableNode(
        package="joy2",
        plugin="joy::Joy",
        name="joy2",
        namespace="",
        remappings=[
            ("output/joy", LaunchConfiguration("output_joy")),
            ("input/joy_feedback", LaunchConfiguration("input_joy_feedback")),
        ],
        parameters=[{
            "autorepeat_rate": LaunchConfiguration("autorepeat_rate"),
            "sticky_buttons": LaunchConfiguration("sticky_buttons"),
            "coalesce_interval_ms": LaunchConfiguration("coalesce_interval_ms"),
            "device_id": LaunchConfiguration("device_id"),
            "device_name": LaunchConfiguration("device_name"),
            "deadzone": LaunchConfiguration("deadzone"),
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
        condition=UnlessCondition(LaunchConfiguration("use_joy_container")),
        output="screen",
    )
    targetContainer = (
        container
        if UnlessCondition(LaunchConfiguration("use_joy_container")).evaluate(context)
        else LaunchConfiguration("container_name")
    )
    composableLoader = LoadComposableNodes(
        composable_node_descriptions=[baidu_joy_controller_component],
        target_container=targetContainer
    )
    return [
        container,
        composableLoader
    ]