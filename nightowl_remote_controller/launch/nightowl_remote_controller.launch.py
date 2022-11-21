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
    addLaunchArguments("container_name", "nightowl_remote_controller_container")
    addLaunchArguments("use_joy_container", "False")

    addLaunchArguments("fps", "12.5")
    addLaunchArguments("bitrate", "1000000")
    addLaunchArguments("signaling_server_url", "ws://127.0.0.1:8000/server")
    addLaunchArguments("ice_servers", '["turn:xqe2011:xqe2011@e-shower.xqe2011.cn:3478"]')

    addLaunchArguments("input_image", "/sensing/camera/front/left_image")
    addLaunchArguments("input_gear_report", "/vehicle/status/gear_status")
    addLaunchArguments("input_control_mode_report", "/vehicle/status/control_mode")
    addLaunchArguments("input_steering_report", "/vehicle/status/steering_status")
    addLaunchArguments("input_velocity_report", "/vehicle/status/velocity_status")
    addLaunchArguments("input_turn_indicators_report", "/vehicle/status/turn_indicators_status")
    addLaunchArguments("input_emergency", "/api/autoware/get/emergency")
    addLaunchArguments("input_gate_mode", "/control/current_gate_mode")
    addLaunchArguments("input_battery_flag_feedback", "/vehicle/driver/battery_flag_feedback")
    addLaunchArguments("input_autoware_state", "/autoware/state")

    return launch.LaunchDescription(
        launchArguments
        + [OpaqueFunction(function=launchSetup)]
    )


def launchSetup(context, *args, **kwargs):
    nightowl_remote_controller_component = ComposableNode(
        package="nightowl_remote_controller",
        plugin="nightowl_remote_controller::NightOwlRemoteControllerNode",
        name="nightowl_remote_controller",
        namespace="",
        remappings=[
            ("input/image", LaunchConfiguration("input_image")),
            ("input/gear_report", LaunchConfiguration("input_gear_report")),
            ("input/control_mode_report", LaunchConfiguration("input_control_mode_report")),
            ("input/steering_report", LaunchConfiguration("input_steering_report")),
            ("input/velocity_report", LaunchConfiguration("input_velocity_report")),
            ("input/turn_indicators_report", LaunchConfiguration("input_turn_indicators_report")),
            ("input/emergency", LaunchConfiguration("input_emergency")),
            ("input/gate_mode", LaunchConfiguration("input_gate_mode")),
            ("input/battery_flag_feedback", LaunchConfiguration("input_battery_flag_feedback")),
            ("input/autoware_state", LaunchConfiguration("input_autoware_state")),
        ],
        parameters=[{
            "fps": LaunchConfiguration("fps"),
            "bitrate": LaunchConfiguration("bitrate"),
            "signaling_server_url": LaunchConfiguration("signaling_server_url"),
            "ice_servers": LaunchConfiguration("ice_servers"),
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
        composable_node_descriptions=[nightowl_remote_controller_component],
        target_container=targetContainer
    )
    return [
        container,
        composableLoader
    ]