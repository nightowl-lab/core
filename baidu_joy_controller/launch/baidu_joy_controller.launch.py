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
    addLaunchArguments("container_name", "baidu_joy_controller_container")
    addLaunchArguments("use_joy_container", "False")

    addLaunchArguments("steering_max_angle", "0.43")
    addLaunchArguments("forward_ratio", "0.5")
    addLaunchArguments("backward_ratio", "0.5")
    addLaunchArguments("external_cmd_source", "remote")
    addLaunchArguments("joy_timeout", "100.0")

    addLaunchArguments("input_joy", "/joy/status")
    addLaunchArguments("input_velocity_report", "/vehicle/status/velocity_status")
    addLaunchArguments("input_state_report", "/autoware/state")
    addLaunchArguments("output_control_command", ("/api/external/set/command/", LaunchConfiguration("external_cmd_source"), "/control"))
    addLaunchArguments("output_shift", ("/api/external/set/command/", LaunchConfiguration("external_cmd_source"), "/shift"))
    addLaunchArguments("output_turn_signal", ("/api/external/set/command/", LaunchConfiguration("external_cmd_source"), "/turn_signal"))
    addLaunchArguments("output_gate_mode", "/control/gate_mode_cmd")
    addLaunchArguments("output_heartbeat", ("/api/external/set/command/", LaunchConfiguration("external_cmd_source"), "/heartbeat"))
    addLaunchArguments("output_vehicle_engage", "/vehicle/engage")
    addLaunchArguments("output_joy_feedback", "/joy/set_feedback")
    addLaunchArguments("service_emergency_stop", "/api/autoware/set/emergency")
    addLaunchArguments("service_autoware_engage", "/api/external/set/engage")
    addLaunchArguments("service_client_engage", "/api/autoware/set/engage")

    return launch.LaunchDescription(
        launchArguments
        + [OpaqueFunction(function=launchSetup)]
    )


def launchSetup(context, *args, **kwargs):
    baidu_joy_controller_component = ComposableNode(
        package="baidu_joy_controller",
        plugin="baidu_joy_controller::BaiduJoyControllerNode",
        name="baidu_joy_controller",
        namespace="",
        remappings=[
            ("input/joy", LaunchConfiguration("input_joy")),
            ("input/velocity_report", LaunchConfiguration("input_velocity_report")),
            ("input/state_report", LaunchConfiguration("input_state_report")),
            ("output/control_command", LaunchConfiguration("output_control_command")),
            ("output/shift", LaunchConfiguration("output_shift")),
            ("output/turn_signal", LaunchConfiguration("output_turn_signal")),
            ("output/gate_mode", LaunchConfiguration("output_gate_mode")),
            ("output/heartbeat", LaunchConfiguration("output_heartbeat")),
            ("output/vehicle_engage", LaunchConfiguration("output_vehicle_engage")),
            ("output/joy_feedback", LaunchConfiguration("output_joy_feedback")),
            ("service/emergency_stop", LaunchConfiguration("service_emergency_stop")),
            ("service/autoware_engage", LaunchConfiguration("service_autoware_engage")),
            ("service/client_engage", LaunchConfiguration("service_client_engage")),
        ],
        parameters=[{
            "steering_max_angle": LaunchConfiguration("steering_max_angle"),
            "forward_ratio": LaunchConfiguration("forward_ratio"),
            "backward_ratio": LaunchConfiguration("backward_ratio"),
            "joy_timeout": LaunchConfiguration("joy_timeout"),
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