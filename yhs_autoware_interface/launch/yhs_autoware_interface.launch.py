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

    addLaunchArguments(
        "vehicle_info_param_file",
        [
            FindPackageShare("vehicle_info_util"),
            "/config/vehicle_info.param.yaml",
        ]
    )
    addLaunchArguments("use_intra_process", "True")
    addLaunchArguments("container_name", "yhs_autoware_interface_container")
    addLaunchArguments("use_vehicle_container", "False")
    addLaunchArguments("input_ctrl_feeback", "/vehicle/driver/ctrl_feedback")
    addLaunchArguments("input_io_feedback", "/vehicle/driver/io_feedback")
    addLaunchArguments("input_ackermann_control_command", "/control/command/control_cmd")
    addLaunchArguments("input_gear_command", "/control/command/gear_cmd")
    addLaunchArguments("input_turn_indicators_command", "/control/command/turn_indicators_cmd")
    addLaunchArguments("input_engage_command", "/vehicle/engage")
    addLaunchArguments("input_vehicle_emergency_command", "/control/command/emergency_cmd")
    addLaunchArguments("output_ctrl_command", "/vehicle/driver/ctrl_command")
    addLaunchArguments("output_io_command", "/vehicle/driver/io_command")
    addLaunchArguments("output_control_mode_report", "/vehicle/status/control_mode")
    addLaunchArguments("output_velocity_report", "/vehicle/status/velocity_status")
    addLaunchArguments("output_steering_report", "/vehicle/status/steering_status")
    addLaunchArguments("output_gear_report", "/vehicle/status/gear_status")
    addLaunchArguments("output_turn_indicators_report", "/vehicle/status/turn_indicators_status")

    return launch.LaunchDescription(
        launchArguments
        + [OpaqueFunction(function=launchSetup)]
    )


def launchSetup(context, *args, **kwargs):
    # 加载车辆配置文件
    vehicle_info_param_path = LaunchConfiguration("vehicle_info_param_file").perform(context)
    with open(vehicle_info_param_path, "r") as f:
        vehicle_info_param = yaml.safe_load(f)["/**"]["ros__parameters"]
    yhs_autoware_interface_component = ComposableNode(
        package="yhs_autoware_interface",
        plugin="yhs_autoware_interface::YHSAutowareInterfaceNode",
        name="yhs_autoware_interface",
        namespace="",
        remappings=[
            ("input/ctrl_feeback", LaunchConfiguration("input_ctrl_feeback")),
            ("input/io_feedback", LaunchConfiguration("input_io_feedback")),
            ("input/ackermann_control_command", LaunchConfiguration("input_ackermann_control_command")),
            ("input/gear_command", LaunchConfiguration("input_gear_command")),
            ("input/turn_indicators_command", LaunchConfiguration("input_turn_indicators_command")),
            ("input/engage_command", LaunchConfiguration("input_engage_command")),
            ("input/vehicle_emergency_command", LaunchConfiguration("input_vehicle_emergency_command")),
            ("output/ctrl_command", LaunchConfiguration("output_ctrl_command")),
            ("output/io_command", LaunchConfiguration("output_io_command")),
            ("output/control_mode_report", LaunchConfiguration("output_control_mode_report")),
            ("output/velocity_report", LaunchConfiguration("output_velocity_report")),
            ("output/steering_report", LaunchConfiguration("output_steering_report")),
            ("output/gear_report", LaunchConfiguration("output_gear_report")),
            ("output/turn_indicators_report", LaunchConfiguration("output_turn_indicators_report"))
        ],
        parameters=[
            vehicle_info_param
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )
    # 如果是单线程模式就直接加载容器
    container = ComposableNodeContainer(
        name=LaunchConfiguration("container_name"),
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[],
        condition=UnlessCondition(LaunchConfiguration("use_vehicle_container")),
        output="screen",
    )
    targetContainer = (
        container
        if UnlessCondition(LaunchConfiguration("use_vehicle_container")).evaluate(context)
        else LaunchConfiguration("container_name")
    )
    composableLoader = LoadComposableNodes(
        composable_node_descriptions=[yhs_autoware_interface_component],
        target_container=targetContainer
    )
    return [
        container,
        composableLoader
    ]