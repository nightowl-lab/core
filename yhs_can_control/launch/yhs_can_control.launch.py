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
    addLaunchArguments("container_name", "yhs_can_control_container")
    addLaunchArguments("use_vehicle_container", "False")

    addLaunchArguments("input_can_send", "/vehicle/can/send")
    addLaunchArguments("output_can_receive", "/vehicle/can/receive")
    addLaunchArguments("output_ctrl_feedback", "/vehicle/driver/ctrl_feedback")
    addLaunchArguments("output_left_wheel_feedback", "/vehicle/driver/left_wheel_feedback")
    addLaunchArguments("output_right_wheel_feedback", "/vehicle/driver/right_wheel_feedback")
    addLaunchArguments("output_io_feedback", "/vehicle/driver/io_feedback")
    addLaunchArguments("output_odometry_feedback", "/vehicle/driver/odometry_feedback")
    addLaunchArguments("output_battery_information_feedback", "/vehicle/driver/battery_information_feedback")
    addLaunchArguments("output_battery_flag_feedback", "/vehicle/driver/battery_flag_feedback")
    addLaunchArguments("output_wheel_encoder_feedback", "/vehicle/driver/wheel_encoder_feedback")
    addLaunchArguments("output_diagnostic_feedback", "/vehicle/driver/diagnostic_feedback")
    addLaunchArguments("output_is_online", "/vehicle/driver/is_online")
    addLaunchArguments("input_ctrl_command", "/vehicle/driver/ctrl_command")
    addLaunchArguments("input_io_command", "/vehicle/driver/io_command")

    return launch.LaunchDescription(
        launchArguments
        + [OpaqueFunction(function=launchSetup)]
    )


def launchSetup(context, *args, **kwargs):
    yhs_can_control_component = ComposableNode(
        package="yhs_can_control",
        plugin="yhs_can_control::YHSCANControlNode",
        name="yhs_can_control",
        namespace="",
        remappings=[
            ("input/can_receive", LaunchConfiguration("output_can_receive")),
            ("output/can_send", LaunchConfiguration("input_can_send")),
            ("output/ctrl_feedback", LaunchConfiguration("output_ctrl_feedback")),
            ("output/left_wheel_feedback", LaunchConfiguration("output_left_wheel_feedback")),
            ("output/right_wheel_feedback", LaunchConfiguration("output_right_wheel_feedback")),
            ("output/io_feedback", LaunchConfiguration("output_io_feedback")),
            ("output/odometry_feedback", LaunchConfiguration("output_odometry_feedback")),
            ("output/battery_information_feedback", LaunchConfiguration("output_battery_information_feedback")),
            ("output/battery_flag_feedback", LaunchConfiguration("output_battery_flag_feedback")),
            ("output/wheel_encoder_feedback", LaunchConfiguration("output_wheel_encoder_feedback")),
            ("output/diagnostic_feedback", LaunchConfiguration("output_diagnostic_feedback")),
            ("output/is_online", LaunchConfiguration("output_is_online")),
            ("input/ctrl_command", LaunchConfiguration("input_ctrl_command")),
            ("input/io_command", LaunchConfiguration("input_io_command")),
        ],
        parameters=[],
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
        composable_node_descriptions=[yhs_can_control_component],
        target_container=targetContainer
    )
    return [
        container,
        composableLoader
    ]