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
    addLaunchArguments("container_name", "starneto_control_container")
    addLaunchArguments("use_ins_container", "False")

    addLaunchArguments("node_rate", "1")
    addLaunchArguments("timesync_flag", "true")
    addLaunchArguments("port", "/dev/ttyACM0")
    addLaunchArguments("baudrate", "115200")
    addLaunchArguments("gnss_frame", "ins_link")
    addLaunchArguments("imu_frame", "ins_link")

    addLaunchArguments("output_topic_gpfpd", "gnss/raw_data")
    addLaunchArguments("output_topic_gtimu", "imu/raw_data")
    addLaunchArguments("output_topic_gpgga", "gpgga/raw_data")

    return launch.LaunchDescription(
        launchArguments
        + [OpaqueFunction(function=launchSetup)]
    )


def launchSetup(context, *args, **kwargs):
    starneto_control_component = ComposableNode(
        package="starneto_control",
        plugin="starneto_control::StarnetoControlNode",
        name="starneto_control",
        namespace="",
        remappings=[
            ("output/topic_gpfpd", [f"/sensing/ins/starneto/", LaunchConfiguration("output_topic_gpfpd")]),
            ("output/topic_gtimu", [f"/sensing/ins/starneto/", LaunchConfiguration("output_topic_gtimu")]),
            ("output/topic_gpgga", [f"/sensing/ins/starneto/", LaunchConfiguration("output_topic_gpgga")])
        ],
        parameters=[{
            "node_rate": LaunchConfiguration("node_rate"),
            "timesync_flag": LaunchConfiguration("timesync_flag"),
            "port": LaunchConfiguration("port"),
            "baudrate": LaunchConfiguration("baudrate"),
            "gnss_frame": LaunchConfiguration("gnss_frame"),
            "imu_frame": LaunchConfiguration("imu_frame")
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
        output="screen",
    )
    targetContainer = (
        container
        if UnlessCondition(LaunchConfiguration("use_ins_container")).evaluate(context)
        else LaunchConfiguration("container_name")
    )
    composableLoader = LoadComposableNodes(
        composable_node_descriptions=[starneto_control_component],
        target_container=targetContainer
    )
    return [
        container,
        composableLoader
    ]