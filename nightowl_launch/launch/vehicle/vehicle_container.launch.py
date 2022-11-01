from launch_ros.actions import ComposableNodeContainer
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch

def generate_launch_description():
    launchArguments = []

    def addLaunchArguments(name: str, defaultValue=None):
        launchArguments.append(DeclareLaunchArgument(name, default_value=defaultValue))

    addLaunchArguments("use_intra_process", "false")
    addLaunchArguments("container_name", "vehicle_container")
    addLaunchArguments("use_vehicle_container", "true")

    return launch.LaunchDescription(
        launchArguments
        + [OpaqueFunction(function=launchSetup)]
    )


def launchSetup(context, *args, **kwargs):
    # 如果是单线程模式就直接加载容器
    container = ComposableNodeContainer(
        name="vehicle_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[],
        condition=IfCondition(LaunchConfiguration("use_vehicle_container")),
        output="screen",
    )
    yhs_can_control_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("yhs_can_control"), "/launch/yhs_can_control.launch.py"]
        ),
        launch_arguments=[
            ("use_intra_process", LaunchConfiguration("use_intra_process")),
            ("container_name", "/vehicle/vehicle_container") if IfCondition(LaunchConfiguration("use_vehicle_container")).evaluate(context) else (),
            ("use_vehicle_container", LaunchConfiguration("use_vehicle_container"))
        ],
    )
    yhs_autoware_interface_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("yhs_autoware_interface"), "/launch/yhs_autoware_interface.launch.py"]
        ),
        launch_arguments=[
            ("use_intra_process", LaunchConfiguration("use_intra_process")),
            ("container_name", "/vehicle/vehicle_container") if IfCondition(LaunchConfiguration("use_vehicle_container")).evaluate(context) else (),
            ("use_vehicle_container", LaunchConfiguration("use_vehicle_container"))
        ],
    )
    return [
        container,
        yhs_can_control_node,
        yhs_autoware_interface_node
    ]