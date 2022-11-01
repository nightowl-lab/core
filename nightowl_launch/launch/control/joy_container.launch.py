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
    addLaunchArguments("container_name", "joy_container")
    addLaunchArguments("use_joy_container", "true")

    return launch.LaunchDescription(
        launchArguments
        + [OpaqueFunction(function=launchSetup)]
    )


def launchSetup(context, *args, **kwargs):
    # 如果是单线程模式就直接加载容器
    container = ComposableNodeContainer(
        name="joy_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[],
        condition=IfCondition(LaunchConfiguration("use_joy_container")),
        output="screen",
    )
    joy_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("joy2"), "/launch/joy.launch.py"]
        ),
        launch_arguments=[
            ("use_intra_process", LaunchConfiguration("use_intra_process")),
            ("container_name", "/control/joy_container") if IfCondition(LaunchConfiguration("use_joy_container")).evaluate(context) else (),
            ("use_joy_container", LaunchConfiguration("use_joy_container"))
        ],
    )
    baidu_joy_controller_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("baidu_joy_controller"), "/launch/baidu_joy_controller.launch.py"]
        ),
        launch_arguments=[
            ("use_intra_process", LaunchConfiguration("use_intra_process")),
            ("container_name", "/control/joy_container") if IfCondition(LaunchConfiguration("use_joy_container")).evaluate(context) else (),
            ("use_joy_container", LaunchConfiguration("use_joy_container"))
        ],
    )
    return [
        container,
        joy_node,
        baidu_joy_controller_node
    ]