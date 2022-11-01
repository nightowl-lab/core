from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
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

    addLaunchArguments("use_cropped_square_output", "true")
    addLaunchArguments("left_image_cropped_square_x_offset", "280")
    addLaunchArguments("camera_name", "front")
    addLaunchArguments("camera_ip", "192.168.1.251")
    addLaunchArguments("base_frame", "front_stereo")
    addLaunchArguments("use_intra_process", "false")
    addLaunchArguments("container_name", [LaunchConfiguration("camera_name"), "_camera_container"])
    addLaunchArguments("use_camera_container", "true")

    return launch.LaunchDescription(
        launchArguments
        + [OpaqueFunction(function=launchSetup)]
    )


def launchSetup(context, *args, **kwargs):
    crop_square_node = ComposableNode(
        package="image_proc",
        plugin="image_proc::CropDecimateNode",
        name="crop_square_node",
        namespace="",
        remappings=[
            ("in/image_raw", [f"/sensing/camera/", LaunchConfiguration("camera_name"), "/left_image"]),
            ("in/camera_info", [f"/sensing/camera/", LaunchConfiguration("camera_name"), "/camera_info"]),
            ("out/image_raw", [f"/sensing/camera/", LaunchConfiguration("camera_name"), "/left_image/cropped_square"]),
        ],
        parameters=[{
            "offset_x": LaunchConfiguration("left_image_cropped_square_x_offset"),
            "height": 720,
            "width": 720
        }],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )
    # 如果是单线程模式就直接加载容器
    container = ComposableNodeContainer(
        name=LaunchConfiguration("container_name"),
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[crop_square_node] if IfCondition(LaunchConfiguration("use_cropped_square_output")).evaluate(context) else [],
        condition=IfCondition(LaunchConfiguration("use_camera_container")),
        output="screen",
    )
    smarter_eye_camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("smarter_eye_camera"), "/launch/smarter_eye_camera.launch.py"]
        ),
        launch_arguments=[
            ("output_point_cloud", [f"/sensing/camera/", LaunchConfiguration("camera_name"), "/point_cloud"]),
            ("output_imu", [f"/sensing/camera/", LaunchConfiguration("camera_name"), "/imu"]),
            ("output_left_camera", [f"/sensing/camera/", LaunchConfiguration("camera_name"), "/left_image"]),
            ("output_right_camera", [f"/sensing/camera/", LaunchConfiguration("camera_name"), "/right_image"]),
            ("output_camera_info", [f"/sensing/camera/", LaunchConfiguration("camera_name"), "/camera_info"]),
            ("camera_ip", LaunchConfiguration("camera_ip")),
            ("use_intra_process", LaunchConfiguration("use_intra_process")),
            ("container_name", ["/sensing/camera/", LaunchConfiguration("container_name")]) if IfCondition(LaunchConfiguration("use_camera_container")).evaluate(context) else (),
            ("use_camera_container", LaunchConfiguration("use_camera_container")),
            ("base_frame", LaunchConfiguration("base_frame"))
        ],
    )
    return [
        container,
        smarter_eye_camera_node
    ]