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
    addLaunchArguments("container_name", "camera_container")
    addLaunchArguments("use_camera_container", "False")

    addLaunchArguments("image_width", "1280")
    addLaunchArguments("image_height", "720")
    addLaunchArguments("camera_ip", "192.168.1.251")
    addLaunchArguments("base_frame", "front_stereo")
    addLaunchArguments("use_ros_timestamp", "True")
    addLaunchArguments("use_sdk_camera_info", "True")
    addLaunchArguments("left_camera_info_url", "")
    addLaunchArguments("right_camera_info_url", "")
    addLaunchArguments("image_rate", "12.5")
    addLaunchArguments("imu_acceleration_range", "4")
    addLaunchArguments("imu_rotation_range", "500")
    addLaunchArguments("imu_rate", "100")

    addLaunchArguments("output_point_cloud", "/point_cloud")
    addLaunchArguments("output_imu", "/imu")
    addLaunchArguments("output_camera_info", "/camera_info")
    addLaunchArguments("output_left_camera", "/left_camera")
    addLaunchArguments("output_right_camera", "/right_camera")

    return launch.LaunchDescription(
        launchArguments
        + [OpaqueFunction(function=launchSetup)]
    )


def launchSetup(context, *args, **kwargs):
    smarter_eye_camera_component = ComposableNode(
        package="smarter_eye_camera",
        plugin="smarter_eye_camera::SmarterEyeCameraNode",
        name="smarter_eye_camera",
        namespace="",
        remappings=[
            ("output/point_cloud", LaunchConfiguration("output_point_cloud")),
            ("output/imu", LaunchConfiguration("output_imu")),
            ("output/camera_info", LaunchConfiguration("output_camera_info")),

            ("output/left_camera", [LaunchConfiguration("output_left_camera")]),
            ("output/left_camera/compressed", [LaunchConfiguration("output_left_camera"), "/compressed"]),
            ("output/left_camera/compressedDepth", [LaunchConfiguration("output_left_camera"), "/compressedDepth"]),
            ("output/left_camera/theora", [LaunchConfiguration("output_left_camera"), "/theora"]),
            
            ("output/right_camera", [LaunchConfiguration("output_right_camera")]),
            ("output/right_camera/compressed", [LaunchConfiguration("output_right_camera"), "/compressed"]),
            ("output/right_camera/compressedDepth", [LaunchConfiguration("output_right_camera"), "/compressedDepth"]),
            ("output/right_camera/theora", [LaunchConfiguration("output_right_camera"), "/theora"]),
        ],
        parameters=[{
            "image_width": LaunchConfiguration("image_width"),
            "image_height": LaunchConfiguration("image_height"),
            "camera_ip": LaunchConfiguration("camera_ip"),
            "base_frame": LaunchConfiguration("base_frame"),
            "use_ros_timestamp": LaunchConfiguration("use_ros_timestamp"),
            "use_sdk_camera_info": LaunchConfiguration("use_sdk_camera_info"),
            "left_camera_info_url": LaunchConfiguration("left_camera_info_url"),
            "right_camera_info_url": LaunchConfiguration("right_camera_info_url"),
            "image_rate": LaunchConfiguration("image_rate"),
            "imu_acceleration_range": LaunchConfiguration("imu_acceleration_range"),
            "imu_rotation_range": LaunchConfiguration("imu_rotation_range"),
            "imu_rate": LaunchConfiguration("imu_rate")
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
        condition=UnlessCondition(LaunchConfiguration("use_camera_container")),
        output="screen",
    )
    targetContainer = (
        container
        if UnlessCondition(LaunchConfiguration("use_camera_container")).evaluate(context)
        else LaunchConfiguration("container_name")
    )
    composableLoader = LoadComposableNodes(
        composable_node_descriptions=[smarter_eye_camera_component],
        target_container=targetContainer
    )
    return [
        container,
        composableLoader
    ]