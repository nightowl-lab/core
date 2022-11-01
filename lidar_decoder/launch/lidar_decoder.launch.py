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

    def addLaunchArguments(name: str, defaultValue=None, description=None):
        launchArguments.append(DeclareLaunchArgument(name, default_value=defaultValue, description=description))

    addLaunchArguments("launch_driver", "True", description="do launch driver")
    addLaunchArguments("base_frame", "base_link",description="base frame id")
    addLaunchArguments("input_frame", LaunchConfiguration("base_frame"), description="use for cropbox")
    addLaunchArguments("output_frame", LaunchConfiguration("base_frame"), description="use for cropbox")
    
    addLaunchArguments("lidar_channel_param_file", description="vertical angle in each channel")
    addLaunchArguments("lidar_data_param_file", description="basic configure of lidar")
    addLaunchArguments("vehicle_param_file", description="path to the file of vehicle info yaml")
    addLaunchArguments("vehicle_mirror_param_file", description="path to the file of vehicle mirror position yaml")

    addLaunchArguments("use_intra_process", "True", "use ROS2 component container communication")
    addLaunchArguments("container_name", "lslidar_container")
    addLaunchArguments("use_lidar_container", "False")
    addLaunchArguments("model", "LSC16")
    addLaunchArguments("time_synchronization", "false")

    addLaunchArguments("input_topic_msop", "lslidar/msop_packet")
    addLaunchArguments("input_topic_difop", "lslidar/difop_packet")
    addLaunchArguments("input_topic_sync", "lslidar/sync_header")

    addLaunchArguments("output_topic_pointCloud", "pointcloud_raw_ex")

    return launch.LaunchDescription(
        launchArguments
        + [OpaqueFunction(function=launchSetup)]
    )

def get_vehicle_info(context):
    path = LaunchConfiguration("vehicle_param_file").perform(context)
    with open(path, "r") as f:
        p = yaml.safe_load(f)["/**"]["ros__parameters"]
    p["vehicle_length"] = p["front_overhang"] + p["wheel_base"] + p["rear_overhang"]
    p["vehicle_width"] = p["wheel_tread"] + p["left_overhang"] + p["right_overhang"]
    p["min_longitudinal_offset"] = -p["rear_overhang"]
    p["max_longitudinal_offset"] = p["front_overhang"] + p["wheel_base"]
    p["min_lateral_offset"] = -(p["wheel_tread"] / 2.0 + p["right_overhang"])
    p["max_lateral_offset"] = p["wheel_tread"] / 2.0 + p["left_overhang"]
    p["min_height_offset"] = 0.0
    p["max_height_offset"] = p["vehicle_height"]
    return p


def get_vehicle_mirror_info(context):
    path = LaunchConfiguration("vehicle_mirror_param_file").perform(context)
    with open(path, "r") as f:
        p = yaml.safe_load(f)["/**"]["ros__parameters"]
    return p

def launchSetup(context, *args, **kwargs):
    def create_parameter_dict(*args):
        result = {}
        for x in args:
            result[x] = LaunchConfiguration(x)
        return result

    # 加载lidar配置文件
    lidar_channel_params_path = LaunchConfiguration("lidar_channel_param_file").perform(context)
    with open(lidar_channel_params_path, "r") as f:
        lidar_channel_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    lidar_data_params_path = LaunchConfiguration("lidar_data_param_file").perform(context)
    with open(lidar_data_params_path, "r") as f:
        lidar_data_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    nodes = []

    nodes.append(
        ComposableNode(
            package="lidar_decoder",
            plugin="lidar_decoder::LidarDecoder",
            name="lidar_decoder",
            namespace="",
            remappings=[
                ("input/msop_packet", [f"/sensing/lidar/top/", LaunchConfiguration("input_topic_msop")]),
                ("input/difop_packet",[f"/sensing/lidar/top/",  LaunchConfiguration("input_topic_difop")]),
                ("input/sync_header", [f"/sensing/lidar/top/", LaunchConfiguration("input_topic_sync")]),
                ("output/point_cloud", "pointcloud_raw_ex"),
            ],
            parameters=[
                lidar_channel_param,
                lidar_data_param,
                {
                    "time_synchronization": LaunchConfiguration("time_synchronization"),
                }
            ],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    cropbox_parameters = create_parameter_dict("input_frame", "output_frame")
    cropbox_parameters["negative"] = True

    vehicle_info = get_vehicle_info(context)
    cropbox_parameters["min_x"] = vehicle_info["min_longitudinal_offset"]
    cropbox_parameters["max_x"] = vehicle_info["max_longitudinal_offset"]
    cropbox_parameters["min_y"] = vehicle_info["min_lateral_offset"]
    cropbox_parameters["max_y"] = vehicle_info["max_lateral_offset"]
    cropbox_parameters["min_z"] = vehicle_info["min_height_offset"]
    cropbox_parameters["max_z"] = vehicle_info["max_height_offset"]

    nodes.append(
        ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::CropBoxFilterComponent",
            name="crop_box_filter_self",
            remappings=[
                ("input", "pointcloud_raw_ex"),
                ("output", "self_cropped/pointcloud_ex"),
            ],
            parameters=[cropbox_parameters],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    # mirror_info = get_vehicle_mirror_info(context)
    # cropbox_parameters["min_x"] = mirror_info["min_longitudinal_offset"]
    # cropbox_parameters["max_x"] = mirror_info["max_longitudinal_offset"]
    # cropbox_parameters["min_y"] = mirror_info["min_lateral_offset"]
    # cropbox_parameters["max_y"] = mirror_info["max_lateral_offset"]
    # cropbox_parameters["min_z"] = mirror_info["min_height_offset"]
    # cropbox_parameters["max_z"] = mirror_info["max_height_offset"]

    # nodes.append(
    #     ComposableNode(
    #         package="pointcloud_preprocessor",
    #         plugin="pointcloud_preprocessor::CropBoxFilterComponent",
    #         name="crop_box_filter_mirror",
    #         remappings=[
    #             ("input", "self_cropped/pointcloud_ex"),
    #             ("output", "mirror_cropped/pointcloud_ex"),
    #         ],
    #         parameters=[cropbox_parameters],
    #         extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    #     )
    # )

    nodes.append(
        ComposableNode(
            package="velodyne_pointcloud",
            plugin="velodyne_pointcloud::Interpolate",
            name="velodyne_interpolate_node",
            remappings=[
                ("velodyne_points_ex", "self_cropped/pointcloud_ex"),
                ("velodyne_points_interpolate", "rectified/pointcloud"),
                ("velodyne_points_interpolate_ex", "rectified/pointcloud_ex"),
            ],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    nodes.append(
        ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::RingOutlierFilterComponent",
            name="ring_outlier_filter",
            remappings=[
                ("input", "rectified/pointcloud_ex"),
                ("output", "outlier_filtered/pointcloud"),
            ],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )

    # 如果是单线程模式就直接加载容器
    container = ComposableNodeContainer(
        name=LaunchConfiguration("container_name"),
        namespace="pointcloud_preprocessor",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[],
        condition=UnlessCondition(LaunchConfiguration("use_lidar_container")),
        output="screen",
    )
    targetContainer = (
        container
        if UnlessCondition(LaunchConfiguration("use_lidar_container")).evaluate(context)
        else LaunchConfiguration("container_name")
    )
    composableLoader = LoadComposableNodes(
        composable_node_descriptions=nodes,
        target_container=targetContainer
    )
    return [
        container,
        composableLoader
    ]