from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "apriltag_family",
            description="Apriltag  family to be detected by camera.",
            default_value="tag36h11",
        )
    )

    apriltag_family = LaunchConfiguration("apriltag_family")

    moveit_config = MoveItConfigsBuilder(
        "gen3", package_name="kinova_gen3_7dof_robotiq_2f_85_moveit_config"
    ).to_dict()

    # MTC node
    pick_place_demo = Node(
        package="arm_mobility",
        executable="mtc_node",
        output="screen",
        parameters=[
            moveit_config,
        ],
    )

    # object detection node
    object_detector = Node(
        package="robot_vision_apriltag",
        executable="image_processor",
        output="log",
        parameters=[{"tag_family": apriltag_family}]
    )

    return LaunchDescription(declared_arguments + [object_detector, pick_place_demo])