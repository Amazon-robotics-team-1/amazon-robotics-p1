from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "gen3", package_name="kinova_gen3_6dof_robotiq_2f_85_moveit_config"
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

    return LaunchDescription([pick_place_demo])