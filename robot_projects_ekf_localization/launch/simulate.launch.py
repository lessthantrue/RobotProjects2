from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os.path

def generate_launch_description():
    ld = LaunchDescription()

    sim_node = Node(
        package="robot_projects_simulator",
        executable="ekf_sim_world"
    )

    ekf_node = Node(
        package="robot_projects_ekf_localization",
        executable="ekf_localization"
    )

    vis_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(get_package_share_directory("robot_projects_ekf_localization"), "config", "ekf_sim_world.rviz")]
    )

    ld.add_action(sim_node)
    ld.add_action(ekf_node)
    ld.add_action(vis_node)

    return ld