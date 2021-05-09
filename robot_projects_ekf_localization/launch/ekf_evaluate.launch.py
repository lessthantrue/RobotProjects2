from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
import sys
import pathlib
import launch
import os

def generate_launch_description():
    ld = LaunchDescription()

    arg = DeclareLaunchArgument("bagfile")

    # this is a bad way to get the launch file
    bag_file = sys.argv[-1]
    bag_process = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', launch.substitutions.LaunchConfiguration("bagfile")]
    )

    ekf_node = Node(
        package="robot_projects_ekf_localization",
        executable="ekf_localization",
        output={"both": "screen"}
    )

    vis_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(get_package_share_directory("robot_projects_ekf_localization"), "config", "ekf_sim_world.rviz")],
        output={"both": "screen"}
    )

    # get params file for eval node
    parameters_file_name = "ekf_evaluate_params.yaml"
    parameters_file_path = str(pathlib.Path(__file__).parents[1]) # get current path and go one level up
    parameters_file_path += '/config/' + parameters_file_name
    print(parameters_file_path)
    eval_node = Node(
        package="robot_projects_ekf_localization",
        executable="ekf_evaluation",
        parameters=[
            parameters_file_path
        ]
    )

    ld.add_action(ekf_node)
    ld.add_action(eval_node)
    ld.add_action(bag_process)
    ld.add_entity(vis_node)
    ld.add_entity(arg)

    return ld