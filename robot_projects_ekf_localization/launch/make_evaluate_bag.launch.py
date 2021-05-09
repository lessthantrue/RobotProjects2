from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os.path

def generate_launch_description():
    ld = LaunchDescription()

    sim_node = Node(
        package="robot_projects_simulator",
        executable="ekf_sim_world",
        output={"both": "screen"}
    )

    vis_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(get_package_share_directory("robot_projects_ekf_localization"), "config", "ekf_sim_world.rviz")],
        output={"both": "screen"}
    )

    test_act_node = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        output='screen',
        prefix = 'xterm -e'
    )

    topics = ['/cmd_vel', '/pt_sensor/reading', '/imu/reading', '/pose_sensor/reading', 'tf']
    bag_process = ExecuteProcess(
        cmd=['ros2', 'bag', 'record'] + topics
    )

    ld.add_action(sim_node)
    ld.add_action(vis_node)
    ld.add_action(test_act_node)
    ld.add_action(bag_process)

    return ld