from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(get_package_share_directory("ros2_description"),"urdf","medi_assist.urdf.xacro"),
        description="Absolute path to the robot URDF file"
    )

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]))

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable = "robot_state_publisher",
        parameters=[{"robot_description":robot_description}]
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    # initial_joint_state = ExecuteProcess(
    #     cmd=[
    #         'ros2', 'topic', 'pub', '--once', '/joint_states',
    #         'sensor_msgs/msg/JointState',
    #         "{name: ['Stand_Base_72_to_base_link_1', 'Ceiling_Carriage_53_to_Ceiling_Base_Link'], "
    #         "position: [0.008, 0.05]}"
    #     ],
    #     output='screen'
    # )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(get_package_share_directory("ros2_description"), "rviz", "display.rviz")]
    )
    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node # Later disable this, once moveit.launch.py is running
    ])