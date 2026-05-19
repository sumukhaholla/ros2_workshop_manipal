import os
from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    is_sim = LaunchConfiguration('is_sim')
    
    is_sim_arg = DeclareLaunchArgument(
        'is_sim',
        default_value='True'
    )

    moveit_config = (
        MoveItConfigsBuilder("Model", package_name="ros2_moveit_files")
        .robot_description(file_path=os.path.join(
            get_package_share_directory("ros2_description"),
            "urdf",
            "medi_assist.urdf.xacro"
            )
        )
        .robot_description_semantic(file_path="config/Model.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl"]) # ADDING THIS TO MAKE SURE THAT OMPL MOTION PLANNER IS SELECTED AS DEFAULT
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), 
                    {'use_sim_time': is_sim},
                    {'publish_robot_description_semantic': True},
                    {'warehouse_plugin': 'warehouse_ros_sqlite::DatabaseConnection',
                     'warehouse_host': '/home/your_user/.ros/moveit2_warehouse.db',
                     'warehouse_port': 0,
                    }],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # RViz
    rviz_config = os.path.join(
        get_package_share_directory("ros2_moveit_files"),
            "world",
            "moveit.rviz",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {'use_sim_time': is_sim},
        ],
    )

    planner_node = Node(
        package="ros2_cpp",
        executable="moveit_planner",
        output="screen",
        parameters=[
            moveit_config.robot_description,          
            moveit_config.robot_description_semantic,  
            moveit_config.robot_description_kinematics,
            {'use_sim_time': is_sim},
        ],
    )

    return LaunchDescription(
        [
            is_sim_arg,
            move_group_node, 
            rviz_node,
            # planner_node
        ]
    )