from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # Launch settings for UR5e robot and environment
    # models_dir = os.path.join(get_package_share_directory("ros2_description"), "worlds", "outside_wall")
    # outside_walls = os.path.join(models_dir, "model.sdf")
    # position = [0.0, 0.0, 0.0]

    is_sim_arg = DeclareLaunchArgument(
        'is_sim',
        default_value='true'
    )

    # World file declaration
    world_arg = DeclareLaunchArgument(
        name='world',
        default_value=os.path.join(get_package_share_directory("ros2_description"), "worlds", "medi_assistV02.world"),
        # default_value=os.path.join(get_package_share_directory("ros2_description"), "worlds", "ur5e_world02.world"),
        description="Path to the Gazebo world file"
    )

    # Robot model file declaration
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(get_package_share_directory("ros2_description"), "urdf", "medi_assist.urdf.xacro"),
        description="Absolute path to the robot URDF file"
    )

    # Environment variable for Gazebo model path
    # Package paths
    ros2_desc_share = get_package_share_directory("ros2_description")

    # Gazebo model path
    gazebo_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=":".join([
            os.path.join(ros2_desc_share, "models"),
            os.path.join(get_package_prefix("ros2_description"), "share"),
            os.path.expanduser("~/.gazebo/models")
        ])
    )

    # Gazebo resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name="GAZEBO_RESOURCE_PATH",
        value=":".join([
            "/usr/share/gazebo-11",
            ros2_desc_share
        ])
    )

    # Gazebo plugin path
    gazebo_plugin_path = SetEnvironmentVariable(
        name="GAZEBO_PLUGIN_PATH",
        value=":".join([
            "/opt/ros/humble/lib",
            os.path.normpath(
                os.path.join(
                    ros2_desc_share,
                    "..",
                    "..",
                    "lib"
                )
            )
        ])
    )

    # Robot description configuration
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]))

    # Robot State Publisher node
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description},
                    {"use_sim_time": LaunchConfiguration("is_sim")}]
    )

    # Gazebo server
    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py")),
        launch_arguments={'world': LaunchConfiguration('world'), 'verbose': 'true'}.items()
    )

    # Gazebo client
    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzclient.launch.py"))
    )

    # Spawn robot
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "Model", "-topic", "robot_description"],
        output="screen"
    )

    # Spawn outside wall model
    # spawn_wall = Node(
    #     package="gazebo_ros",
    #     executable="spawn_entity.py",
    #     arguments=["-entity", "outside_wall", "-file", outside_walls, "-x", str(position[0]), "-y", str(position[1]), "-z", str(position[2])],
    #     output="screen"
    # )

    return LaunchDescription([
        is_sim_arg,
        world_arg,
        gazebo_model_path,
        gazebo_resource_path,
        gazebo_plugin_path,
        model_arg,
        robot_state_publisher,
        start_gazebo_server,
        start_gazebo_client,
        # spawn_wall,  # Uncomment if you need to spawn the wall model
        spawn_robot   
    ])