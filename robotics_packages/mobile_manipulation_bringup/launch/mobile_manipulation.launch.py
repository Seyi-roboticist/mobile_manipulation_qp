from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    declared_arguments = [
        DeclareLaunchArgument("ur_type", default_value="ur5e"),
        DeclareLaunchArgument("use_gazebo", default_value="false"),
        DeclareLaunchArgument("sim_ignition", default_value="false")
    ]

    # Set Gazebo resource path for model discovery (idea gotten from Turtlebot Navigation JHU project)
    set_gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
            ':',
            PathJoinSubstitution([FindPackageShare('husky_description'), '..']),
            ':',
            PathJoinSubstitution([FindPackageShare('robotiq_description'), '..'])
        ]
    )

    # Launch Configuration 
    ur_type = LaunchConfiguration("ur_type")
    use_gazebo = LaunchConfiguration("use_gazebo")
    sim_ignition = LaunchConfiguration("sim_ignition")

    # Combined robot description
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([
            FindPackageShare("mobile_manipulation_bringup"), 
            "urdf", 
            "mobile_manipulation.urdf.xacro"
        ]), 
        " ur_type:=", ur_type,
        " sim_ignition:=", sim_ignition,
    ])
    
    robot_description = {"robot_description": robot_description_content}
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )
    
    # Joint state publisher GUI
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )
    
    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", PathJoinSubstitution([
            FindPackageShare("mobile_manipulation_bringup"), "rviz", "mobile_robot.rviz"
        ])],
    )

    ######################################## GAZEBO #############################################
    # Gazebo Launch (ign)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ]), 
        launch_arguments={"gz_args": "empty.sdf"}.items(),
        condition=IfCondition(use_gazebo)
    )

    # Robot Spawn
    robot_spawn = Node(
        package="ros_gz_sim",
        executable="create", 
        arguments=["-string", robot_description_content, "-name", "mobile_manipulation_robot"],
        output="screen", 
        parameters=[{"use_sim_time": use_gazebo}], 
        condition=IfCondition(use_gazebo)
    )

    # Gazebo Bridge
    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]"],
        output="screen",
        condition=IfCondition(use_gazebo),
    )
    ################################################################################################

    nodes_to_start = [set_gazebo_resource_path,
                      robot_state_publisher_node, 
                      joint_state_publisher_gui_node, 
                      gazebo_launch, 
                      robot_spawn,
                      gz_sim_bridge, 
                      rviz_node]

    
    return LaunchDescription(declared_arguments + nodes_to_start)