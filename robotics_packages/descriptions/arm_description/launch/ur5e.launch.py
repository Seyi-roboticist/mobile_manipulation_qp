from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("ur_type", default_value="ur5e"),
        DeclareLaunchArgument("robot_ip", default_value="172.22.22.2"),
        DeclareLaunchArgument("safety_limits", default_value="true"),
        DeclareLaunchArgument("safety_k_position", default_value="20"),
        DeclareLaunchArgument("runtime_config_package", default_value="ur5e_cartesian_control"),
        DeclareLaunchArgument("controllers_file", default_value="ur5e_controllers.yaml"),
        DeclareLaunchArgument("description_package", default_value="ur5e_cartesian_control"),
        DeclareLaunchArgument("description_file", default_value="ur5e_with_sensors.xacro"),
        DeclareLaunchArgument("tf_prefix", default_value=""),
        DeclareLaunchArgument("fake_sensor_commands", default_value="false"),
        DeclareLaunchArgument("controller_spawner_timeout", default_value="10"),
        DeclareLaunchArgument("initial_joint_controller", default_value="scaled_joint_trajectory_controller"),
        DeclareLaunchArgument("activate_joint_controller", default_value="true"),
        DeclareLaunchArgument("launch_rviz", default_value="false"),
        DeclareLaunchArgument("headless_mode", default_value="true"),
        DeclareLaunchArgument("launch_dashboard_client", default_value="false"),
        DeclareLaunchArgument("use_tool_communication", default_value="false"),
        DeclareLaunchArgument("tool_parity", default_value="0"),
        DeclareLaunchArgument("tool_baud_rate", default_value="115200"),
        DeclareLaunchArgument("tool_stop_bits", default_value="1"),
        DeclareLaunchArgument("tool_rx_idle_chars", default_value="1.5"),
        DeclareLaunchArgument("tool_tx_idle_chars", default_value="3.5"),
        DeclareLaunchArgument("tool_device_name", default_value="/tmp/ttyUR"),
        DeclareLaunchArgument("tool_tcp_port", default_value="54321"),
        DeclareLaunchArgument("tool_voltage", default_value="0"),
        DeclareLaunchArgument("reverse_ip", default_value="172.22.22.10"),
        DeclareLaunchArgument("script_command_port", default_value="50004"),
        DeclareLaunchArgument("reverse_port", default_value="50001"),
        DeclareLaunchArgument("script_sender_port", default_value="50002"),
        DeclareLaunchArgument("trajectory_port", default_value="50003"),
        DeclareLaunchArgument("sim_ignition", default_value="false"),
        DeclareLaunchArgument("sim_isaac", default_value="false"),
        DeclareLaunchArgument("com_port", default_value="/dev/ttyUSB0"),
     
    ]

    # Launch Configurations 
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_k_position = LaunchConfiguration("safety_k_position")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    joint_limits_parameters_file = LaunchConfiguration("joint_limits_parameters_file")
    kinematics_parameters_file = LaunchConfiguration("kinematics_parameters_file")
    physical_parameters_file = LaunchConfiguration("physical_parameters_file")
    visual_parameters_file = LaunchConfiguration("visual_parameters_file")
    tf_prefix = LaunchConfiguration("tf_prefix")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    headless_mode = LaunchConfiguration("headless_mode")
    launch_dashboard_client = LaunchConfiguration("launch_dashboard_client")
    use_tool_communication = LaunchConfiguration("use_tool_communication")
    tool_parity = LaunchConfiguration("tool_parity")
    tool_baud_rate = LaunchConfiguration("tool_baud_rate")
    tool_stop_bits = LaunchConfiguration("tool_stop_bits")
    tool_rx_idle_chars = LaunchConfiguration("tool_rx_idle_chars")
    tool_tx_idle_chars = LaunchConfiguration("tool_tx_idle_chars")
    tool_device_name = LaunchConfiguration("tool_device_name")
    tool_tcp_port = LaunchConfiguration("tool_tcp_port")
    tool_voltage = LaunchConfiguration("tool_voltage")
    reverse_ip = LaunchConfiguration("reverse_ip")
    script_command_port = LaunchConfiguration("script_command_port")
    reverse_port = LaunchConfiguration("reverse_port")
    script_sender_port = LaunchConfiguration("script_sender_port")
    trajectory_port = LaunchConfiguration("trajectory_port")
    sim_ignition = LaunchConfiguration("sim_ignition")
    sim_isaac = LaunchConfiguration("sim_isaac")
    com_port = LaunchConfiguration("com_port")


    # Paths 
    urdf_path = FindPackageShare("arm_description").find("arm_description")
    ur_description_path = FindPackageShare("ur_description").find("ur_description")
    xacro_file = PathJoinSubstitution([urdf_path, "urdf", "ur5e.urdf.xacro"])

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ", xacro_file,
        " sim_ignition:=", sim_ignition,
        " ur_type:=", ur_type,
        # " joint_limits_parameters_file:=", joint_limits_parameters_file,
        # " kinematics_parameters_file:=", kinematics_parameters_file,
        # " physical_parameters_file:=", physical_parameters_file,
        # " visual_parameters_file:=", visual_parameters_file,
        " tf_prefix:=", tf_prefix
    ])
    robot_description = {"robot_description": robot_description_content}

     # Base Launch 
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("ur_robot_driver"), "/launch/ur_control.launch.py"]),
        launch_arguments={
            "ur_type": ur_type, 
            "robot_ip": robot_ip,
            "safety_limits": safety_limits,
            "safety_k_position": safety_k_position,
            "runtime_config_package": runtime_config_package,
            # "controllers_file": controllers_file,
            "description_package": description_package,
            "description_file": description_file,
            # "kinematics_parameters_file": kinematics_parameters_file,
            # "joint_limits_parameters_file": joint_limits_parameters_file,
            # "physical_parameters_file": physical_parameters_file,
            # "visual_parameters_file": visual_parameters_file,
            "tf_prefix": tf_prefix,
            # "use_fake_hardware": use_fake,
            "fake_sensor_commands": fake_sensor_commands,
            "controller_spawner_timeout": controller_spawner_timeout,
            "initial_joint_controller": initial_joint_controller,
            "activate_joint_controller": activate_joint_controller,
            "headless_mode": headless_mode,
            "launch_rviz": launch_rviz,
            "launch_dashboard_client": launch_dashboard_client,
            "use_tool_communication": use_tool_communication,
            "tool_parity": tool_parity,
            "tool_baud_rate": tool_baud_rate,
            "tool_stop_bits": tool_stop_bits,
            "tool_rx_idle_chars": tool_rx_idle_chars, 
            "tool_tx_idle_chars": tool_tx_idle_chars, 
            "tool_device_name": tool_device_name,
            "tool_tcp_port": tool_tcp_port,
            "tool_voltage": tool_voltage,
            "reverse_ip": reverse_ip,
            "script_command_port": script_command_port,
            "reverse_port": reverse_port,
            "script_sender_port": script_sender_port,
            "trajectory_port": trajectory_port,
            # "script_filename": script_filename
        }.items(),
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    nodes_to_start = [robot_state_publisher_node]

    return LaunchDescription( declared_arguments + nodes_to_start)