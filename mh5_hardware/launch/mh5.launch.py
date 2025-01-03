from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, Command, \
    LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    arguments = []

    arguments.append(
        DeclareLaunchArgument(
            name="use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring commands.",
            choices=["true", "false"],
        )
    )

    # controller manager
    robot_controllers_yaml = PathJoinSubstitution([
        FindPackageShare("mh5_hardware"),
        "config",
        "robot_controllers.yaml", ]
    )
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers_yaml],
        output="both",
        remappings=[
            ("~/robot_description", "robot_description"),
        ],
    )
    # robot state publisher
    urdf_path = PathJoinSubstitution([
        FindPackageShare("mh5_hardware"),
        "urdf",
        "mh5_robot.urdf.xacro", ]
    )
    robot_description = Command([
        "xacro ", urdf_path, " ",
        "use_mock_hardware:=", LaunchConfiguration("use_mock_hardware"), ]
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description}],
    )
    # controllers
    controllers = [
        "joint_state_broadcaster",
        "joint_status_broadcaster",
    ]

    remap = {
        "joint_status_broadcaster": "-r dynamic_joint_states:=joint_statuses"
    }

    controller_nodes = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                controller_name,
                "--param-file", robot_controllers_yaml,
                "--controller-ros-args", remap.get(controller_name,"")
            ],
        )
        for controller_name in controllers
    ]

    nodes = [
        controller_manager_node,
        robot_state_pub_node,
    ] + controller_nodes

    return LaunchDescription(arguments + nodes)
