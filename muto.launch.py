from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import socket
import uuid


def generate_launch_description():
    hostname = socket.gethostname()
    # Arguments
    muto_namespace_arg = DeclareLaunchArgument("muto_namespace", default_value="muto")
    vehicle_id_namespace_arg = DeclareLaunchArgument(
        "vehicle_id_namespace",
        default_value="org.eclipse.muto.sandbox",
        description="Vehicle ID namespace",
    )
    vehicle_id_arg = DeclareLaunchArgument(
        "vehicle_id", default_value=f"{hostname}", description="Edge Device ID"
    )

    # Files
    muto_params = "./config/muto.yaml"

    # Agent
    node_agent = Node(
        namespace=LaunchConfiguration("muto_namespace"),
        name=f"agent_{hostname}",
        package="agent",
        executable="muto_agent",
        output="screen",
        parameters=[
            muto_params,
            {"namespace": LaunchConfiguration("vehicle_id_namespace")},
            {"name": LaunchConfiguration("vehicle_id")},
        ],
    )

    node_mqtt_gateway = Node(
        namespace=LaunchConfiguration("muto_namespace"),
        name=f"gateway_{hostname}",
        package="agent",
        executable="mqtt",
        output="screen",
        parameters=[
            muto_params,
            {"namespace": LaunchConfiguration("vehicle_id_namespace")},
            {"name": LaunchConfiguration("vehicle_id")},
        ],
    )

    node_commands = Node(
        namespace=LaunchConfiguration("muto_namespace"),
        name=f"commands_plugin_{hostname}",
        package="agent",
        executable="commands",
        output="screen",
        parameters=[
            muto_params,
            {"namespace": LaunchConfiguration("vehicle_id_namespace")},
            {"name": LaunchConfiguration("vehicle_id")},
        ],
    )

    # Core
    node_twin = Node(
        namespace=LaunchConfiguration("muto_namespace"),
        name=f"core_twin_{hostname}",
        package="core",
        executable="twin",
        output="screen",
        parameters=[
            muto_params,
            {"namespace": LaunchConfiguration("vehicle_id_namespace")},
            {"name": LaunchConfiguration("vehicle_id")},
        ],
    )

    # Composer
    node_composer = Node(
        name=f"composer_{hostname}",
        namespace=LaunchConfiguration("muto_namespace"),
        package="composer",
        executable="muto_composer",
        output="screen",
        parameters=[
            muto_params,
            {"namespace": LaunchConfiguration("vehicle_id_namespace")},
            {"name": LaunchConfiguration("vehicle_id")},
        ],
    )

    node_compose_plugin = Node(
        name=f"compose_plugin_{hostname}",
        namespace=LaunchConfiguration("muto_namespace"),
        package="composer",
        executable="compose_plugin",
        output="screen",
        parameters=[
            muto_params,
            {"namespace": LaunchConfiguration("vehicle_id_namespace")},
            {"name": LaunchConfiguration("vehicle_id")},
        ],
    )

    node_native_plugin = Node(
        name=f"native_plugin_{hostname}",
        namespace=LaunchConfiguration("muto_namespace"),
        package="composer",
        executable="native_plugin",
        output="screen",
        parameters=[
            muto_params,
            {"namespace": LaunchConfiguration("vehicle_id_namespace")},
            {"name": LaunchConfiguration("vehicle_id")},
        ],
    )

    node_launch_plugin = Node(
        name=f"launch_plugin_{hostname}",
        namespace=LaunchConfiguration("muto_namespace"),
        package="composer",
        executable="launch_plugin",
        output="screen",
        parameters=[
            muto_params,
            {"namespace": LaunchConfiguration("vehicle_id_namespace")},
            {"name": LaunchConfiguration("vehicle_id")},
        ],
    )

    # Launch Description Object
    ld = LaunchDescription()

    ld.add_action(muto_namespace_arg)
    ld.add_action(vehicle_id_namespace_arg)
    ld.add_action(vehicle_id_arg)

    ld.add_action(node_agent)
    ld.add_action(node_mqtt_gateway)
    ld.add_action(node_commands)
    ld.add_action(node_twin)
    ld.add_action(node_composer)
    ld.add_action(node_compose_plugin)
    ld.add_action(node_native_plugin)
    ld.add_action(node_launch_plugin)

    return ld
