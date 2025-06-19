import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import uuid

def generate_launch_description():
    # Arguments
    default_id = str(uuid.uuid4())
    muto_namespace_arg = DeclareLaunchArgument("muto_namespace", default_value=f"muto_{default_id}".replace("-", "_"))  # replace uuid dashes as they are not allowed in ROS namespaces
    vehicle_id_namespace_arg = DeclareLaunchArgument(
        "vehicle_id_namespace",
        default_value="org.eclipse.muto.sandbox",
        description="Vehicle ID namespace",
    )
    vehicle_id_arg = DeclareLaunchArgument(
        "vehicle_id", default_value=f"docker-{str(uuid.uuid4())}", description="Vehicle ID"
    )

    # for prints
    os.environ["PYTHONUNBUFFERED"] = "1"

    # Files
    muto_params = "/edge/config/muto.yaml"

    # Agent
    node_agent = Node(
        namespace=LaunchConfiguration("muto_namespace"),
        name="agent",
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
        name="gateway",
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
        name="commands_plugin",
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
        name="core_twin",
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
        namespace=LaunchConfiguration("muto_namespace"),
        name="muto_composer",
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
        namespace=LaunchConfiguration("muto_namespace"),
        name="compose_plugin",
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
        namespace=LaunchConfiguration("muto_namespace"),
        name="native_plugin",
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
        namespace=LaunchConfiguration("muto_namespace"),
        name="launch_plugin",
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
