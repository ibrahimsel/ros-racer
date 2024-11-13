from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    muto_namespace_arg = DeclareLaunchArgument(
        "muto_namespace",
        default_value="muto"
    )
    muto_params = "./config/muto.yaml"

    node_agent = Node(
        namespace=LaunchConfiguration("muto_namespace"),
        name="agent",
        package="agent",
        executable="muto_agent",
        output="screen",
        parameters=[
            muto_params            
        ]
    )

    node_mqtt_gateway = Node(
        namespace=LaunchConfiguration("muto_namespace"),
        name="gateway",
        package="agent",
        executable="mqtt",
        output="screen",
        parameters=[
            muto_params            
        ]
    )

    node_commands = Node(
        namespace=LaunchConfiguration("muto_namespace"),
        name="commands_plugin",
        package="agent",
        executable="commands",
        output="screen",
        parameters=[
            muto_params            
        ]
    )

    node_twin = Node(
        namespace=LaunchConfiguration("muto_namespace"),
        name="core_twin",
        package="core",
        executable="twin",
        output="screen",
        parameters=[
            muto_params            
        ]
    )

    node_composer = Node(
        namespace=LaunchConfiguration("muto_namespace"),
        package="composer",
        executable="muto_composer",
        output="screen",
        parameters=[
            muto_params            
        ]
    )

    node_compose_plugin = Node(
        namespace=LaunchConfiguration("muto_namespace"),
        package="composer",
        executable="compose_plugin",
        output="screen",
        parameters=[
            muto_params            
        ]
    )

    node_native_plugin = Node(
        namespace=LaunchConfiguration("muto_namespace"),
        package="composer",
        executable="native_plugin",
        output="screen",
        parameters=[
            muto_params            
        ]
    )

    node_launch_plugin = Node(
        namespace=LaunchConfiguration("muto_namespace"),
        package="composer",
        executable="launch_plugin",
        output="screen",
        parameters=[
            muto_params            
        ]
    )


    ld = LaunchDescription()

    ld.add_action(muto_namespace_arg)
    
    ld.add_action(node_agent)
    ld.add_action(node_mqtt_gateway)
    ld.add_action(node_commands)
    ld.add_action(node_twin)
    ld.add_action(node_composer)
    ld.add_action(node_compose_plugin)
    ld.add_action(node_native_plugin)
    ld.add_action(node_launch_plugin)

    return ld
