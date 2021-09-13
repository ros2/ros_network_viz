import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    composed = ComposableNodeContainer(
        name='ComponentManager',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='composition',
                plugin='composition::Talker',
                name='comp_talker'),
            ComposableNode(
                package='composition',
                plugin='composition::Listener',
                name='comp_listener')
        ],
        output='both',
    )

    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='both',
    )

    lc_talker = LifecycleNode(
        name='lc_talker',
        namespace='',
        package='lifecycle',
        executable='lifecycle_talker',
        output='both',
    )

    listener = Node(
        package='demo_nodes_cpp',
        executable='listener',
        output='both',
    )

    talker = Node(
        package='demo_nodes_cpp',
        executable='listener',
        output='both',
    )

    turtle = Node(
        package='turtlesim',
        executable='turtlesim_node',
        output='both',
    )

    return launch.LaunchDescription([
        composed,
        jsp,
        lc_talker,
        listener,
        talker,
        turtle,
    ])
