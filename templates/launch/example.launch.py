# Copyright 2026 Antigravity
# Example ROS 2 Launch File Template

from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch.event_handlers import OnProcessStart


def generate_launch_description():
    """Generate launch description for example nodes."""
    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    # Standard Node example
    standard_node = Node(
        package='test_agent_pkg',
        executable='test_publisher',
        name='test_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # Lifecycle Node example
    lifecycle_node = LifecycleNode(
        package='antigravity_cpp_node',
        executable='lifecycle_component_exe',
        name='lifecycle_component',
        namespace='',
        output='screen'
    )

    # Auto-configure lifecycle node on start
    configure_event = RegisterEventHandler(
        OnProcessStart(
            target_action=lifecycle_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda node: True,
                        transition_id=Transition.TRANSITION_CONFIGURE
                    )
                )
            ]
        )
    )

    return LaunchDescription([
        use_sim_time,
        standard_node,
        lifecycle_node,
        configure_event
    ])
