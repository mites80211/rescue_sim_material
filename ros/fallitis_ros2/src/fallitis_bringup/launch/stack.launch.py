from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config = Path(__file__).resolve().parent.parent / "config" / "stack.yaml"
    return LaunchDescription(
        [
            Node(
                package="fallitis_actuation",
                executable="actuation_node",
                parameters=[str(config)],
                output="screen",
            ),
            Node(
                package="fallitis_perception",
                executable="perception_node",
                parameters=[str(config)],
                output="screen",
            ),
            Node(
                package="fallitis_mapping",
                executable="mapping_node",
                parameters=[str(config)],
                output="screen",
            ),
            Node(
                package="fallitis_web",
                executable="web_server",
                output="screen",
            ),
        ]
    )
