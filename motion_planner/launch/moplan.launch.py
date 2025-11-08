"""Integration test launchfile for motion planner."""

from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    """ROS2 launch description generator."""
    moveit_config = MoveItConfigsBuilder(
        'fer', package_name='franka_fer_moveit_config'
    ).to_moveit_configs()

    return LaunchDescription(
        [
            Node(
                package='rviz2',
                executable='rviz2',
                output='log',
                arguments=[
                    '-d',
                    PathJoinSubstitution(
                        [
                            FindPackageShare('franka_fer_moveit_config'),
                            'config',
                            'moveit.rviz',
                        ]
                    ),
                ],
                parameters=[
                    moveit_config.planning_pipelines,
                    moveit_config.robot_description_kinematics,
                ],
                remappings=[('/move_action', '/viz/move_action')],
            ),
            Node(
                package='motion_planner',
                executable='motion_planner_integration_test',
            ),
        ]
    )
