"""Move the robot from one pose to another."""

####################### Begin_Citation [3] ######################  # noqa: E266
import os
import time
import math
import numpy as np
import asyncio
import threading

import pytest
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener

from motion_planner.motion_planning_interface import MotionPlanningInterface


@pytest.mark.rostest
def generate_test_description():
    """Generate test description for the node."""
    pkg_share = get_package_share_directory('franka_fer_moveit_config')
    launch_file = os.path.join(pkg_share, 'launch', 'demo.launch.py')

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_file)
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


def _pos_from_tf(tf: TransformStamped) -> np.ndarray:
    """Extract position (x, y, z) from a TransformStamped."""
    return np.array(
        [
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z,
        ],
        dtype=float,
    )


def _pos_err(a: np.ndarray, b: np.ndarray) -> float:
    """Calculate distance between two points."""
    diff = a - b
    return float(np.linalg.norm(diff))


def _quat_to_np(q) -> np.ndarray:
    """Convert geometry_msgs/Quaternion to np.array([x, y, z, w])."""
    return np.array([q.x, q.y, q.z, q.w], dtype=float)


def _ori_err(q_target: np.ndarray, q_actual: np.ndarray) -> float:
    """Compute orientation difference (radians) between two quaternions."""
    q_target = q_target / np.linalg.norm(q_target)
    q_actual = q_actual / np.linalg.norm(q_actual)
    dot = abs(float(np.dot(q_target, q_actual)))
    dot = max(min(dot, 1.0), -1.0)
    return 2.0 * math.acos(dot)


def _quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Calculate Hamilton quaterion product q = q1 * q2."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    return np.array([x, y, z, w], dtype=float)


def test_move_pose_to_pose():
    """Plan and execute; check if goal is achieved."""
    rclpy.init()
    node = rclpy.create_node('it_move_pose_to_pose')

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        # TF buffer + listener
        tf_buffer = Buffer()
        tf_listener = TransformListener(tf_buffer, node, spin_thread=False)  # noqa: F841

        # Create MotionPlanningInterface object.
        mpi = MotionPlanningInterface(
            node=node,
            planning_group='fer_manipulator',
            base_link='base',
            ee_link='fer_hand_tcp',
            planning_scene_topic='/planning_scene',
        )

        if not mpi.planner.check_services_up():
            pytest.skip('MoveIt services/action server not available.')

        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            time.sleep(0.1)

        # Get starting EE transformation.
        base_frame = mpi.base_link
        ee_frame = mpi.ee_link

        tf_start = tf_buffer.lookup_transform(
            base_frame, ee_frame, rclpy.time.Time()
        )
        start_xyz = _pos_from_tf(tf_start)

        goal_xyz = start_xyz.copy()
        goal_xyz[0] += 0.10
        goal_xyz[1] += 0.10
        goal_xyz[2] += 0.10

        # orientation change
        q_delta = q_delta = np.array(
            [0.0, 0.0, math.sin(0.5), math.cos(0.5)],
            dtype=float,
        )

        start_q = _pos_from_tf(tf_start.transform.rotation)
        goal_xyzw = _quat_multiply(start_q, q_delta)

        async def _go():
            """Plan and execute via MPI."""
            return await mpi.plan_to_ee_pose_async(
                position_xyz=goal_xyz,
                orientation_xyzw=goal_xyzw,
                start_joints=None,
                execute_immediately=True,
            )

        goal_handle = asyncio.run(_go())
        assert goal_handle is not None

        # Wait for the robot to move
        time.sleep(3.0)

        # Get final EE pose from TF again.
        tf_final = tf_buffer.lookup_transform(
            base_frame, ee_frame, rclpy.time.Time()
        )
        final_xyz = _pos_from_tf(tf_final)
        error_pos = _pos_err(final_xyz, goal_xyz)
        assert error_pos <= 0.03

        final_q = _quat_to_np(tf_final.transform.rotation)
        error_ori = _ori_err(goal_xyzw, final_q)
        assert error_ori <= 0.1

    finally:
        node.destroy_node()
        executor.shutdown()
        executor_thread.join(timeout=1.0)
        rclpy.shutdown()


####################### End_Citation [3] ######################  # noqa: E266
