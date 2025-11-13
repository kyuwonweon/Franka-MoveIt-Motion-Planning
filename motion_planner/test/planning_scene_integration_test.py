"""Test planning scene."""

import time
import numpy as np
import asyncio

import threading

import rclpy
import pytest

from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener

from motion_planner.motion_planning_interface import MotionPlanningInterface


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


def test_move_into_block():
    """Place a block and try to move into it."""
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
        goal_xyz[0] += 0.15
        goal_xyz[1] += 0.15
        goal_xyz[2] += 0.15

        q = tf_start.transform.rotation
        goal_xyzw = np.array([q.x, q.y, q.z, q.w], dtype=float)

        # Add a box at the goal position.
        box_size = (0.03, 0.03, 0.03)
        box_pos = (float(goal_xyz[0]), float(goal_xyz[1]), float(goal_xyz[2]))
        mpi.scene.add_box('box1', box_size, box_pos)
        time.sleep(1.0)

        async def _plan():
            """Plan via MPI and check if it fails."""
            goal_handle = await mpi.plan_to_ee_pose_async(
                position_xyz=goal_xyz,
                orientation_xyzw=goal_xyzw,
                start_joints=None,
                execute_immediately=True,
            )
            if goal_handle is None:
                return None

            result_future = goal_handle.get_result_async()
            result_msg = await result_future
            return result_msg

        result_msg = asyncio.run(_plan())

        mpi.scene.remove_world_object('box1')
        time.sleep(1.0)

        # Check if planning failed (as expected).
        if result_msg is None:
            blocked = True
        else:
            error_code = result_msg.result.error_code
            blocked = error_code.val != error_code.SUCCESS
        assert blocked

    finally:
        node.destroy_node()
        executor.shutdown()
        executor_thread.join(timeout=1.0)
        rclpy.shutdown()
