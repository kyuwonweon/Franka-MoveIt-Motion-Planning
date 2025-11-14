"""
pick_node.

Exposes a /pick service (std_srvs/Trigger) that:
  1) Publishes a planning scene (table, target object, obstacle)
  2) Moves above the object (approach)
  3) Descends to grasp
  4) Attaches a box to the EE to simulate grasp
  5) Carries it to the place location and releases.
"""

from __future__ import annotations

import asyncio
import threading

from motion_planner.motion_planning_interface import MotionPlanningInterface
from motion_planner.planning_scene import PlanningScene
import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger


# Optional service type import (only for readiness check)


class PickNode(Node):
    """Pick-and-place sample task node."""

    def __init__(self) -> None:
        """Initialize node."""
        super().__init__('pick_node')

        # Parameters (can be overridden in launch)
        self.declare_parameter('table.size', [0.8, 1.2, 0.05])
        self.declare_parameter('table.xyz', [0.5, 0.0, 0.25])
        self.declare_parameter('object.size', [0.04, 0.04, 0.12])
        self.declare_parameter('object.xyz', [0.50, -0.10, 0.31])
        self.declare_parameter('obstacle.size', [0.2, 0.05, 0.25])
        self.declare_parameter('obstacle.xyz', [0.55, 0.00, 0.35])
        self.declare_parameter('place.xyz', [0.50, 0.20, 0.31])
        self.declare_parameter('approach_dz', 0.15)
        self.declare_parameter('lift_dz', 0.10)
        self.declare_parameter('ee.orientation_xyzw', [1.0, 0.0, 0.0, 0.0])

        # Interfaces (share the same Node)
        self.mpi = MotionPlanningInterface(
            node=self,
            planning_group='fer_manipulator',
            base_link='base',
            ee_link='fer_hand_tcp',
        )
        self.scene: PlanningScene = self.mpi.scene

        # Service API
        self._pick_srv = self.create_service(Trigger, 'pick', self.on_pick)
        self._ready_srv = self.create_service(Trigger, 'ready', self.on_ready)

        # Internal flags
        self._is_running = False
        self._scene_bootstrapped = False

        # Bootstrap the scene once the MoveGroup API is ready.
        # This timer allows the node to come up first, then we perform
        # readiness checks.
        self.create_timer(
            0.1,
            self._try_bootstrap_scene,
            clock=self.get_clock(),
        )

        self.get_logger().info(
            'pick_node ready. Call: '
            r'ros2 service call /pick std_srvs/srv/Trigger {}'
        )

    # ---------- readiness & scene bootstrap ----------

    def _move_group_ready(self) -> bool:
        """
        Check whether move_group exposes its planning scene service.

        This is a robust way to tell the PlanningSceneMonitor is alive.
        """
        return self.mpi.scene.n_subscribers() >= 1

    async def _republish_scene_burst(
        self,
        repeats: int = 5,
        interval_sec: float = 0.25,
    ) -> None:
        """Re-publish collision objects to avoid race conditions."""
        table_lwh = tuple(self.get_parameter('table.size').value)
        table_xyz = tuple(self.get_parameter('table.xyz').value)
        obj_lwh = tuple(self.get_parameter('object.size').value)
        obj_xyz = tuple(self.get_parameter('object.xyz').value)
        obs_lwh = tuple(self.get_parameter('obstacle.size').value)
        obs_xyz = tuple(self.get_parameter('obstacle.xyz').value)

        for i in range(repeats):
            # Deterministic IDs; repeated ADDs are idempotent for same id.
            self.scene.add_box('table', table_lwh, table_xyz)
            self.scene.add_box('target_obj', obj_lwh, obj_xyz)
            self.scene.add_box('obstacle', obs_lwh, obs_xyz)
            self.get_logger().info(
                f'Published planning scene burst {i + 1}/{repeats}'
            )
            await asyncio.sleep(interval_sec)

        self._scene_bootstrapped = True
        self.get_logger().info('Planning scene bootstrapped.')

    def _try_bootstrap_scene(self) -> None:
        """Timer: once move_group is up, publish the scene repeatedly once."""
        if self._scene_bootstrapped:
            return

        # Wait for MoveGroup service to be available (~5 s overall at 10 Hz).
        if not self._move_group_ready():
            return

        self.get_logger().info(
            'move_group is ready. Bootstrapping planning scene...'
        )
        asyncio.create_task(
            self._republish_scene_burst(repeats=8, interval_sec=0.2)
        )

    # ---------- pick workflow ----------

    async def do_pick_place(self) -> None:
        """Execute the pick-and-place sequence asynchronously."""
        # If /pick is called early, enforce scene bootstrap first.
        deadline = self.get_clock().now() + Duration(seconds=5.0)
        while (
            not self._scene_bootstrapped and self.get_clock().now() < deadline
        ):
            await asyncio.sleep(0.05)
        if not self._scene_bootstrapped:
            # As a last resort, republish a short burst synchronously.
            await self._republish_scene_burst(repeats=3, interval_sec=0.2)

        obj_xyz: tuple[float, float, float] = tuple(
            self.get_parameter('object.xyz').value  # type: ignore
        )
        place_xyz: tuple[float, float, float] = tuple(
            self.get_parameter('place.xyz').value  # type: ignore
        )
        approach_dz: float = float(self.get_parameter('approach_dz').value)
        lift_dz: float = float(self.get_parameter('lift_dz').value)
        ori_xyzw = np.array(
            self.get_parameter('ee.orientation_xyzw').value,
            dtype=float,  # type: ignore
        )
        object_size = np.array(
            self.get_parameter('object.size').value  # type: ignore
        )
        gripper_closed_offset = object_size[1] / 2.0

        approach = np.array(
            [obj_xyz[0], obj_xyz[1], obj_xyz[2] + approach_dz], dtype=float
        )
        grasp = np.array(obj_xyz, dtype=float)
        lift = np.array(
            [obj_xyz[0], obj_xyz[1], obj_xyz[2] + lift_dz], dtype=float
        )
        place = np.array(place_xyz, dtype=float)
        place_up = np.array(
            [place_xyz[0], place_xyz[1], place_xyz[2] + lift_dz], dtype=float
        )

        log = self.get_logger().info

        log('OPEN GRIPPER...\n\n')
        await self.mpi.grip_open()

        log('MOVE ABOVE OBJECT...\n\n')
        await self.mpi.go_to_ee_pose(
            position_xyz=approach,
            orientation_xyzw=ori_xyzw,
        )

        log('DESCEND TO GRASP...\n\n')
        await self.mpi.go_to_ee_pose(
            position_xyz=grasp,
            orientation_xyzw=ori_xyzw,
        )

        log('ATTACH OBJECT...\n\n')
        # Attach at the TCP; PlanningScene should use ee_link frame.
        self.scene.attach_box('target_obj')

        log('CLOSE GRIPPER...\n\n')
        await self.mpi.grip(gripper_closed_offset)

        log('LIFT...\n\n')
        await self.mpi.go_to_ee_pose(
            position_xyz=lift,
            orientation_xyzw=ori_xyzw,
        )

        log('CARRY TO PLACE...\n\n')
        await self.mpi.go_to_ee_pose(
            position_xyz=place_up,
            orientation_xyzw=ori_xyzw,
        )
        await self.mpi.go_to_ee_pose(
            position_xyz=place,
            orientation_xyzw=ori_xyzw,
        )

        log('OPEN GRIPPER...\n\n')
        await self.mpi.grip_open()

        log('RELEASE...\n\n')
        self.scene.detach_box('target_obj')

        log('RETREAT...\n\n')
        await self.mpi.go_to_ee_pose(
            position_xyz=place_up,
            orientation_xyzw=ori_xyzw,
        )

        log('Pick-and-place complete.')
        return

    def _run_pick_in_thread(self) -> None:
        """Worker thread that runs the async sequence."""
        try:
            asyncio.run(self.do_pick_place())
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'pick task failed: {e}')
        finally:
            self._is_running = False

    def on_pick(self, req: Trigger.Request, _ctx) -> Trigger.Response:
        """Service callback: start the sequence in background."""
        self.get_logger().info(
            'pick service called. Triggering pick & place...'
        )

        resp = Trigger.Response()
        if self._is_running:
            resp.success = False
            resp.message = 'pick already running'
            return resp

        self._is_running = True
        t = threading.Thread(target=self._run_pick_in_thread, daemon=True)
        t.start()

        resp.success = True
        resp.message = 'complete'
        return resp

    async def _run_ready(self) -> None:
        await self.mpi.ready()

    def on_ready(self, req: Trigger.Request, _ctx) -> Trigger.Response:
        """Perform ready service callback."""
        self.get_logger().info('Returning to the ready position...')
        try:
            self._is_running = True
            asyncio.run(self._run_ready())
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'ready task failed: {e}')
        finally:
            self._is_running = False

        resp = Trigger.Response()
        resp.success = True
        resp.message = 'started'
        return resp


def main() -> None:
    """Entry point."""
    rclpy.init()
    node = PickNode()
    try:
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
