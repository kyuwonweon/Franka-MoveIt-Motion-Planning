"""MotionPlanningInterface class."""

from typing import Optional
import numpy as np
from rclpy.node import Node

from motion_planner.robot_state import RobotState
from motion_planner.planning_scene import PlanningScene
from motion_planner.motion_planner import MotionPlanner


class MotionPlanningInterface:
    """
    Class to integrate RobotState, PlanningScene, and MotionPlanner.

    Binds integrated classes to a user-provided rclpy Node.
    """

    def __init__(
        self,
        node: Node,
        planning_group: str = 'fer_manipulator',
        base_link: str = 'base',
        ee_link: str = 'fer_hand_tcp',
        planning_scene_topic: str = '/planning_scene',
    ) -> None:
        """Initialize the MotionPlanningInterface with given parameters."""
        self.node = node
        self.logger = node.get_logger()

        # Compose the three subsystems, sharing the same node

        # Initialize RobotState
        self.robot_state = RobotState(
            node=node,
            planning_group=planning_group,
            base_link=base_link,
            ee_link=ee_link,
        )

        # Initialize PlanningScene
        self.scene = PlanningScene(
            node=node,
            world_frame=base_link,
            ee_link=ee_link,
            planning_scene_topic=planning_scene_topic,
        )

        # Initialize MotionPlanner
        self.planner = MotionPlanner(node, self.robot_state, self.scene)

        self.base_link = base_link
        self.ee_link = ee_link
        self.group = planning_group

    async def go_to_ee_pose(
        self,
        position_xyz: Optional[np.ndarray],
        orientation_xyzw: Optional[np.ndarray],
        start_joints: Optional[np.ndarray] = None,
    ):
        """Asynchronously plan to an end-effector pose."""
        goal_handle = await self.planner.move_to_ee_pose(
            goal_ee_position=position_xyz,
            goal_ee_orientation=orientation_xyzw,
            start_joints=start_joints,
            execute_immediately=True,
        )
        if goal_handle is not None:
            result_future = goal_handle.get_result_async()
            result = await result_future
            if result is None:
                self.logger.error('Result of move action is None.')
            else:
                self.node.get_logger().info(
                    f'MoveGroup action completed with status: {result.status}'
                )

    async def go_to_joint_target(
        self,
        goal_joints: np.ndarray,
        start_joints: Optional[np.ndarray] = None,
    ) -> None:
        """Asynchronously plan to a joint target."""
        await self.planner.move_to_joint_target(
            goal_joints=goal_joints,
            start_joints=start_joints,
            execute_immediately=True,
        )

    async def grip(
        self,
        offset: float,
    ) -> None:
        """Block for control the gripper."""
        await self.planner.gripper(
            offset=offset,
            execute_immediately=True,
        )

    async def ready(self) -> None:
        """Return the robot to the ready state."""
        goal_handle = await self.planner.plan_to_named_config(
            'ready', execute_immediately=True
        )
        if goal_handle is not None:
            result_future = goal_handle.get_result_async()
            result = await result_future
            if result is None:
                self.logger.error('Result of move action is None.')
            else:
                self.node.get_logger().info(
                    f'MoveGroup action completed with status: {result.status}'
                )

    async def grip_open(self) -> None:
        """Open the gripper."""
        await self.grip(self.planner.GRIPPER_OPEN)

    async def grip_closed(self) -> None:
        """Close the gripper."""
        await self.grip(self.planner.GRIPPER_CLOSED)
