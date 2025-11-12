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

    async def plan_to_ee_pose_async(
        self,
        position_xyz: Optional[np.ndarray],
        orientation_xyzw: Optional[np.ndarray],
        start_joints: Optional[np.ndarray] = None,
        execute_immediately: bool = False,
    ):
        """Asynchronously plan to an end-effector pose."""
        return await self.planner.move_to_ee_pose(
            goal_ee_position=position_xyz,
            goal_ee_orientation=orientation_xyzw,
            start_joints=start_joints,
            execute_immediately=execute_immediately,
        )

    async def plan_to_joint_target_async(
        self,
        goal_joints: np.ndarray,
        start_joints: Optional[np.ndarray] = None,
        execute_immediately: bool = False,
    ):
        """Asynchronously plan to a joint target."""
        return await self.planner.move_to_joint_target(
            goal_joints=goal_joints,
            start_joints=start_joints,
            execute_immediately=execute_immediately,
        )
