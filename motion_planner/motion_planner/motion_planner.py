"""Plan motion of the robot."""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    RobotState,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
)
from moveit_msgs.srv import GetCartesianPath_Response


class MotionPlanner:
    """Briefly describes the motion planner class."""

    def __init__(self, node: Node):
        """Initialize the motion planner node."""
        self._node = node
        self._cbgroup = MutuallyExclusiveCallbackGroup()
        self._c_move_group = ActionClient(
            node, MoveGroup, '/move_action', callback_group=self._cbgroup
        )
        self._logger = node.get_logger()
        self._logger.info('Motion_Planner Started. Waiting for goal')

    async def move_to_ee_pose(
        self,
        goal_ee_position: np.ndarray | None,
        goal_ee_orientation: np.ndarray | None,
        start_ee_pose: np.ndarray | None = None,
        execute_immediately: bool = False,
    ) -> None:
        """
        Move from a specified end-effector configuration to another.

        Args:
            goal_ee_position (np.ndarray): end EE position [x,y,z]. If not
            specified, any position is allowed such that the given orientation
            is achieved.
            goal_ee_orientation (np.ndarray): end EE position [x,y,z,w]-
            quaternion.If not specified, any orientation is allowed such that
            the given position is achieved.
            start_ee_pose (list[float]): start EE position & orientation;
            [x,y,z,x,y,z,w]. If not given, use current robot pose as start.
            execute_immediately (bool): immediately execute the pat

        """
        if goal_ee_orientation is None and goal_ee_position is None:
            raise ValueError(
                'One of orientation and position must be specified.'
            )
        goal_msg = MoveGroup.Goal()
        request = MotionPlanRequest()
        goal_constraint = Constraints()

        if goal_ee_position is not None:
            pos_constraint = PositionConstraint()
            pos_constraint.target_point_offset.x = goal_ee_position[0]
            pos_constraint.target_point_offset.y = goal_ee_position[1]
            pos_constraint.target_point_offset.z = goal_ee_position[2]
            goal_constraint.position_constraints.append(pos_constraint)
        if goal_ee_orientation is not None:
            orient_constraint = OrientationConstraint()
            orient_constraint.orientation.x = goal_ee_orientation[0]
            orient_constraint.orientation.y = goal_ee_orientation[1]
            orient_constraint.orientation.z = goal_ee_orientation[2]
            orient_constraint.orientation.w = goal_ee_orientation[3]
            goal_constraint.orientation_constraints.append(orient_constraint)

        request.goal_constraints = [goal_constraint]
        goal_msg.request = request
        planning_options = MoveGroup.PlanningOptions()
        planning_options.plan_only = not execute_immediately
        goal_msg.planning_options = planning_options

        self._logger.info('Sending goal to /move_action...')
        response_goal_handle = await self._c_move_group.send_goal_async(
            goal_msg
        )
        self._logger.info(
            f'Received response goal handle: {response_goal_handle.accepted}'
        )
        self._logger.info('Awaiting the result')
        response = await response_goal_handle.get_result_async()
        self._logger.info(f'Received the result: {response}')
        response_goal_handle.succeed()
        self._logger.info('Returning the result')

    async def move_to_joint_target(
        self,
        goal_joints: np.ndarray,
        start_joints: np.ndarray | None = None,
        execute_immediately: bool = False,
    ) -> None:
        """
        Move from a specified configuration in joint space to another.

        Args:
            start_joints (np.ndarray): array of joint angles for each joint.
            If not given, use current robot pose as start.
            goal_joints (np.ndarray): array of joint angles for each joint
            execute_immediately (bool): immediately execute the path

        """
        goal_msg = MoveGroup.Goal()
        request = MotionPlanRequest()

        if start_joints is not None:
            request.start_state = self.start_state(start_joints)

        request.goal_constraints = self.joint_constraints(goal_joints)
        goal_msg.request = request
        planning_options = MoveGroup.PlanningOptions()
        planning_options.plan_only = not execute_immediately
        goal_msg.planning_options = planning_options

        self._logger.info('Sending goal to /move_action...')
        response_goal_handle = await self._c_move_group.send_goal_async(
            goal_msg
        )
        self._logger.info(
            f'Received response goal handle: {response_goal_handle.accepted}'
        )
        self._logger.info('Awaiting the result')
        response = await response_goal_handle.get_result_async()
        self._logger.info(f'Received the result: {response}')
        response_goal_handle.succeed()
        self._logger.info('Returning the result')

        return response.result

    def plan_cartesian_path(
        self, goal_ee_pose: np.ndarray, start_ee_pose: np.ndarray | None = None
    ) -> GetCartesianPath_Response:
        """
        Plan a Cartesian path from any valid starting pose to a goal pose.

        Uses moveit_msgs/GetCartesianPath Service

        Args:
            goal_ee_pose (np.ndarray): destination pose [x,y,z,r,p,y]
            start_ee_pose (np.ndarray): start pose [x,y,z,r,p,y].
            If not provided, use current robot pose as start pose.
            execute_immediately (bool): immediately execute the path

        Returns:
            GetCartesianPath_Response: response of moveit GetCartesianPath srv

        """
        raise NotImplementedError

    def plan_to_named_config(
        self,
        named_config: str,
        start_ee_pose: np.ndarray | None = None,
        execute_immediately: bool = False,
    ) -> GetCartesianPath_Response:
        """
        Plan a path from any valid starting pose to a named configuration.

        This "named configuration" can be defined in an SRDF or a remembered
        from a previous call to moveit python library's remember_joint_values()
        method, per docs here:
        https://docs.ros.org/en/jade/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html#af9c9fc79be7fee5c366102db427fb28b

        Args:
            named_config (str): Named configuration
            start_ee_pose (np.ndarray, optional): start pose; if None, use
                current robot pose.
            execute_immediately (bool): immediately execute the path

        Returns:
            GetCartesianPath_Response: TODO figure out

        """
        raise NotImplementedError

    def list_named_configs(self) -> list[str]:
        """Return a list of the named configs currently registered."""
        return []

    def start_state(self, joints):
        """Set start state."""
        return RobotState()

    def joint_constraints(self, joints):
        """Set goal state."""
        return Constraints()


def main(args=None):
    """Spin the node."""
    rclpy.init()
    node = Node('moplan_test')

    plan = MotionPlanner(node)

    pos1 = np.array([0, 0, 0, 0, 0, 0])
    pos2 = np.array([1, 1, 1, 1, 1])
    _ = plan.move_to_joint_target(goal_joints=pos2, start_joints=pos1)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
