"""Plan motion of the robot."""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    RobotState,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    PlanningOptions,
    RobotTrajectory,
    BoundingVolume,
)
from motion_planner.robot_state import RobotState as RS
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import JointConstraint
from geometry_msgs.msg import Pose, Quaternion, PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint
from shape_msgs.msg import SolidPrimitive
import asyncio
import threading
import yaml

from motion_planner import robot_state, planning_scene


class MotionPlanner:
    """Briefly describes the motion planner class."""

    def __init__(
        self,
        node: Node,
        robot_state: robot_state.RobotState,
        planning_scene: planning_scene.PlanningScene,
    ):
        """Initialize the motion planner node."""
        self._node = node
        self.robot_state = RS(node)
        self._cbgroup = MutuallyExclusiveCallbackGroup()
        self._c_move_group = ActionClient(
            node, MoveGroup, '/move_action', callback_group=self._cbgroup
        )
        self._logger = node.get_logger()
        self._c_cartesian_path = self._node.create_client(
            GetCartesianPath,
            'compute_cartesian_path',
            callback_group=self._cbgroup,
        )
        self._services_up = False
        self._logger.info('MotionPlanner initialized.')

    def check_services_up(self) -> bool:
        """Wait until services we depend upon are available."""
        if self._services_up:
            return True

        for idx in range(5):
            if self._c_cartesian_path.wait_for_service(
                timeout_sec=5.0
            ) and self._c_move_group.wait_for_server(timeout_sec=5.0):
                self._logger.info('Services online.')
                self._services_up = True
                return True
            else:
                self._logger.warn(
                    'Still waiting for cartesian and/or move_group '
                    'service(s)...'
                )
        self._logger.error(
            'Retries expired. cartesian and/or move_group '
            'service(s) not available.'
        )
        return False

    async def move_to_ee_pose(
        self,
        goal_ee_position: np.ndarray | None,
        goal_ee_orientation: np.ndarray | None,
        start_joints: np.ndarray | None = None,
        execute_immediately: bool = False,
    ) -> ClientGoalHandle | None:
        """
        Move from a specified end-effector configuration to another.

        Args:
            goal_ee_position (np.ndarray): end EE position [x,y,z]. If not
            specified, any position is allowed such that the given orientation
            is achieved.
            goal_ee_orientation (np.ndarray): end EE position [x,y,z,w]-
            quaternion.If not specified, any orientation is allowed such that
            the given position is achieved.
            start_joints (np.ndarray): array of joint angles for each joint.
            If not given, use current robot pose as start.
            execute_immediately (bool): immediately execute the pat

        """
        if goal_ee_orientation is None and goal_ee_position is None:
            raise ValueError(
                'One of orientation and position must be specified.'
            )
        goal_msg = MoveGroup.Goal()
        request = MotionPlanRequest()
        goal_constraint = Constraints()
        request.group_name = 'fer_manipulator'
        request.num_planning_attempts = 10
        request.allowed_planning_time = 20.0
        request.max_velocity_scaling_factor = 0.1
        request.max_acceleration_scaling_factor = 0.1

        if goal_ee_position is not None:
            pos_constraint = PositionConstraint()
            pos_constraint.header.frame_id = 'base'
            pos_constraint.link_name = 'fer_hand_tcp'
            pos_constraint.target_point_offset.x = 0.0
            pos_constraint.target_point_offset.y = 0.0
            pos_constraint.target_point_offset.z = 0.0
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [0.01, 0.01, 0.01]
            pos_constraint.constraint_region = BoundingVolume()
            pos_constraint.constraint_region.primitives.append(box)  # type: ignore

            goal_box_pose = Pose()
            goal_box_pose.position.x = goal_ee_position[0]
            goal_box_pose.position.y = goal_ee_position[1]
            goal_box_pose.position.z = goal_ee_position[2]
            goal_box_pose.orientation.w = 1.0
            pos_constraint.weight = 1.0
            pos_constraint.constraint_region.primitive_poses.append(  # type: ignore
                goal_box_pose
            )
            goal_constraint.position_constraints.append(pos_constraint)  # type: ignore

        if goal_ee_orientation is not None:
            orient_constraint = OrientationConstraint()
            orient_constraint.header.frame_id = 'base'
            orient_constraint.link_name = 'fer_hand_tcp'
            q = Quaternion()
            q.x = goal_ee_orientation[0]
            q.y = goal_ee_orientation[1]
            q.z = goal_ee_orientation[2]
            q.w = goal_ee_orientation[3]
            orient_constraint.orientation = q
            orient_constraint.absolute_x_axis_tolerance = 0.2
            orient_constraint.absolute_y_axis_tolerance = 0.2
            orient_constraint.absolute_z_axis_tolerance = 0.2
            orient_constraint.weight = 1.0
            goal_constraint.orientation_constraints.append(orient_constraint)  # type: ignore

        request.goal_constraints = [goal_constraint]
        goal_msg.request = request
        planning_options = PlanningOptions()
        planning_options.plan_only = not execute_immediately
        goal_msg.planning_options = planning_options

        if start_joints is not None:
            request.start_state = self.start_state(start_joints)

        self._logger.info('Sending goal to /move_action...')
        response_goal = await self._c_move_group.send_goal_async(goal_msg)
        if response_goal is None:
            self._logger.error(
                'Received response goal of None from send_goal_async.'
            )
        else:
            self._logger.info(
                f'Received response goal handle: {response_goal.accepted}'
            )
        return response_goal

    async def move_to_joint_target(
        self,
        goal_joints: np.ndarray,
        start_joints: np.ndarray | None = None,
        execute_immediately: bool = False,
    ) -> MoveGroup.Result | None:
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
        request.group_name = 'fer_manipulator'
        request.num_planning_attempts = 5
        request.allowed_planning_time = 10.0
        request.max_velocity_scaling_factor = 0.1
        request.max_acceleration_scaling_factor = 0.1

        if start_joints is not None:
            request.start_state = self.start_state(start_joints)

        request.goal_constraints = self.joint_constraints(goal_joints)
        goal_msg.request = request
        planning_options = PlanningOptions()
        planning_options.plan_only = not execute_immediately
        goal_msg.planning_options = planning_options

        self._logger.info('Sending goal to /move_action...')
        response_goal_handle = await self._c_move_group.send_goal_async(
            goal_msg
        )
        if response_goal_handle is None:
            self._logger.error('Received None response_goal_handle')
            return None
        self._logger.info(
            f'Received response goal handle: {response_goal_handle.accepted}'
        )
        self._logger.info('Awaiting the result')
        response = await response_goal_handle.get_result_async()
        self._logger.debug(f'Received the response: {response}')
        return response.result  # type: ignore

    async def plan_cartesian_path(
        self,
        goal_ee_pose: np.ndarray,
        start_ee_pose: np.ndarray | None = None,
        execute_immediately: bool = False,
    ) -> GetCartesianPath.Response:
        """
        Plan a Cartesian path from any valid starting pose to a goal pose.

        Uses moveit_msgs/GetCartesianPath Service

        Args:
            goal_ee_pose (np.ndarray): destination pose [x,y,z,x,y,z,w]
            start_ee_pose (np.ndarray): start pose [x,y,z,x,y,z,w].
            If not provided, use current robot pose as start pose.
            execute_immediately (bool): immediately execute the path

        Returns:
            GetCartesianPath_Response: response of moveit GetCartesianPath srv

        """
        request = GetCartesianPath.Request()
        request.group_name = 'fer_manipulator'
        request.link_name = 'fer_hand_tcp'

        if goal_ee_pose is not None:
            goal_pose = Pose()
            goal_pose.position.x = goal_ee_pose[0]
            goal_pose.position.y = goal_ee_pose[1]
            goal_pose.position.z = goal_ee_pose[2]
            goal_pose.orientation.x = goal_ee_pose[3]
            goal_pose.orientation.y = goal_ee_pose[4]
            goal_pose.orientation.z = goal_ee_pose[5]
            goal_pose.orientation.w = goal_ee_pose[6]
            request.waypoints = [goal_pose]
            request.max_step = 0.01

        if start_ee_pose is not None:
            request.start_state = self.start_state(start_ee_pose)

        self._logger.info('Waiting for /compute_cartesian_path service')
        while not self._c_cartesian_path.wait_for_service(timeout_sec=10.0):
            self._logger.warn('Still waiting for service')

        self._logger.info('Request Cartesian path from service')
        response = await self._c_cartesian_path.call_async(request)

        return response  # type: ignore

    async def plan_to_named_config(
        self,
        named_config: str,
        start_ee_pose: np.ndarray | None = None,
        execute_immediately: bool = False,
    ) -> MoveGroup.Result | None:
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
            MoveGroup.Result: TODO figure out

        """
        goal_msg = MoveGroup.Goal()
        request = MotionPlanRequest()
        request.group_name = 'fer_manipulator'
        request.num_planning_attempts = 5
        request.allowed_planning_time = 10.0
        request.max_velocity_scaling_factor = 0.1
        request.max_acceleration_scaling_factor = 0.1

        named_states = {
            'ready': np.array(
                [
                    0.0,
                    -0.7853981633974483,
                    0.0,
                    -2.356194490192345,
                    0.0,
                    1.5707963267948966,
                    0.7853981633974483,
                ]
            ),
            'extended': np.array(
                [
                    0.0,
                    0.0,
                    0.0,
                    -0.1,
                    0.0,
                    1.5707963267948966,
                    0.7853981633974483,
                ]
            ),
        }
        if named_config not in named_states:
            raise ValueError('No such named configuration.')

        goal_joints = named_states[named_config]

        if start_ee_pose is not None:
            msg = PoseStamped()
            msg.header.frame_id = 'base'
            msg.pose.position.x = start_ee_pose[0]
            msg.pose.position.y = start_ee_pose[1]
            msg.pose.position.z = start_ee_pose[2]
            msg.pose.orientation.x = start_ee_pose[3]
            msg.pose.orientation.y = start_ee_pose[4]
            msg.pose.orientation.z = start_ee_pose[5]
            msg.pose.orientation.w = start_ee_pose[6]
            start_joint_state = self.robot_state.inverse_kinematics(msg)
            if start_joint_state is None:
                return None

            start_state = RobotState()
            start_state.joint_state = start_joint_state
            request.start_state = start_state
        request.goal_constraints = self.joint_constraints(goal_joints)

        goal_msg.request = request
        planning_options = PlanningOptions()
        planning_options.plan_only = not execute_immediately
        goal_msg.planning_options = planning_options
        self._logger.info('Sending goal')
        response_goal = await self._c_move_group.send_goal_async(goal_msg)
        if response_goal is None:
            self._logger.error('response_goal=None')
            return
        self._logger.info(
            f'Received response goal handle: {response_goal.accepted}'
        )
        response = await response_goal.get_result_async()
        if response is None:
            self._logger.error('response=None')
            return None
        return response.result

    def list_named_configs(self) -> list[str]:
        """Return a list of the named configs currently registered."""
        return ['ready', 'extended']

    def start_state(self, joints):
        """Set start state."""
        robotstate = RobotState()
        robotstate.joint_state.name = [
            'fer_joint1',
            'fer_joint2',
            'fer_joint3',
            'fer_joint4',
            'fer_joint5',
            'fer_joint6',
            'fer_joint7',
        ]
        if joints is None:
            robotstate.joint_state.position = [
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ]
        else:
            robotstate.joint_state.position = [float(j) for j in joints]
        return robotstate

    def joint_constraints(self, joints):
        """Set goal state."""
        constraints = Constraints()
        joint_names = [
            'fer_joint1',
            'fer_joint2',
            'fer_joint3',
            'fer_joint4',
            'fer_joint5',
            'fer_joint6',
            'fer_joint7',
        ]
        for i, joint_value in enumerate(joints):
            jointconstraint = JointConstraint()
            jointconstraint.joint_name = joint_names[i]
            jointconstraint.position = float(joint_value)
            jointconstraint.tolerance_above = 0.01
            jointconstraint.tolerance_below = 0.01
            constraints.joint_constraints.append(jointconstraint)  # type: ignore
        return [constraints]

    async def gripper(
        self,
        offset: float = 0.3,
        execute_immediately: bool = False,
    ) -> None:
        """
        Open and close the gripper.

        Args:
            offset(float): offset of each fingers from origin
            execute_immediately (bool): immediately execute the path

        """
        goal_msg = MoveGroup.Goal()
        request = MotionPlanRequest()
        request.group_name = 'hand'
        request.max_velocity_scaling_factor = 0.1
        request.max_acceleration_scaling_factor = 0.1

        constraints = Constraints()
        for joint in ['fer_finger_joint1', 'fer_finger_joint2']:
            jc = JointConstraint()
            jc.joint_name = joint
            jc.position = offset
            jc.tolerance_above = 0.005
            jc.tolerance_below = 0.005
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        request.goal_constraints = [constraints]
        goal_msg.request = request
        planning_options = PlanningOptions()
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
        self._logger.info('Returning the result')

        return response.result

    def save_plan(self, trajectory: RobotTrajectory, file_path: str) -> None:
        """
        Save planned trajectory.

        Args:
            trajectory (moveit_msgs/RobotTrajectory): Trajectory to be saved.
            file_path (str) : path to where the yaml file will be saved.

        """
        data = {
            'joint_names': list(trajectory.joint_trajectory.joint_names),
            'points': [],
        }

        for point in trajectory.joint_trajectory.points:
            data['points'].append(
                {
                    'positions': list(point.positions),
                    'velocities': list(point.velocities),
                    'accelerations': list(point.accelerations),
                    'time_from_start': {
                        'sec': point.time_from_start.sec,
                        'nanosec': point.time_from_start.nanosec,
                    },
                }
            )

        with open(file_path, 'w') as f:
            yaml.dump(data, f)
        self.node.get_logger().info(f'Trajectory saved to {file_path}')

    def load_plan(self, file_path: str) -> RobotTrajectory:
        """
        Open saved planned trajectory for inspection.

        Args:
            file_path (str): path to the saved file.

        """
        with open(file_path, 'r') as f:
            data = yaml.safe_load(f)

        traj = RobotTrajectory()
        traj.joint_trajectory.joint_names = data['joint_names']

        for pt in data['points']:
            p = JointTrajectoryPoint()
            p.positions = pt['positions']
            p.velocities = pt['velocities']
            p.accelerations = pt['accelerations']
            p.time_from_start.sec = pt['time_from_start']['sec']
            p.time_from_start.nanosec = pt['time_from_start']['nanosec']
            traj.joint_trajectory.points.append(p)

        self.node.get_logger().info(f'Loaded trajectory from {file_path}')
        return traj


async def integration_test(node: Node, planner: MotionPlanner) -> None:
    """Test move plan functions."""
    logger = node.get_logger()
    try:
        planner.check_services_up()
        logger.info('Service is ready. Waiting 5sec...')
        await asyncio.sleep(5.0)

        # Test for joint state movement
        pos1 = np.zeros(7)
        pos2 = np.ones(7)
        await planner.move_to_joint_target(
            goal_joints=pos2, start_joints=pos1, execute_immediately=True
        )

        # Test for ee pose movement
        ee_pos1 = np.array([0.3, 0.3, 0.5])
        ee_orient1 = np.array([1.0, 0.0, 0.0, 0.0])
        node.get_logger().info('Starting end-effector pose motion test...')

        goal_handle = await planner.move_to_ee_pose(
            goal_ee_position=ee_pos1,
            goal_ee_orientation=ee_orient1,
            start_joints=None,
            execute_immediately=True,
        )
        if goal_handle is not None and goal_handle.accepted:
            node.get_logger().info('Goal accepted.')
        else:
            node.get_logger().warn('Goal was rejected.')

        # Test for cartesian path
        # from ros2 param get /move_group robot_description_semantic - ready
        start_joints_pose = np.array(
            [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        )
        cartesian_goal_pose = np.array(
            [0.0, 0, 0.0, -2.356, 0.0, 1.571, 0.785]
        )
        response = await planner.plan_cartesian_path(
            goal_ee_pose=cartesian_goal_pose,
            start_ee_pose=start_joints_pose,
            execute_immediately=True,
        )
        node.get_logger().info(
            f'Cartesian path plan complete.'
            f'Fraction of path found: {response.fraction}'
        )

        resp = await planner.plan_to_named_config(
            named_config='extended',
            start_ee_pose=None,
            execute_immediately=True,
        )
        logger.info(f'NAMED CONFIG RESPONSE: {resp}')

        # Test gripper open/close
        await planner.gripper(offset=0.03, execute_immediately=True)
        await asyncio.sleep(2.0)
        await planner.gripper(offset=0.0, execute_immediately=True)

    finally:
        node.get_logger().info('Integration test finished.')


def main():
    """Run main."""
    rclpy.init()
    node = rclpy.create_node('test_motion_planner_node')
    rstate = robot_state.RobotState(node)
    scene = planning_scene.PlanningScene(node)
    planner = MotionPlanner(node, rstate, scene)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        asyncio.run(integration_test(node, planner))
    finally:
        executor.shutdown()
        executor_thread.join()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
