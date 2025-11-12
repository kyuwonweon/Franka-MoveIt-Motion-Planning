"""Robot state class. TODO better docs."""

from typing import List, Optional

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotState as MoveItRobotState
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState


class RobotState:
    """
    Wrapper for robot state functionality.

    Provides:
      - Current robot joint state (via /joint_states subscriber)
      - Forward kinematics (via /compute_fk service)
      - Inverse kinematics (via /compute_ik service)
    """

    def __init__(
        self,
        node: Node,
        planning_group: str = 'fer_manipulator',
        base_link: str = 'base',
        ee_link: str = 'fer_hand_tcp',
    ) -> None:
        """Initialize RobotState with ROS2 node and parameters."""
        self._node = node
        self._group = planning_group
        self._base_link = base_link
        self._ee_link = ee_link

        self.client_group = MutuallyExclusiveCallbackGroup()

        # Store latest /joint_states messages
        self._js_cache: Optional[JointState] = None
        self._js_sub = node.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        self._ik_cli = node.create_client(
            GetPositionIK, '/compute_ik', callback_group=self.client_group
        )
        self._fk_cli = node.create_client(
            GetPositionFK, '/compute_fk', callback_group=self.client_group
        )

    def joint_state_callback(self, msg: JointState) -> None:
        """Update cached joint state whenever the robot publishes."""
        self._js_cache = msg

    def current_joint_state(self) -> JointState:
        """Return the latest /joint_states message."""
        return self._js_cache

    def current_ee_pose(self) -> PoseStamped:
        """Compute EE pose from current joints via FK."""
        js = self._js_cache
        if js is None:
            return None
        fk_pose_list = self.forward_kinematics(js, [self._ee_link])
        return fk_pose_list[0] if fk_pose_list else None

    def inverse_kinematics(
        self, target_pose: PoseStamped
    ) -> Optional[JointState]:
        """Compute IK for a target pose."""
        req = GetPositionIK.Request()
        req.ik_request.group_name = self._group
        req.ik_request.pose_stamped = target_pose
        req.ik_request.timeout = Duration(sec=1, nanosec=0)

        if self._js_cache is not None:
            req.ik_request.robot_state.joint_state = self._js_cache

        fut = self._ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self._node, fut)

        # If pose is unreachable, return None instead of crashing
        res = fut.result()
        if res is None or res.error_code.val != res.error_code.SUCCESS:
            return None

        return res.solution.joint_state

    def forward_kinematics(
        self,
        joint_state: JointState,
        link_names: List[str],
    ) -> List[PoseStamped]:
        """Compute FK for given joint state to the requested link_names."""
        rs = MoveItRobotState()
        rs.joint_state = joint_state

        req = GetPositionFK.Request()
        req.fk_link_names = link_names
        req.robot_state = rs
        req.header.frame_id = self._base_link

        fut = self._fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self._node, fut)

        res = fut.result()
        if res is None or res.error_code.val != res.error_code.SUCCESS:
            return []

        return list(res.pose_stamped)
