"""Test RobotState functionality."""

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState

from motion_planner.robot_state import RobotState


def _pose_dist(a: PoseStamped, b: PoseStamped) -> float:
    dx = a.pose.position.x - b.pose.position.x
    dy = a.pose.position.y - b.pose.position.y
    dz = a.pose.position.z - b.pose.position.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)


class RobotStateTest(Node):
    """Test node that logs RobotState once a second."""

    def __init__(self) -> None:
        """Initialize RobotState and create timer to test it."""
        super().__init__('robot_state_poll_test')
        self.rs = RobotState(
            node=self,
            planning_group='fer_arm',
            base_link='fer_link0',
            ee_link='fer_hand',
        )
        self.create_timer(1.0, self.test_robot_state)

    def test_robot_state(self) -> None:
        """Print current joints, FK, IK, FK(IK) results."""
        js: Optional[JointState] = self.rs.current_joint_state()
        if js is None or not js.name:
            self.get_logger().info('Waiting for /joint_states')
            return

        self.get_logger().info(
            f'Joints: {list(js.name)}\n'
            f'Pos: {[round(p, 4) for p in js.position]}'
        )

        if not self.rs._fk_cli.service_is_ready():
            self.get_logger().warn('/compute_fk not ready yet')
            return

        fk_list = self.rs.forward_kinematics(
            joint_state=js, link_names=[self.rs._ee_link]
        )
        if not fk_list:
            self.get_logger().warn('FK returned no poses.')
            return
        cur_pose = fk_list[0]
        self.get_logger().info(
            f'FK(current) -> EE @ {cur_pose.header.frame_id}: '
            f'({cur_pose.pose.position.x:.3f}, '
            f'{cur_pose.pose.position.y:.3f}, '
            f'{cur_pose.pose.position.z:.3f})'
        )

        if not self.rs._ik_cli.service_is_ready():
            self.get_logger().warn('/compute_ik not ready yet')
            return

        goal = PoseStamped()
        goal.header.frame_id = self.rs._base_link
        goal.pose = cur_pose.pose
        goal.pose.position.x += 0.05

        ik_js = self.rs.inverse_kinematics(target_pose=goal)
        if ik_js is None:
            self.get_logger().warn('No IK solution found.')
            return

        fk_of_ik = self.rs.forward_kinematics(
            joint_state=ik_js, link_names=[self.rs._ee_link]
        )
        if not fk_of_ik:
            self.get_logger().warn('FK after IK returned no poses.')
            return

        err = _pose_dist(goal, fk_of_ik[0])
        self.get_logger().info(f'FK(IK(goal)) distance to goal: {err:.6f} m')
        self.get_logger().info(
            f'IK joints (first 7): {[round(p, 5) for p in ik_js.position[:7]]}'
        )
        self.get_logger().info(
            'FK(IK) pose: '
            f'({fk_of_ik[0].pose.position.x:.6f}, '
            f'{fk_of_ik[0].pose.position.y:.6f}, '
            f'{fk_of_ik[0].pose.position.z:.6f})'
        )


def main() -> None:
    """Run the RobotState test node."""
    rclpy.init()
    node = RobotStateTest()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
