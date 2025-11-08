"""Plan motion of the robot."""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, RobotState, Constraints


class Motion_Planner(Node):
    """Briefly describes the motion planner class."""

    def __init__(self):
        """Initialize the motion planner node."""
        super().__init__('motion_planner')
        self._cbgroup = MutuallyExclusiveCallbackGroup()
        """
        self.joint_config_server = ActionServer(self,
                                    MoveGroup,
                                    '/viz/move_joint_config',
                                    self.move_joint_config_cb,
                                    callback_group = self._cbgroup)
        """
        self._client = ActionClient(
            self, MoveGroup, '/move_action', callback_group=self._cbgroup
        )
        self.get_logger().info('Motion_Planner Started. Waiting for goal')

    async def move_to_joint_target(self, goal_joints, start_joints):
        """Plan a path from any valid starting joint configuration."""
        goal_msg = MoveGroup.Goal()

        request = MotionPlanRequest()

        if start_joints is not None:
            request.start_state = self.start_state(start_joints)

        request.goal_constraints = self.joint_constraints(goal_joints)
        goal_msg.request = request
        self.get_logger().info('Sending goal to /move_action...')
        response_goal_handle = await self._client.send_goal_async(goal_msg)
        self.get_logger().info(
            f'Received response goal handle: {response_goal_handle.accepted}'
        )
        self.get_logger().info('Awaiting the result')
        response = await response_goal_handle.get_result_async()
        self.get_logger().info(
            f'Received the result: {response}'
        )  # Do more formatting
        response_goal_handle.succeed()
        self.get_logger().info('Returning the result')

        return response.result

    def start_state(self, joints):
        """Set start state."""
        return RobotState()

    def joint_constraints(self, joints):
        """Set goal state."""
        return Constraints()


def main(args=None):
    """Spin the node."""
    rclpy.init()
    node = Motion_Planner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
