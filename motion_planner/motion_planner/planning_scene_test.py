"""Test for PlanningScene functionality."""

from time import sleep

from motion_planner.planning_scene import PlanningScene

import rclpy


def main():
    """Spin a node that publishes planning scene changes."""
    rclpy.init()
    node = rclpy.create_node('planning_scene_test')
    ps = PlanningScene(node, world_frame='base', ee_link='fer_hand_tcp')

    def _publish_scene():
        ps.add_box('box1', (0.05, 0.05, 0.1), (0.50, 0.0, 0.0))
        ps.add_box('box2', (0.2, 0.2, 0.4), (0.55, 1.0, 0.0))
        sleep(5)

        ps.attach_box('box1')
        sleep(5)

        ps.detach_box('box1')
        sleep(5)

        ps.remove_world_object('box1')
        ps.remove_world_object('box2')
        sleep(5)

    node.create_timer(0.5, _publish_scene)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
