"""Test for PlanningScene functionality."""

from time import sleep

import rclpy

from motion_planner.planning_scene import PlanningScene


def main():
    """Spin a node that publishes planning scene changes."""
    rclpy.init()
    node = rclpy.create_node('planning_scene_test')
    ps = PlanningScene(node, world_frame='base', ee_link='fer_hand')

    def _publish_scene():
        params = {
            'boxes': [
                {
                    'name': 'obj1',
                    'lwh': [0.8, 0.2, 0.05],
                    'xyz': [0.60, 0.20, -0.025],
                },
                {
                    'name': 'obj2',
                    'lwh': [0.05, 0.05, 0.10],
                    'xyz': [0.50, 0.50, 0.05],
                },
            ]
        }
        ps.load_scene(params)
        sleep(3)

        ps.add_box('box1', (0.05, 0.05, 0.1), (0.50, 0.0, 0.0))
        ps.add_box('box2', (0.2, 0.2, 0.4), (0.55, 1.0, 0.0))
        sleep(3)

        ps.attach_box('box1')
        sleep(3)

        ps.detach_box('box1')
        sleep(3)

        ps.remove_world_object('box1')
        ps.remove_world_object('box2')
        ps.remove_world_object('obj1')
        ps.remove_world_object('obj2')
        sleep(3)

    node.create_timer(0.5, _publish_scene)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
