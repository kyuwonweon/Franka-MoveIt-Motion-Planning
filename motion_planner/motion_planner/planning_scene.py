"""Planning scene wrapper. TODO: improve docstrings."""

from typing import Dict, Tuple

from geometry_msgs.msg import PoseStamped
from moveit_commander import PlanningSceneInterface


class PlanningScene:
    """
    A helper class to manage collision objects in the planning scene.

    This class provides methods for adding, removing, attaching, and detaching
    objects to/from the planning scene using MoveIt's PlanningSceneInterface.
    """

    def __init__(
        self, node, world_frame: str = 'fer_link0', ee_link: str = 'fer_hand'
    ):
        """Initialize the planning scene interface."""
        self._node = node
        self._frame = world_frame
        self._ee_link = ee_link
        self._scene = PlanningSceneInterface()

    def add_box(
        self,
        name: str,
        size: Tuple[float, float, float],
        position: Tuple[float, float, float],
    ):
        """Add boxes to the planning scene dynamically, at any location."""
        pose = PoseStamped()
        pose.header.frame_id = self._world_frame
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]
        pose.pose.orientation.w = 1.0

        self._scene.add_box(name, pose, size)

    def remove_world_object(self, name: str):
        """Remove boxes from the planning scene dynamically."""
        self._scene.remove_world_object(name)

    def attach_box(
        self,
        name: str,
    ):
        """Attach collision objects to the robot's end-effector."""
        self._scene.attach_box(
            self._ee_link, name, touch_links=[self._ee_link]
        )

    def detach_box(self, name: str):
        """Detach collision objects from the robot's end-effector."""
        self._scene.remove_attached_object(self._ee_link, name)

    def load_scene(self, params: Dict) -> None:
        """Load a planning scene from parameters."""
        for box in params.get('boxes', []):
            self.add_box(
                name=box['name'],
                size=tuple(box['lwh']),
                position=tuple(box['xyz']),
                frame=box.get('frame', None),
            )
