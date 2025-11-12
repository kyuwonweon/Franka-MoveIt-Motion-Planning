"""Planning scene wrapper. TODO: improve docstrings."""

from typing import Dict, Tuple

from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from moveit_msgs.msg import PlanningScene as PS


class PlanningScene:
    """
    A wrapper class to manage collision objects in the planning scene.

    This class provides methods for adding, removing, attaching, and detaching
    objects to/from the planning scene using MoveIt's PlanningSceneInterface.
    """

    def __init__(
        self,
        node: Node,
        world_frame: str = 'base',
        ee_link: str = 'fer_hand_tcp',
        planning_scene_topic: str = '/planning_scene',
    ) -> None:
        """Initialize the planning scene interface."""
        self._node = node
        self._frame = world_frame
        self._ee_link = ee_link
        self._scene_pub = node.create_publisher(PS, planning_scene_topic, 10)

    def add_box(
        self,
        name: str,
        size: Tuple[float, float, float],
        position: Tuple[float, float, float],
    ) -> None:
        """Add boxes to the planning scene dynamically, at any location."""
        lx, ly, lz = size
        px, py, pz = position

        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [lx, ly, lz]

        pose = PoseStamped()
        pose.header.frame_id = self._frame
        pose.pose.position.x = px
        pose.pose.position.y = py
        pose.pose.position.z = pz
        pose.pose.orientation.w = 1.0

        co = CollisionObject()
        co.header.frame_id = self._frame
        co.id = name
        co.primitives = [prim]
        co.primitive_poses = [pose.pose]
        co.operation = CollisionObject.ADD

        scene = PS()
        scene.is_diff = True
        scene.world.collision_objects = [co]
        self._scene_pub.publish(scene)

    def remove_world_object(self, name: str) -> None:
        """Remove boxes from the planning scene dynamically."""
        co = CollisionObject()
        co.header.frame_id = self._frame
        co.id = name
        co.operation = CollisionObject.REMOVE

        scene = PS()
        scene.is_diff = True
        scene.world.collision_objects = [co]
        self._scene_pub.publish(scene)

    def attach_box(
        self,
        name: str,
        size: Tuple[float, float, float],
        ee_pose_in_ee: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        """Attach a box to EE; remove the world object in one diff."""
        lx, ly, lz = size
        ox, oy, oz = ee_pose_in_ee

        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [lx, ly, lz]

        world_remove = CollisionObject()
        world_remove.header.frame_id = self._frame
        world_remove.id = name
        world_remove.operation = CollisionObject.REMOVE

        aco = AttachedCollisionObject()
        aco.link_name = self._ee_link
        aco.touch_links = [self._ee_link]
        aco.object.id = name
        aco.object.header.frame_id = self._ee_link
        aco.object.operation = CollisionObject.ADD
        aco.object.primitives = [prim]

        pose = PoseStamped()
        pose.header.frame_id = self._ee_link
        pose.pose.position.x = ox
        pose.pose.position.y = oy
        pose.pose.position.z = oz
        pose.pose.orientation.w = 1.0
        aco.object.primitive_poses = [pose.pose]

        scene = PS()
        scene.is_diff = True
        scene.world.collision_objects = [world_remove]
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [aco]
        self._scene_pub.publish(scene)

    def detach_box(
        self,
        name: str,
        size: Tuple[float, float, float],
        world_pose_xyz: Tuple[float, float, float],
    ) -> None:
        """Detach from EE and add back to world in a single diff."""
        lx, ly, lz = size
        px, py, pz = world_pose_xyz

        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [lx, ly, lz]

        aco = AttachedCollisionObject()
        aco.link_name = self._ee_link
        aco.object.id = name
        aco.object.operation = CollisionObject.REMOVE

        pose = PoseStamped()
        pose.header.frame_id = self._frame
        pose.pose.position.x = px
        pose.pose.position.y = py
        pose.pose.position.z = pz
        pose.pose.orientation.w = 1.0

        co = CollisionObject()
        co.header.frame_id = self._frame
        co.id = name
        co.primitives = [prim]
        co.primitive_poses = [pose.pose]
        co.operation = CollisionObject.ADD

        scene = PS()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [aco]
        scene.world.collision_objects = [co]
        self._scene_pub.publish(scene)

    def load_scene(self, params: Dict) -> None:
        """Load a planning scene from parameters."""
        for box in params.get('boxes', []):
            self.add_box(
                name=str(box['name']),
                size=tuple(box['lwh']),
                position=tuple(box['xyz']),
            )
