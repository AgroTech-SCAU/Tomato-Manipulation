#!/usr/bin/env python3
"""Keep configured obstacles and optional physical fruit in the MoveIt scene."""
from copy import deepcopy
from pathlib import Path
import math

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (AllowedCollisionEntry, AttachedCollisionObject,
                             CollisionObject, PlanningScene, PlanningSceneComponents)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import UInt64
from std_srvs.srv import Trigger
from tomato_picker_interfaces.msg import HarvestPhysicsState
import yaml


def parse_harvest_basket(config):
    """Read the basket contract shared with the simulated harvest scene."""
    if not isinstance(config, dict) or not isinstance(config.get('frame_id'), str) or not config['frame_id'].strip():
        raise ValueError('Harvest scene requires a nonempty frame_id')
    basket = config.get('basket')
    if not isinstance(basket, dict) or not isinstance(basket.get('id'), str) or not basket['id'].strip():
        raise ValueError('Harvest scene requires a basket with a nonempty id')
    result = {'frame_id': config['frame_id'], 'id': basket['id']}
    for key in ('position', 'dimensions'):
        values = basket.get(key)
        if (not isinstance(values, list) or len(values) != 3 or
                any(type(value) not in (int, float) or not math.isfinite(value) for value in values)):
            raise ValueError(f'Basket {key} must contain three finite numbers')
        if key == 'dimensions' and any(value <= 0 for value in values):
            raise ValueError('Basket dimensions must be positive')
        result[key] = [float(value) for value in values]
    if basket.get('slot_axis') not in ('x_axis', 'y_axis'):
        raise ValueError('Basket slot_axis must be x_axis or y_axis')
    result['slot_axis'] = basket['slot_axis']
    orientation = basket.get('release_orientation_xyzw')
    if (not isinstance(orientation, list) or len(orientation) != 4 or
            any(type(value) not in (int, float) or not math.isfinite(value) for value in orientation)):
        raise ValueError('Basket release_orientation_xyzw must contain four finite numbers')
    norm = math.sqrt(sum(value * value for value in orientation))
    if abs(norm - 1.) > .01:
        raise ValueError('Basket release_orientation_xyzw must be normalized')
    result['release_orientation_xyzw'] = [float(value) / norm for value in orientation]
    for key in ('slot_spacing_m', 'release_clearance_m'):
        value = basket.get(key)
        if (type(value) not in (int, float) or not math.isfinite(value) or value < 0 or
                key == 'slot_spacing_m' and value == 0):
            raise ValueError(f'Basket {key} must be finite and {"positive" if key == "slot_spacing_m" else "nonnegative"}')
        result[key] = float(value)
    for key in ('wall_thickness_m', 'wall_height_m'):
        if key in basket:
            value = basket[key]
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'Basket {key} must be finite and positive')
            result[key] = float(value)
    if ('wall_thickness_m' in result) != ('wall_height_m' in result):
        raise ValueError('Basket walls require both thickness and height')
    return result


class SimulationScene(Node):
    CONTACT_LINKS = ('Link6', 'Link7', 'Link7-1', 'Link7-2', 'tool0', 'tip')

    def __init__(self):
        super().__init__('servo_simulation_scene')
        default = str(Path(get_package_share_directory('tomato_picker_bringup')) /
                      'config/servo_simulation/scene.yaml')
        self.declare_parameter('scene_file', default)
        self.declare_parameter('physics_state_topic', '')
        self.declare_parameter('fruit_radius_m', 0.025)
        config = yaml.safe_load(Path(self.get_parameter('scene_file').value).read_text())
        if not isinstance(config, dict):
            raise ValueError('Simulation scene must be a mapping')
        self.scene = PlanningScene(is_diff=True)
        boxes = list(config.get('boxes', []))
        if 'basket' in config:
            basket = parse_harvest_basket(config)
            if any(box.get('id') == basket['id'] for box in boxes):
                raise ValueError('Basket id must differ from other simulation boxes')
            boxes.append(basket)
            if 'wall_thickness_m' in basket:
                x, y, z = basket['position']
                dx, dy, dz = basket['dimensions']
                thickness, height = basket['wall_thickness_m'], basket['wall_height_m']
                for name, px, py, sx, sy in (
                    ('left', x-dx/2-thickness/2, y, thickness, dy+2*thickness),
                    ('right', x+dx/2+thickness/2, y, thickness, dy+2*thickness),
                    ('front', x, y-dy/2-thickness/2, dx, thickness),
                    ('rear', x, y+dy/2+thickness/2, dx, thickness)):
                    boxes.append({'id': basket['id']+'_'+name,
                                  'position': [px, py, z+dz/2+height/2],
                                  'dimensions': [sx, sy, height]})
        for box in boxes:
            item = CollisionObject(id=str(box['id']))
            item.header.frame_id = config.get('frame_id', 'base_link')
            item.operation = CollisionObject.ADD if box.get('enabled', True) else CollisionObject.REMOVE
            if item.operation == CollisionObject.ADD:
                dimensions = [float(v) for v in box['dimensions']]
                position = [float(v) for v in box['position']]
                if len(dimensions) != 3 or len(position) != 3 or any(v <= 0 for v in dimensions):
                    raise ValueError(f'Invalid box configuration: {item.id}')
                item.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=dimensions)]
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = position
                pose.orientation.w = 1.0
                item.primitive_poses = [pose]
            self.scene.world.collision_objects.append(item)
        self.client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.pending = None
        self.ready_service = None
        self.physics_topic = self.get_parameter('physics_state_topic').value
        self.latest_physics = None
        self.pending_revision = None
        self.last_acknowledged_revision = None
        self.applied_fruit_ids = set()
        self.applied_attached_ids = set()
        if self.physics_topic:
            self.fruit_radius = float(self.get_parameter('fruit_radius_m').value)
            if not math.isfinite(self.fruit_radius) or self.fruit_radius <= 0:
                raise ValueError('fruit_radius_m must be finite and positive')
            self.physics_subscription = self.create_subscription(
                HarvestPhysicsState, self.physics_topic, self.receive_physics, 1)
            self.scene_client = self.create_client(GetPlanningScene, '/get_planning_scene')
            self.matrix_pending = None
            self.base_collision_matrix = None
            self.revision_publisher = self.create_publisher(
                UInt64, '/simulation/physics/planning_scene_revision',
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.timer = self.create_timer(0.2, self.apply_scene)

    def receive_physics(self, message):
        previous = self.latest_physics
        self.latest_physics = message
        if previous is None or message.revision != previous.revision:
            self.apply_scene()

    def load_collision_matrix(self):
        if self.base_collision_matrix is not None:
            return True
        if self.matrix_pending is not None:
            if self.matrix_pending.done():
                result = self.matrix_pending.result()
                if result is None:
                    raise RuntimeError('MoveIt did not return its allowed collision matrix')
                self.base_collision_matrix = deepcopy(result.scene.allowed_collision_matrix)
                return True
        elif self.scene_client.service_is_ready():
            request = GetPlanningScene.Request()
            request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
            self.matrix_pending = self.scene_client.call_async(request)
        return False

    def physics_scene(self, snapshot):
        scene = deepcopy(self.scene)
        scene.robot_state.is_diff = True
        fruit_ids = {f'harvest_fruit_{index + 1}' for index in range(len(snapshot.fruit_poses))}
        held_ids = ({f'harvest_fruit_{snapshot.held_index}'}
                    if 0 < snapshot.held_index <= len(snapshot.fruit_poses) else set())
        for fruit_id in self.applied_attached_ids - held_ids:
            attached = AttachedCollisionObject(link_name='tip')
            attached.object.id = fruit_id
            attached.object.operation = CollisionObject.REMOVE
            scene.robot_state.attached_collision_objects.append(attached)
        for fruit_id in self.applied_fruit_ids - fruit_ids:
            scene.world.collision_objects.append(
                CollisionObject(id=fruit_id, operation=CollisionObject.REMOVE))
        for index, pose in enumerate(snapshot.fruit_poses, 1):
            fruit_id = f'harvest_fruit_{index}'
            item = CollisionObject(id=fruit_id, operation=CollisionObject.ADD)
            item.header = deepcopy(snapshot.header)
            item.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE,
                                              dimensions=[self.fruit_radius])]
            item.primitive_poses = [deepcopy(pose)]
            if fruit_id in held_ids:
                # MoveIt transfers the object out of world when attaching it
                scene.robot_state.attached_collision_objects.append(
                    AttachedCollisionObject(link_name='tip', object=item,
                                            touch_links=list(self.CONTACT_LINKS)))
            else:
                scene.world.collision_objects.append(item)

        # Preserve SRDF and existing scene permissions, extending only our fruit rows
        matrix = deepcopy(self.base_collision_matrix)
        for name in (*self.CONTACT_LINKS, *sorted(fruit_ids)):
            if name not in matrix.entry_names:
                matrix.entry_names.append(name)
                for entry in matrix.entry_values:
                    entry.enabled.append(False)
                matrix.entry_values.append(
                    AllowedCollisionEntry(enabled=[False] * len(matrix.entry_names)))
        name_indices = {name: index for index, name in enumerate(matrix.entry_names)}
        for index in range(1, len(snapshot.fruit_poses) + 1):
            fruit_id = f'harvest_fruit_{index}'
            fruit_row = name_indices[fruit_id]
            allow_contact = index in (snapshot.selected_index, snapshot.held_index)
            for link in self.CONTACT_LINKS:
                link_row = name_indices[link]
                matrix.entry_values[fruit_row].enabled[link_row] = allow_contact
                matrix.entry_values[link_row].enabled[fruit_row] = allow_contact
        scene.allowed_collision_matrix = matrix
        return scene, fruit_ids, held_ids

    def scene_applied(self, future):
        if future is not self.pending:
            return
        self.pending = None
        try:
            result = future.result()
        except Exception as error:
            self.get_logger().error(f'MoveIt scene update failed; retrying on the scene timer: {error}')
            return
        if result is None or not result.success:
            self.get_logger().error('MoveIt rejected the configured simulation scene; retrying on the scene timer')
            return
        if self.physics_topic:
            self.applied_fruit_ids = self.pending_fruit_ids
            self.applied_attached_ids = self.pending_attached_ids
            if self.pending_revision != self.last_acknowledged_revision:
                self.revision_publisher.publish(UInt64(data=self.pending_revision))
                self.last_acknowledged_revision = self.pending_revision
        if self.ready_service is None:
            self.get_logger().info('Simulation collision scene applied')
            self.ready_service = self.create_service(
                Trigger, '/tomato_picker/simulation/scene_ready', self.ready)
        if not self.physics_topic:
            self.timer.cancel()
        elif self.latest_physics.revision != self.last_acknowledged_revision:
            self.apply_scene()

    def apply_scene(self):
        if self.pending is not None:
            return
        if self.client.service_is_ready():
            if self.physics_topic:
                if self.latest_physics is None or not self.load_collision_matrix():
                    return
                snapshot = self.latest_physics
                scene, fruit_ids, attached_ids = self.physics_scene(snapshot)
                self.pending_revision = snapshot.revision
                self.pending_fruit_ids = fruit_ids
                self.pending_attached_ids = attached_ids
            else:
                scene = self.scene
            self.pending = self.client.call_async(ApplyPlanningScene.Request(scene=scene))
            self.pending.add_done_callback(self.scene_applied)

    def ready(self, _request, response):
        response.success = True
        response.message = 'Simulation collision scene applied'
        return response


def main():
    rclpy.init()
    node = SimulationScene()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
