#!/usr/bin/env python3
"""Load the configured kinematic simulation obstacles into the real MoveIt scene."""
from pathlib import Path
import math

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger
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
    return result


class SimulationScene(Node):
    def __init__(self):
        super().__init__('servo_simulation_scene')
        default = str(Path(get_package_share_directory('tomato_picker_bringup')) /
                      'config/servo_simulation/scene.yaml')
        self.declare_parameter('scene_file', default)
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
        self.timer = self.create_timer(0.2, self.apply_scene)

    def apply_scene(self):
        if self.pending is not None:
            if not self.pending.done():
                return
            result = self.pending.result()
            if result is None or not result.success:
                raise RuntimeError('MoveIt rejected the configured simulation scene')
            self.get_logger().info('Simulation collision scene applied')
            self.ready_service = self.create_service(
                Trigger, '/tomato_picker/simulation/scene_ready', self.ready)
            self.timer.cancel()
            return
        if self.client.service_is_ready():
            self.pending = self.client.call_async(ApplyPlanningScene.Request(scene=self.scene))

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
