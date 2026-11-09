"""Calculations for the robot's simulated multi-fruit harvest workflow."""

from copy import deepcopy
import math

from .servo_operations import finite, matrix_quaternion, stamp_ns


def phase_reset(payload, stop, *, context):
    """Start a new servo phase with an independent movement and sample budget."""
    return {'tracking': {}, 'attempt': 0, 'refresh_reason': ''}


def prepare_harvest(payload, stop, *, context):
    """Validate the simulation response and reset association for the next fruit."""
    scene = payload['scene']
    if scene.get('success') is not True:
        raise ValueError(scene.get('message') or '仿真采摘场景准备失败')
    result = {
        'scene': scene,
        'fruit_index': int(scene['fruit_index']),
        'total_fruits': int(scene['total_fruits']),
        'harvested_count': int(scene['harvested_count']),
        'finished': scene['finished'],
    }
    if (result['total_fruits'] < 1 or not 0 <= result['harvested_count'] <= result['total_fruits']
            or type(result['finished']) is not bool
            or result['finished'] != (result['harvested_count'] == result['total_fruits'])):
        raise ValueError('仿真采摘进度无效')
    if not 0 <= result['fruit_index'] <= result['total_fruits']:
        raise ValueError('当前果实编号无效')
    result.update(phase_reset(payload, stop, context=context))
    return result


def motion_goal(payload, stop, *, context):
    """Read a scene waypoint and measure its distance from the actual TCP."""
    pose_key = payload['pose_key']
    if pose_key not in ('home_pose', 'retreat_pose', 'place_pose'):
        raise ValueError('采摘路径位姿名称无效')
    goal = deepcopy(payload['scene'][pose_key])
    frame = goal['header']['frame_id']
    if not isinstance(frame, str) or not frame or frame != payload.get('base_frame', frame):
        raise ValueError('采摘路径与基座坐标系不一致')
    position = {axis: finite(goal['pose']['position'][axis], '路径位置') for axis in ('x', 'y', 'z')}
    if pose_key == 'retreat_pose' and 'min_retreat_height_m' in payload:
        position['z'] = max(position['z'], finite(payload['min_retreat_height_m'], '撤离通行高度'))
    orientation = {axis: finite(goal['pose']['orientation'][axis], '路径旋转') for axis in ('x', 'y', 'z', 'w')}
    matrix_quaternion(orientation)
    goal['header']['stamp'] = {'sec': 0, 'nanosec': 0}
    goal['pose']['position'] = position
    goal['pose']['orientation'] = orientation
    tf = context.transform(frame, payload.get('tcp_frame', 'tip'))
    age = (context.now() - stamp_ns(tf['header'])) / 1e9
    if not 0 <= age <= finite(payload.get('max_pose_age_s', 1), '工具位姿有效期'):
        raise ValueError('当前工具位姿已过期')
    actual = tf['transform']['translation']
    remaining = math.sqrt(sum((position[axis] - finite(actual[axis], '工具位置')) ** 2 for axis in ('x', 'y', 'z')))
    return {'goal': goal, 'remaining_distance': remaining, 'pose_key': pose_key}


def _basket_slot(payload):
    index = payload['scene']['fruit_index']
    count = len(payload['joints_by_slot'])
    order = payload.get('harvest_order', list(range(1, count + 1)))
    if (not isinstance(order, list) or any(type(value) is not int for value in order) or
            sorted(order) != list(range(1, count + 1)) or type(index) is not int or index not in order):
        raise ValueError('采摘顺序须恰好包含每个果实的编号')
    return order.index(index) + 1


def basket_approach_goal(payload, stop, *, context):
    """Select the verified basket-entry joints and the vertical exit pose."""
    index = _basket_slot(payload)
    rows = payload['joints_by_slot']
    if type(index) is not int or not 1 <= index <= len(rows):
        raise ValueError('果篮槽位编号无效')
    joints = rows[index - 1]
    if not isinstance(joints, list) or len(joints) != 6:
        raise ValueError('果篮上方关节目标须包含六轴')
    joints = [finite(value, '果篮上方关节角') for value in joints]
    lift = finite(payload['lift_m'], '果篮抬升距离')
    if lift <= 0:
        raise ValueError('果篮抬升距离须为正数')
    goal = motion_goal({**payload, 'pose_key': 'place_pose'}, stop, context=context)['goal']
    goal['pose']['position']['z'] += lift
    return {'joints': joints, 'goal': goal}


def continuous_transport_goal(payload, stop, *, context):
    """Build contact-preserving carry and empty-return plans for this fruit."""
    if stop.is_set():
        raise RuntimeError('连续搬运准备已取消')
    retreat = motion_goal({**payload, 'pose_key': 'retreat_pose'}, stop, context=context)
    basket = basket_approach_goal(payload, stop, context=context)
    place = motion_goal({**payload, 'pose_key': 'place_pose'}, stop, context=context)
    group, link = payload['planning_group'], payload['tcp_frame']

    def segment(planner, constraints, limits, radius):
        velocity = finite(limits['velocity_scale'], '连续运动速度缩放')
        acceleration = finite(limits['acceleration_scale'], '连续运动加速度缩放')
        if not 0 < velocity <= 1 or not 0 < acceleration <= 1:
            raise ValueError('连续运动缩放应在 (0, 1] 内')
        return {'req': {'pipeline_id': 'pilz_industrial_motion_planner', 'planner_id': planner,
                        'group_name': group, 'num_planning_attempts': 1, 'allowed_planning_time': 5.,
                        'max_velocity_scaling_factor': velocity, 'max_acceleration_scaling_factor': acceleration,
                        'goal_constraints': [constraints]}, 'blend_radius': radius}

    def linear(goal, limits, radius):
        header = {'frame_id': goal['header']['frame_id']}
        constraint = {
            'position_constraints': [{'header': header, 'link_name': link, 'weight': 1.,
                'constraint_region': {'primitives': [{'type': 2, 'dimensions': [.0001]}],
                    'primitive_poses': [{'position': goal['pose']['position'], 'orientation': {'w': 1.}}]}}],
            'orientation_constraints': [{'header': header, 'link_name': link, 'weight': 1.,
                'orientation': goal['pose']['orientation'], 'absolute_x_axis_tolerance': .001,
                'absolute_y_axis_tolerance': .001, 'absolute_z_axis_tolerance': .001}]}
        return segment('LIN', constraint, limits, radius)

    def joint(values, limits, radius):
        names = payload['joint_names']
        if len(names) != len(values):
            raise ValueError('连续关节目标数量与关节名称不一致')
        constraints = {'joint_constraints': [{'joint_name': name, 'position': finite(value, '连续关节角'),
                        'tolerance_above': .0001, 'tolerance_below': .0001, 'weight': 1.}
                       for name, value in zip(names, values)]}
        return segment('PTP', constraints, limits, radius)

    radius = finite(payload['carry_blend_radius_m'], '携果过渡半径')
    empty_radius = finite(payload['return_blend_radius_m'], '空返过渡半径')
    if min(radius, empty_radius) < 0:
        raise ValueError('连续运动过渡半径不能为负')
    carry_limits, return_limits = payload['carry_limits'], payload['return_limits']
    carry = [linear(retreat['goal'], carry_limits['retreat'], radius),
             joint(basket['joints'], carry_limits['turn'], radius),
             linear(place['goal'], carry_limits['place'], 0.)]
    empty = [linear(basket['goal'], return_limits['lift'], empty_radius)]
    index = _basket_slot(payload)
    for waypoint in payload.get('return_joint_waypoints_by_slot', {}).get(str(index), []):
        empty.append(joint(waypoint, return_limits['return'], empty_radius))
    scene = payload['scene']
    harvested, total = scene['harvested_count'], scene['total_fruits']
    if type(harvested) is not int or type(total) is not int or not 0 <= harvested < total:
        raise ValueError('连续回程的采摘进度无效')
    last_fruit = harvested + 1 == total
    home_radius = finite(payload['final_home_blend_radius_m'], '回零过渡半径') if last_fruit else 0.
    if home_radius < 0:
        raise ValueError('回零过渡半径不能为负')
    empty.append(joint(payload['observation_joints'], return_limits['return'], home_radius))
    if last_fruit:
        empty.append(joint(payload['home_joints'], return_limits['home'], 0.))
    carry[0]['req']['start_state'] = {'is_diff': True}
    empty[0]['req']['start_state'] = {'is_diff': True}
    if stop.is_set():
        raise RuntimeError('连续搬运准备已取消')
    return {'carry_request': {'items': carry}, 'return_request': {'items': empty},
            'basket_approach_pose': basket['goal'], 'remaining_distance': retreat['remaining_distance'],
            'continuous_return': index in payload['return_sequence_slots']}
