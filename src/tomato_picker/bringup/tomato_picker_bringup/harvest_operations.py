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
