"""Tomato visual-servo calculations executed in the robot workspace.

The runtime supplies a context with ROS-clock now() and timestamped transform().
Fixed hand-eye extrinsics, target association and approach limits belong here.
"""
import math
from pathlib import Path


def finite(value, name):
    result = float(value)
    if not math.isfinite(result): raise ValueError(name+' 必须为有限数值')
    return result


def matrix_quaternion(q):
    import numpy as np
    values=np.array([q[k] for k in ('x','y','z','w')],dtype=float)
    norm=np.linalg.norm(values)
    if not np.isfinite(values).all() or abs(norm-1)>.01:raise ValueError('旋转四元数无效')
    x,y,z,w=values/norm
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def stamp_ns(header):
    stamp=header['stamp']
    return int(stamp['sec'])*1000000000+int(stamp['nanosec'])


def refresh_attempt(payload, reason):
    attempt = int(payload.get('attempt', 0)) + 1
    limit = int(payload.get('max_refresh_attempts', 5))
    if not payload.get('allow_refresh'):
        raise ValueError(reason)
    if attempt > limit:
        raise ValueError(f'观测刷新已达到 {limit} 次上限：{reason}')
    return attempt


def transform_targets(payload,stop,*,context):
    import numpy as np
    import yaml
    data=payload['observation']
    def refresh(reason):
        return {'target':{},'ready':False,'attempt':refresh_attempt(payload,reason),'refresh_reason':reason}
    if not data.get('success',True):
        reason=data.get('message') or '观测失败'
        if data.get('status_code') in payload.get('refresh_status_codes',[]):return refresh(reason)
        raise ValueError(reason)
    header=data['header'];stamp=stamp_ns(header)
    age=(context.now()-stamp)/1e9
    if stamp<=0 or age<0:raise ValueError('观测时间戳无效')
    if stamp<int(payload.get('after_ns',0)):
        return refresh('观测图像早于本次静止确认，需要重新观测')
    if age>finite(payload.get('max_age_s',.5),'观测有效期'):return refresh('观测数据已过期')
    calibration=yaml.safe_load(Path(payload['calibration_file']).read_text())
    parent=calibration['parent_frame'];camera=calibration['child_frame']
    if camera!=header['frame_id'] or parent!=payload['tool_frame'] or calibration.get('stored_transform_convention')!='camera_to_gripper (^gripper T_camera)':
        raise ValueError('标定方向、工具坐标系或观测坐标系不一致')
    matrix=np.asarray(calibration['transform_matrix'],dtype=float)
    if matrix.shape!=(4,4) or not np.isfinite(matrix).all() or not np.allclose(matrix[3],[0,0,0,1],atol=1e-8) or not np.allclose(matrix[:3,:3].T@matrix[:3,:3],np.eye(3),atol=1e-6) or not np.isclose(np.linalg.det(matrix[:3,:3]),1,atol=1e-6):raise ValueError('标定矩阵不是有效刚体变换')
    tf=context.transform(payload['base_frame'],parent,header['stamp'])['transform']
    transform=np.eye(4);transform[:3,:3]=matrix_quaternion(tf['rotation']);transform[:3,3]=[tf['translation'][k] for k in ('x','y','z')]
    matrix=transform@matrix
    tracking=payload.get('tracking') or {};previous=tracking.get('target');label=tracking.get('label',payload.get('label',''))
    candidates=[]
    for target in data.get('targets',[]):
        xyz=np.asarray([target['position'][k] for k in ('x','y','z')]+[1.],dtype=float)
        confidence=finite(target['confidence'],'置信度')
        if not np.isfinite(xyz).all() or xyz[2]<=0 or confidence<payload.get('min_confidence',0):continue
        if label and target.get('semantic_label','').lower()!=label.lower():continue
        point=(matrix@xyz)[:3]
        score=float(np.linalg.norm(point-np.asarray(previous))) if previous is not None else -confidence
        candidates.append((score,point.tolist(),target.get('semantic_label','')))
    if not candidates:return refresh('没有满足条件的空间目标')
    if previous is not None and payload.get('require_unique_association'):
        nearby=[item for item in candidates if item[0]<=payload.get('max_target_jump_m',.03)]
        if len(nearby)>1:return refresh('关联门限内存在多个候选目标，需要重新观测')
    score,point,label=min(candidates,key=lambda item:item[0])
    if previous is not None and score>payload.get('max_target_jump_m',.03):raise ValueError('目标位置跳变，已拒绝切换目标')
    return {'target':{'point':point,'frame_id':payload['base_frame'],'stamp':header['stamp'],'label':label},'ready':True,'attempt':int(payload.get('attempt',0)),'refresh_reason':''}


def bounded_pose_step(payload,stop,*,context):
    import numpy as np
    target=payload['target'];pose=payload['current_pose'];tracking=dict(payload.get('tracking') or {})
    first_target=not tracking
    if payload.get('base_frame') and payload.get('tcp_frame'):
        tf=context.transform(payload['base_frame'],payload['tcp_frame'])
        pose={'header':tf['header'],'pose':{'position':tf['transform']['translation'],'orientation':tf['transform']['rotation']}}
    if target['frame_id']!=pose['header']['frame_id']:raise ValueError('目标与工具位姿坐标系不同')
    target_stamp=stamp_ns({'stamp':target['stamp']})
    target_age=(context.now()-target_stamp)/1e9
    if target_stamp<=0 or target_age<0:raise ValueError('目标时间戳无效')
    if not 0<=(context.now()-stamp_ns(pose['header']))/1e9<=payload.get('max_pose_age_s',1):raise ValueError('当前工具位姿已过期')
    position=np.array([pose['pose']['position'][k] for k in ('x','y','z')]);point=np.array(target['point'])
    if not np.isfinite(position).all() or not np.isfinite(point).all():raise ValueError('目标包含非有限数值')
    if not tracking:tracking={'origin':position.tolist(),'orientation':pose['pose']['orientation'],'travel':0.,'steps':0}
    standoff=finite(payload.get('standoff_m',.1),'安全距离');tolerance=finite(payload.get('tolerance_m',.005),'误差阈值')
    step=finite(payload.get('max_step_m',.01),'单步距离');limit=finite(payload.get('max_travel_m',.25),'总距离')
    if not 0<tolerance<standoff or standoff<payload.get('min_standoff_m',.05) or min(step,limit)<=0:raise ValueError('距离约束不合法')
    direction=matrix_quaternion(tracking['orientation'])[:,2]
    delta=point-direction*standoff-position;distance=float(np.linalg.norm(delta))
    within=distance<=tolerance or math.isclose(distance,tolerance,rel_tol=0,abs_tol=1e-12)
    if target_age>payload.get('max_target_age_s',1):
        attempt=refresh_attempt(payload,'目标位姿已过期')
        return {'goal':{},'tracking':dict(payload.get('tracking') or {}),'done':False,'next_step':'refresh',
                'remaining_distance':distance,'steps':tracking['steps'],'attempt':attempt,'refresh_reason':'目标位姿已过期'}
    if first_target and distance>limit+tolerance:
        raise ValueError('目标停距点超出首次接近的累计运动预算，请先预定位')
    confirmations=payload.get('convergence_samples',1);max_cycles=payload.get('max_cycles',100)
    if type(confirmations) is not int or confirmations<1 or type(max_cycles) is not int or max_cycles<confirmations:raise ValueError('收敛确认次数与观测循环上限不合法')
    sample_stamp=stamp_ns({'stamp':target['stamp']})
    if sample_stamp<=tracking.get('last_sample_ns',0):raise ValueError('观测时间未前进，不能重复使用同一帧确认收敛')
    cycles=tracking.get('cycles',0)+1
    if cycles>max_cycles:raise ValueError('已达到观测循环上限，停止继续伺服')
    stable=tracking.get('stable_samples',0)+1 if within else 0
    done=stable>=confirmations
    if not within and tracking['steps']>=payload.get('max_steps',30):raise ValueError('已达到最大步数，停止继续步进')
    length=0 if within else min(distance,step)
    travel=tracking['travel']+length
    if travel>limit:
        # Trim arithmetic roundoff down to the remaining budget, never raise the limit.
        if tracking['travel']<=limit and math.isclose(travel,limit,rel_tol=0,abs_tol=1e-12):
            length=limit-tracking['travel'];travel=limit
        else:raise ValueError('下一步超过累计运动距离上限')
    next_position=position+(delta/distance*length if distance else 0)
    tracking.update(target=target['point'],label=target['label'],travel=travel,steps=tracking['steps']+(0 if within else 1),last_sample_ns=sample_stamp,cycles=cycles,stable_samples=stable)
    goal={'header':{'frame_id':target['frame_id'],'stamp':{'sec':0,'nanosec':0}},'pose':{'position':dict(zip(('x','y','z'),map(float,next_position))),'orientation':tracking['orientation']}}
    return {'goal':goal,'tracking':tracking,'done':done,'next_step':'complete' if done else 'confirm' if within else 'move','remaining_distance':distance,'steps':tracking['steps'],'attempt':0,'refresh_reason':''}
