#!/usr/bin/env python3
"""Rebuild Tomato's checked-in MuJoCo model and convex collision assets offline.

The ROS runtime needs only the generated MJCF and assets. Rebuilding requires
numpy, scipy, trimesh, networkx, fast-simplification, coacd and PyYAML in a
separate Python environment; no dependency is installed by this script.
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET


def numbers(values):
    return ' '.join(f'{float(value):.12g}' for value in values)


def quaternion(rpy):
    r, p, y = (float(v) / 2 for v in rpy)
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return (cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy,
            cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy)


def origin(element):
    return {'pos': element.get('xyz', '0 0 0'),
            'quat': numbers(quaternion(element.get('rpy', '0 0 0').split()))}


def make_collision_assets(description, destination, only_links=None):
    """Decompose individual CAD solids, never the complete articulated link hull.

    Screw threads and enclosed motor cavities are simplified to the solid's hull;
    structural concavities and the fixed cradle are decomposed. The existing
    clearance model is the input, and the final home pose must be contact checked.
    """
    import coacd
    import numpy as np
    import trimesh
    from scipy.spatial import QhullError

    coacd.set_log_level('warn')
    destination.mkdir(parents=True, exist_ok=True)
    source = ET.parse(description / 'urdf/tomato_picker.urdf').getroot()
    for link in source.findall('link'):
        name = link.get('name')
        if only_links and name not in only_links:
            continue
        mesh = link.find('collision/geometry/mesh')
        if mesh is None:
            continue
        source_file = description / mesh.get('filename').split('tomato_picker_description/', 1)[1]
        if name == 'base_link':
            source_file = description / 'meshes/simulation/base_link_collision.stl'
        parts = trimesh.load_mesh(source_file).split(only_watertight=False)
        output = []
        for part_index,part in enumerate(parts):
            # Retain every meaningful CAD solid; omit submillimetre thread skins.
            if len(part.faces) < 4 or np.min(part.extents) < 0.0004:
                continue
            try:
                hull = part.convex_hull
            except QhullError:
                continue
            if hull.volume < 3e-9:
                continue
            if part.volume < 0:
                part.invert()
            fraction = abs(part.volume) / max(hull.volume, 1e-20)
            # Individual thin covers, circuit boards, threads and the pinion do
            # not define the gripper's opening or the fixed support's reliefs.
            # Keep them as solid convex parts instead of spending hundreds of
            # contacts on internal screw holes. Structural jaw/L frames retain
            # their concavity through CoACD below.
            thin_cover = np.min(part.extents) < .012 and np.max(part.extents) < .1
            if fraction > .87 or hull.volume < 8e-8 or thin_cover or name == 'Link7':
                output.append(hull)
                continue
            print(f'{name}: decompose CAD solid {part_index} ({len(part.faces)} faces)',flush=True)
            if len(part.faces) > 3500:
                part = part.simplify_quadric_decimation(face_count=3500)
            pieces = coacd.run_coacd(
                coacd.Mesh(np.asarray(part.vertices, dtype=np.float64),
                           np.asarray(part.faces, dtype=np.int32)),
                threshold=.0003, real_metric=True,
                preprocess_mode='off' if part.is_watertight else 'auto', preprocess_resolution=80,
                resolution=2000, mcts_nodes=8, mcts_iterations=15,
                max_convex_hull=32, merge=True, seed=0)
            output.extend(trimesh.Trimesh(vertices=v, faces=f, process=False) for v, f in pieces)
        if not output:
            raise RuntimeError(f'No collision solids generated for {name}')
        for old in destination.glob(f'{name}__*.obj'):
            old.unlink()
        for index, part in enumerate(output):
            part.export(destination / f'{name}__{index:03d}.obj', include_normals=False,
                        include_color=False, include_texture=False)
        print(f'{name}: {len(parts)} CAD solids -> {len(output)} convex collision solids', flush=True)


def generate(description, output, scene_file, handeye_file):
    import yaml

    source = ET.parse(description / 'urdf/tomato_picker.urdf').getroot()
    scene = yaml.safe_load(scene_file.read_text())
    handeye = yaml.safe_load(handeye_file.read_text())
    root = ET.Element('mujoco', model='tomato_picker_harvest')
    ET.SubElement(root, 'compiler', angle='radian', autolimits='true',
                  inertiafromgeom='false', balanceinertia='false', fusestatic='false')
    ET.SubElement(root, 'option', timestep='0.001', gravity='0 0 -9.81',
                  integrator='implicitfast', solver='Newton', iterations='80',
                  tolerance='1e-9', cone='elliptic', impratio='10')
    ET.SubElement(root, 'size', njmax='6000', nconmax='1500')
    default = ET.SubElement(root, 'default')
    ET.SubElement(default, 'joint', armature='0.002', damping='0.1')
    ET.SubElement(default, 'geom', friction='1.2 0.01 0.001', condim='4',
                  solref='0.008 1', solimp='0.95 0.99 0.001', margin='0.00001')
    visual = ET.SubElement(default, 'default', {'class': 'visual'})
    ET.SubElement(visual, 'geom', type='mesh', contype='0', conaffinity='0',
                  group='2', density='0', rgba='.70 .72 .74 1')
    collision = ET.SubElement(default, 'default', {'class': 'collision'})
    ET.SubElement(collision, 'geom', type='mesh', group='3', rgba='.3 .6 .3 .15')
    asset = ET.SubElement(root, 'asset')
    body_root = ET.SubElement(root, 'worldbody')
    ET.SubElement(body_root, 'light', pos='0 -1 2', dir='0 .4 -1', directional='true')
    ET.SubElement(body_root, 'geom', name='ground', type='plane', size='2 2 .05',
                  pos='0 0 -0.0001', rgba='.2 .24 .2 1')
    links = {link.get('name'): link for link in source.findall('link')}
    child_joints = {}
    for joint in source.findall('joint'):
        child_joints.setdefault(joint.find('parent').get('link'), []).append(joint)
    actuators = []

    def append_link(name, parent, pose=None):
        body = ET.SubElement(parent, 'body', name=name, **(pose or {}))
        link = links[name]
        inertia = link.find('inertial')
        if inertia is not None:
            tensor = inertia.find('inertia')
            # The CAD inertial frames all have zero RPY; fullinertia already
            # describes the tensor in that frame and MJCF disallows a quat too.
            inertial_origin = inertia.find('origin')
            if any(abs(float(v)) > 1e-12 for v in inertial_origin.get('rpy','0 0 0').split()):
                raise ValueError(f'Nonzero inertial RPY requires tensor rotation for {name}')
            ET.SubElement(body, 'inertial', pos=inertial_origin.get('xyz','0 0 0'),
                          mass=inertia.find('mass').get('value'),
                          fullinertia=numbers(float(tensor.get(k)) for k in ('ixx','iyy','izz','ixy','ixz','iyz')))
        geometry = link.find('visual/geometry/mesh')
        if geometry is not None:
            filename = geometry.get('filename').split('tomato_picker_description/', 1)[1]
            original = (description/filename).read_bytes()
            face_count = struct.unpack_from('<I', original, 80)[0]
            # MuJoCo's STL decoder rejects a file with >200,000 triangles.
            # Split the exact binary face stream; do not decimate CAD visuals.
            visual_files = [f'../../tomato_picker_description/{filename}']
            if face_count > 200000:
                visual_files = []
                directory = output.parent/'assets/visual'
                directory.mkdir(parents=True,exist_ok=True)
                for index,start in enumerate(range(0,face_count,150000)):
                    count = min(150000,face_count-start)
                    chunk = directory/f'{name}__{index}.stl'
                    chunk.write_bytes(original[:80]+struct.pack('<I',count)+
                                      original[84+50*start:84+50*(start+count)])
                    visual_files.append(f'assets/visual/{chunk.name}')
            for index,visual_file in enumerate(visual_files):
                mesh_name = f'{name}_visual_{index}'
                ET.SubElement(asset, 'mesh', name=mesh_name,file=visual_file)
                ET.SubElement(body, 'geom', {'class':'visual','mesh':mesh_name})
        for index, path in enumerate(sorted((output.parent/'assets/collision').glob(f'{name}__*.obj'))):
            mesh_name = f'{name}_collision_{index}'
            ET.SubElement(asset, 'mesh', name=mesh_name,
                          file=f'assets/collision/{path.name}')
            ET.SubElement(body, 'geom', {'class':'collision', 'name':mesh_name,
                                       'mesh':mesh_name,
                                       **({'friction':'2.0 .02 .002','condim':'6'} if name.startswith('Link7-') else {}),
                                       # CAD cradle solids overlap by <=0.196 mm.
                                       # A 0.21 mm base clearance retains friction
                                       # and every collision beyond that depth.
                                       # MuJoCo >=3.12 sums geom margin/gap;
                                       # other geoms have margin +0.00001.
                                       **({'margin':'-0.00022','gap':'0.00023'} if name == 'base_link' else {})})
        if name == 'tip':
            ET.SubElement(body, 'site', name='tip', size='.002', rgba='0 1 1 1')
        if name == 'tool0':
            ET.SubElement(body, 'site', name='tool0', size='.002')
            camera = ET.SubElement(body, 'body', name='camera_color_optical_frame',
                                   pos=numbers(handeye['translation_m']),
                                   quat=numbers([handeye['quaternion_xyzw'][3], *handeye['quaternion_xyzw'][:3]]))
            ET.SubElement(camera, 'site', name='camera_color_optical_frame', size='.001')
            # OpenGL camera points along -Z with +Y up; ROS optical +Z, +Y down.
            ET.SubElement(camera, 'camera', name='harvest_camera', quat='0 1 0 0',
                          fovy='66.3', resolution='640 400')
        for joint in child_joints.get(name, []):
            joint_name = joint.get('name')
            child_pose = origin(joint.find('origin'))
            if joint_name == 'tip_joint':
                # The exported tip lies ahead of both CAD contact strips.
                # This physical grasp centre holds a 50 mm sphere between
                # their broad faces rather than squeezing it off both tips.
                child_pose['pos'] = '-0.0039 0.0083 0.066'
            child = append_link(joint.find('child').get('link'), body, child_pose)
            if joint.get('type') == 'fixed':
                continue
            limit = joint.find('limit')
            bounds = [float(limit.get('lower')), float(limit.get('upper'))]
            if joint_name in ('joint7-1','joint7-2'):
                bounds = [-.009,.042]
            ET.SubElement(child, 'joint', name=joint_name,
                          type='slide' if joint.get('type')=='prismatic' else 'hinge',
                          axis=joint.find('axis').get('xyz'), range=numbers(bounds),
                          # Gear inertia/damping reflect into slide coordinates
                          # divided by the 4.3 mm pinion radius squared.
                          armature=('1e-7' if joint_name=='joint7' else
                                    '.001' if joint.get('type')=='prismatic' else '.01'),
                          damping=('1e-5' if joint_name=='joint7' else
                                   '2' if joint.get('type')=='prismatic' else '.2'))
            if joint_name in ('joint1','joint2','joint3','joint4','joint5','joint6','joint7-1'):
                effort = float(limit.get('effort'))
                # Finite joint stiffness holds the loaded TCP within 0.5 mm
                # under gravity while retaining the CAD motor effort limits.
                gains = {'joint1':9000,'joint2':15000,'joint3':15000,
                         'joint4':6000,'joint5':3600,'joint6':2400,'joint7-1':6000}
                actuators.append(dict(name=joint_name, joint=joint_name,
                                      kp=str(gains[joint_name]), dampratio='1',
                                      ctrlrange=numbers(bounds), forcerange=numbers([-effort,effort])))
        return body

    append_link('base_link', body_root)
    for index in range(1,4):
        pos = numbers([.3, -.1+.055*(index-1), .35])
        ET.SubElement(body_root, 'body', name=f'stem_{index}', mocap='true', pos=pos)
        fruit = ET.SubElement(body_root, 'body', name=f'fruit_{index}', pos=pos)
        ET.SubElement(fruit, 'freejoint', name=f'fruit_{index}_freejoint')
        ET.SubElement(fruit, 'inertial', pos='0 0 0', mass='.06',
                      diaginertia='.000015 .000015 .000015')
        ET.SubElement(fruit, 'geom', name=f'fruit_geom_{index}', type='sphere',
                      size='.025', rgba='.85 .06 .025 1', friction='2 .02 .002',condim='6')
    basket = scene['basket']
    x,y,z = basket['position']
    dx,dy,dz = basket['dimensions']
    ET.SubElement(body_root, 'geom', name='basket_floor', type='box',
                  pos=numbers([x,y,z]), size=numbers([dx/2,dy/2,dz/2]), rgba='.15 .45 .20 1')
    thickness = float(basket.get('wall_thickness_m',.004))
    height = float(basket.get('wall_height_m',.025))
    for name, px,py,sx,sy in (
        ('left',x-dx/2-thickness/2,y,thickness/2,dy/2+thickness),
        ('right',x+dx/2+thickness/2,y,thickness/2,dy/2+thickness),
        ('front',x,y-dy/2-thickness/2,dx/2,thickness/2),
        ('rear',x,y+dy/2+thickness/2,dx/2,thickness/2)):
        ET.SubElement(body_root, 'geom', name=f'basket_{name}', type='box',
                      pos=numbers([px,py,z+dz/2+height/2]),
                      size=numbers([sx,sy,height/2]), rgba='.15 .45 .20 1')
    for box in scene.get('boxes', []):
        if box.get('enabled',True):
            ET.SubElement(body_root,'geom',name=box['id'],type='box',
                          pos=numbers(box['position']),
                          size=numbers(value/2 for value in box['dimensions']),rgba='.25 .5 .25 1')
    equality = ET.SubElement(root, 'equality')
    ET.SubElement(equality, 'joint', name='pinion_rack', joint1='joint7', joint2='joint7-1',
                  polycoef='0 -232.558139535 0 0 0', solref='.002 1', solimp='.999 .9999 .0001')
    ET.SubElement(equality, 'joint', name='parallel_fingers', joint1='joint7-2', joint2='joint7-1',
                  polycoef='0 1 0 0 0', solref='.002 1', solimp='.999 .9999 .0001')
    for index in range(1,4):
        ET.SubElement(equality, 'weld', name=f'stem_weld_{index}',
                      body1=f'fruit_{index}', body2=f'stem_{index}',
                      relpose='0 0 0 1 0 0 0', solref='.002 1')
    contact = ET.SubElement(root, 'contact')
    exclusions = [('base_link','Link1'),('Link1','Link2'),('Link2','Link3'),
                  ('Link3','Link4'),('Link4','Link5'),('Link5','Link6'),
                  ('Link6','Link7'),('Link6','Link7-1'),('Link6','Link7-2'),
                  ('Link7','Link7-1'),('Link7','Link7-2')]
    for left,right in exclusions:
        ET.SubElement(contact,'exclude',body1=left,body2=right)
    actuator = ET.SubElement(root, 'actuator')
    for attributes in sorted(actuators,key=lambda item:item['name']):
        ET.SubElement(actuator, 'position', **attributes)
    ET.SubElement(root,'statistic',center='.1 -.05 .25',extent='.9')
    ET.indent(root, space='  ')
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(output, encoding='utf-8', xml_declaration=True)
    print(output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    project = Path(__file__).resolve().parents[2]
    parser.add_argument('--description-dir',type=Path,default=project/'serial_arm_core_profile/tomato_picker/tomato_picker_description')
    parser.add_argument('--scene-file',type=Path,required=True)
    parser.add_argument('--handeye-file',type=Path,required=True)
    parser.add_argument('--output',type=Path,default=Path(__file__).resolve().parents[1]/'mjcf/tomato_picker.xml')
    parser.add_argument('--rebuild-collisions',action='store_true')
    parser.add_argument('--only-links',nargs='+',help='Only rebuild these collision links')
    parser.add_argument('--collision-only',action='store_true')
    args = parser.parse_args()
    if args.rebuild_collisions:
        make_collision_assets(args.description_dir,args.output.parent/'assets/collision',args.only_links)
    if args.collision_only:
        return
    generate(args.description_dir,args.output,args.scene_file,args.handeye_file)


if __name__ == '__main__':
    main()
