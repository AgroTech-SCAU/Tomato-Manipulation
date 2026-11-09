#!/usr/bin/env python3
"""Derive physics joint travel from the calibrated RViz description."""
import argparse
import math
import yaml
from pathlib import Path
import xml.etree.ElementTree as ET

parser = argparse.ArgumentParser()
parser.add_argument('source', type=Path)
parser.add_argument('output', type=Path)
parser.add_argument('calibration', type=Path)
args = parser.parse_args()
root = ET.parse(args.source)
for name in ('joint7-1', 'joint7-2'):
    root.find(f"./joint[@name='{name}']/limit").set('lower', '-0.009')
# The CAD-exported tip lies beyond the pads; use the verified physical grasp center
root.find("./joint[@name='tip_joint']/origin").set('xyz', '-0.0039 0.0083 0.066')
calibration = yaml.safe_load(args.calibration.read_text())
rotation = calibration['transform_matrix']
rpy = [math.atan2(rotation[2][1], rotation[2][2]),
       math.asin(-rotation[2][0]), math.atan2(rotation[1][0], rotation[0][0])]
frame = calibration['child_frame']
ET.SubElement(root.getroot(), 'link', name=frame)
joint = ET.SubElement(root.getroot(), 'joint', name='mujoco_camera_joint', type='fixed')
ET.SubElement(joint, 'parent', link=calibration['parent_frame'])
ET.SubElement(joint, 'child', link=frame)
ET.SubElement(joint, 'origin',
              xyz=' '.join(str(row[3]) for row in rotation[:3]),
              rpy=' '.join(str(value) for value in rpy))
args.output.parent.mkdir(parents=True, exist_ok=True)
root.write(args.output, encoding='utf-8', xml_declaration=True)
