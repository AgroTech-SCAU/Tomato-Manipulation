"""Derive the kinematic simulation model without modifying the production model.

The complete base block remains fixed; its circular reliefs receive
100/250 micrometre clearance and its rear contact plane receives 1 micrometre
Maximum saved-STL vertex displacement is 0.250006 mm; visual meshes stay unchanged
Source base_link.STL SHA256: cf870da98a788b15d358a1a4c6dbc9c64bbb0c822778590e204cd77119c811eb
"""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


source, destination = map(Path, sys.argv[1:])
robot = ET.parse(source)
mesh = robot.getroot().find("link[@name='base_link']/collision/geometry/mesh")
mesh.set('filename', 'package://tomato_picker_description/meshes/simulation/base_link_collision.stl')
# Center the simulation TCP between the asymmetric CAD finger surfaces for a
# 50 mm fruit; preserve the production TCP and real motor calibration
robot.getroot().find("joint[@name='tip_joint']/origin").set('xyz', '-0.00254842 0 0.101')
for name in ('joint7-1', 'joint7-2'):
    robot.getroot().find(f"joint[@name='{name}']/limit").set('lower', '-0.0073')
robot.write(destination, encoding='utf-8', xml_declaration=True)
