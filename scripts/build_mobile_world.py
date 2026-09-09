#!/usr/bin/env python3
"""Build the separated-station world and a collision-free 20 cm floor grid."""

from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]


def build_world():
    tree = ET.parse(ROOT / 'src/mobile_manipulator/worlds/pick_and_place.sdf')
    world = tree.getroot().find('world')
    world.find("model[@name='place_support']/pose").text = '1.063 0.694 0.070 0 0 1.570796327'
    world.find("model[@name='evidence_camera']/pose").text = '-0.65 -0.85 1.80 0 0.85 0.86'
    camera = world.find("model[@name='evidence_camera']/link/sensor/camera")
    camera.find('horizontal_fov').text = '1.05'
    camera.find('clip/far').text = '10.0'
    for plane in world.findall("model[@name='ground']/link/*/geometry/plane/size"):
        plane.text = '8 8'
    camera.find('image/width').text = '1280'
    camera.find('image/height').text = '960'
    world.find("model[@name='ground']/link/visual/material/diffuse").text = '0.32 0.35 0.38 1'
    grid = ET.SubElement(world, 'model', name='inspection_grid_20cm')
    ET.SubElement(grid, 'static').text = 'true'
    link = ET.SubElement(grid, 'link', name='grid')
    for axis in ('x', 'y'):
        for i in range(-20, 21):
            coordinate = i * 0.2
            major = i % 5 == 0
            width = 0.006 if major else 0.0025
            visual = ET.SubElement(link, 'visual', name=f'{axis}_{i + 20:02d}')
            ET.SubElement(visual, 'pose').text = (
                f'0 {coordinate:.3f} 0.001 0 0 0' if axis == 'x'
                else f'{coordinate:.3f} 0 0.001 0 0 0'
            )
            box = ET.SubElement(ET.SubElement(visual, 'geometry'), 'box')
            ET.SubElement(box, 'size').text = (
                f'8 {width} 0.001' if axis == 'x' else f'{width} 8 0.001'
            )
            color = '0.62 0.65 0.68 1' if major else '0.45 0.48 0.51 1'
            material = ET.SubElement(visual, 'material')
            ET.SubElement(material, 'ambient').text = color
            ET.SubElement(material, 'diffuse').text = color
            ET.SubElement(visual, 'cast_shadows').text = 'false'
    ET.indent(tree, space='  ')
    return '<?xml version="1.0"?>\n' + ET.tostring(tree.getroot(), encoding='unicode') + '\n'


if __name__ == '__main__':
    path = ROOT / 'src/mobile_manipulator/worlds/pick_and_place_mobile.sdf'
    path.write_text(build_world())
    print(path)
