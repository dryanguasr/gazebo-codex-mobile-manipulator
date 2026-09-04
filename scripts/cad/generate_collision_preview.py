#!/usr/bin/env python3
"""Create a static URDF preview whose collision geometries are cyan visuals."""

from __future__ import annotations

import argparse
from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET


def add_collision_visuals(root: ET.Element, collision_only: bool) -> int:
    count = 0
    for link in root.findall('link'):
        if collision_only:
            for visual in list(link.findall('visual')):
                link.remove(visual)
        for collision in list(link.findall('collision')):
            visual = deepcopy(collision)
            visual.tag = 'visual'
            visual.attrib['name'] = f'collision_preview_{count}'
            material = ET.SubElement(visual, 'material')
            material.attrib['name'] = 'CollisionCyan'
            color = ET.SubElement(material, 'color')
            color.attrib['rgba'] = '0.0 1.0 1.0 0.55'
            link.append(visual)
            count += 1
    return count


def make_static(root: ET.Element) -> None:
    for control in list(root.findall('ros2_control')):
        root.remove(control)
    for gazebo in list(root.findall('gazebo')):
        root.remove(gazebo)
    gazebo = ET.SubElement(root, 'gazebo')
    static = ET.SubElement(gazebo, 'static')
    static.text = 'true'


def indent(element: ET.Element, level: int = 0) -> None:
    padding = '\n' + '  ' * level
    child_padding = '\n' + '  ' * (level + 1)
    if len(element):
        if not element.text or not element.text.strip():
            element.text = child_padding
        for child in element:
            indent(child, level + 1)
            if not child.tail or not child.tail.strip():
                child.tail = child_padding
        element[-1].tail = padding
    elif level and (not element.tail or not element.tail.strip()):
        element.tail = padding


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('input_urdf', type=Path)
    parser.add_argument('output_urdf', type=Path)
    parser.add_argument(
        '--collision-only',
        action='store_true',
        help='hide normal visuals instead of rendering a translucent overlay',
    )
    args = parser.parse_args()

    root = ET.parse(args.input_urdf).getroot()
    count = add_collision_visuals(root, args.collision_only)
    if count == 0:
        raise RuntimeError('input URDF contains no collision geometry')
    make_static(root)
    root.attrib['name'] = 'mobile_manipulator_collision_preview'
    indent(root)
    args.output_urdf.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(
        args.output_urdf,
        encoding='utf-8',
        xml_declaration=True,
    )
    print(
        f'wrote {args.output_urdf} with {count} collision visuals '
        f"(mode={'collision-only' if args.collision_only else 'overlay'})"
    )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
