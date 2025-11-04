#!/usr/bin/env python3
import os
import random
import tempfile
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory

# Parameters
NUM_BOXES = 5
NUM_WALLS = 3
LIMIT = 5.5          # map radius
BOX_SIZE = 0.7
WALL_THICKNESS = 0.20
MIN_WALL_LEN = 1.5
MAX_WALL_LEN = 3.5

def _random_boxes_sdf(num, limit, size):
    parts = []
    for i in range(num):
        x = random.uniform(-limit, limit)
        y = random.uniform(-limit, limit)
        parts.append(f"""
        <model name="rand_box_{i}">
          <static>true</static>
          <pose>{x:.3f} {y:.3f} 0 0 0 0</pose>
          <link name="link">
            <collision name="collision">
              <geometry><box><size>{size} {size} {size}</size></box></geometry>
            </collision>
            <visual name="visual">
              <geometry><box><size>{size} {size} {size}</size></box></geometry>
              <material><ambient>1 0 0 1</ambient></material>
            </visual>
          </link>
        </model>
        """)
    return "\n".join(parts)

def _random_walls_sdf(num, limit):
    parts = []
    for i in range(num):
        x = random.uniform(-limit, limit)
        y = random.uniform(-limit, limit)
        length = random.uniform(MIN_WALL_LEN, MAX_WALL_LEN)
        yaw = random.choice([0, 1.57])  # horizontal or vertical

        parts.append(f"""
        <model name="rand_wall_{i}">
          <static>true</static>
          <pose>{x:.3f} {y:.3f} 0 0 0 {yaw}</pose>
          <link name="link">
            <collision name="collision">
              <geometry><box><size>{length} {WALL_THICKNESS} 1</size></box></geometry>
            </collision>
            <visual name="visual">
              <geometry><box><size>{length} {WALL_THICKNESS} 1</size></box></geometry>
              <material><ambient>0 0 1 1</ambient></material>
            </visual>
          </link>
        </model>
        """)
    return "\n".join(parts)

def main():
    pkg_gazebo = get_package_share_directory('simulation_gazebo')
    world_file = os.path.join(pkg_gazebo, 'worlds', 'diff_drive.sdf')

    with open(world_file, "r") as f:
        world_xml = f.read()

    insert_at = world_xml.rfind("</world>")
    if insert_at == -1:
        raise RuntimeError("Cannot find </world> tag in diff_drive.sdf")

    world_mod = (
        world_xml[:insert_at]
        + _random_boxes_sdf(NUM_BOXES, LIMIT, BOX_SIZE)
        + _random_walls_sdf(NUM_WALLS, LIMIT)
        + world_xml[insert_at:]
    )

    try:
        ET.fromstring(world_mod)
    except ET.ParseError as e:
        print("❌ INVALID XML:", e)
        return

    tmp = tempfile.NamedTemporaryFile(prefix="rand_world_", suffix=".sdf", delete=False)
    tmp.write(world_mod.encode("utf-8"))
    tmp.close()

    print(tmp.name)  # This gets passed to Gazebo

if __name__ == "__main__":
    main()