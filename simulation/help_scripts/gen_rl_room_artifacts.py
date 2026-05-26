#!/usr/bin/env python3
"""Genera SDF + free_mask.png + metadata.json для rl_room_<meta> миров.

Single source of truth — ROOMS dict ниже. Каждый мир описан как набор стен (AABB
boxes в Gazebo coords) + spawn zone + operational z.

Координатная конвенция (matches sprint plan v2 VisitedGridBuilder offset):
  Gazebo origin = центр комнаты, +x East, +y North.
  Cell index (iy, ix) ∈ [0, 63]² с iy=0 соответствует y_gazebo = -3.15.
  Cell center: x_g = (ix - 31.5)*0.1, y_g = (iy - 31.5)*0.1.

  free_mask[iy, ix] = 1 (255 в PNG) если cell center внутри any wall AABB.

PNG saved с iy=0 на верху image — то есть image top соответствует
y_gazebo = -3.15 (юг). Top-down Gazebo рендеры будут визуально перевёрнуты
относительно free_mask.png — это intentional, сохраняем consistency с
bridge VisitedGridBuilder.

Usage:
    cd $AEROSEARCH_ROOT/claudedrone-git/simulation
    python3 help_scripts/gen_rl_room_artifacts.py            # генерит все
    python3 help_scripts/gen_rl_room_artifacts.py --room rl_room_empty_6x6
    python3 help_scripts/gen_rl_room_artifacts.py --preview  # ASCII в stdout
"""
from __future__ import annotations

import argparse
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Tuple

import numpy as np
from PIL import Image


CELL_SIZE_M = 0.1
GRID_SIZE = 64
ROOM_SIZE_M = CELL_SIZE_M * GRID_SIZE  # 6.4
HALF = ROOM_SIZE_M / 2.0               # 3.2

WALL_HEIGHT_M = 4.0  # attempt #21 RCA 2026-05-20: drone hovers @ z=3 (TARGET_ALTITUDE_M
# attempt #19 z-drift buffer); walls @ 2m → drone OVER walls → sensors see void → WF
# spins forever. 4m gives 1m buffer above drone hover altitude.
WALL_Z_CENTER = WALL_HEIGHT_M / 2.0  # 2.0
FLOOR_Z_CENTER = 0.0
FLOOR_THICKNESS = 0.05

DRONE_Z_SPAWN_M = 0.2      # z для include pose (чуть выше пола, takeoff поднимет)
DRONE_Z_OPERATIONAL = 0.5  # z для bridge / metadata (operational, после takeoff)

# ArduPilot SITL default home (Canberra) — required для navsat/AHRS init.
APARDU_LAT = -35.363262
APARDU_LON = 149.165237
APARDU_ELEV = 584.0


@dataclass
class Wall:
    """AABB wall — описание geometry для SDF + free_mask rasterization."""
    x_center: float
    y_center: float
    x_size: float
    y_size: float
    name: str  # для SDF model name
    color: Tuple[float, float, float] = (0.7, 0.7, 0.7)  # rgb 0..1

    def contains(self, x: float, y: float) -> bool:
        """True если (x, y) внутри AABB (по cell center)."""
        return (
            abs(x - self.x_center) <= self.x_size / 2.0
            and abs(y - self.y_center) <= self.y_size / 2.0
        )


@dataclass
class Room:
    name: str
    description: str
    walls: List[Wall]
    spawn_x_range: Tuple[float, float]
    spawn_y_range: Tuple[float, float]
    expected_cov_3k: Tuple[float, float]
    difficulty: str


# Outer walls — общие для всех миров (4 thin boxes по периметру).
def outer_walls() -> List[Wall]:
    return [
        Wall(0.0, +HALF - 0.05, ROOM_SIZE_M, 0.1, "wall_north", (0.6, 0.8, 0.9)),
        Wall(0.0, -HALF + 0.05, ROOM_SIZE_M, 0.1, "wall_south", (0.6, 0.9, 0.6)),
        Wall(+HALF - 0.05, 0.0, 0.1, ROOM_SIZE_M, "wall_east", (0.95, 0.65, 0.65)),
        Wall(-HALF + 0.05, 0.0, 0.1, ROOM_SIZE_M, "wall_west", (0.95, 0.90, 0.55)),
    ]


ROOMS: List[Room] = [
    Room(
        name="rl_room_empty_6x6",
        description="6.4×6.4 m empty room — sanity baseline (open-space sweep).",
        walls=outer_walls(),
        spawn_x_range=(-2.5, 2.5),
        spawn_y_range=(-2.5, 2.5),
        expected_cov_3k=(0.92, 0.97),
        difficulty="easy",
    ),
    Room(
        name="rl_room_pillar_center",
        description="6.4×6.4 m с центральной колонной 1×1 m (obstacle avoidance).",
        walls=outer_walls() + [
            Wall(0.0, 0.0, 1.0, 1.0, "pillar_center", (0.5, 0.4, 0.3)),
        ],
        spawn_x_range=(-2.5, -1.0),
        spawn_y_range=(-2.5, -1.0),
        expected_cov_3k=(0.88, 0.95),
        difficulty="easy-medium",
    ),
    Room(
        name="rl_room_two_chambers",
        description="6.4×6.4 m, центральная стена 0.4 m thick с gap 1 m (dead-end recovery).",
        walls=outer_walls() + [
            # middle wall — south part (cells iy=1..26 = 26 rows, ix=30..33)
            # iy=1: y_g = -3.05; iy=26: y_g = -0.55. Center y = -1.80, height = 2.6
            Wall(0.0, -1.80, 0.4, 2.6, "wall_middle_south", (0.7, 0.7, 0.7)),
            # middle wall — north part (cells iy=37..62 = 26 rows)
            # iy=37: y_g = 0.55; iy=62: y_g = 3.05. Center y = +1.80, height = 2.6
            Wall(0.0, +1.80, 0.4, 2.6, "wall_middle_north", (0.7, 0.7, 0.7)),
        ],
        spawn_x_range=(-2.5, -1.0),
        spawn_y_range=(-2.0, 2.0),
        expected_cov_3k=(0.65, 0.85),
        difficulty="hard",
    ),
]


# ============================================================================
# Rasterization
# ============================================================================

def rasterize(room: Room) -> np.ndarray:
    """Return free_mask uint8 (64×64) — 255=wall, 0=free."""
    mask = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.uint8)
    for iy in range(GRID_SIZE):
        for ix in range(GRID_SIZE):
            x = (ix - (GRID_SIZE - 1) / 2.0) * CELL_SIZE_M
            y = (iy - (GRID_SIZE - 1) / 2.0) * CELL_SIZE_M
            for wall in room.walls:
                if wall.contains(x, y):
                    mask[iy, ix] = 255
                    break
    return mask


def ascii_preview(mask: np.ndarray) -> str:
    """ASCII top-down preview. iy=0 на верху (как PNG)."""
    rows = []
    for iy in range(GRID_SIZE):
        row = "".join("#" if mask[iy, ix] == 255 else "." for ix in range(GRID_SIZE))
        rows.append(row)
    return "\n".join(rows)


# ============================================================================
# SDF generation
# ============================================================================

SDF_TEMPLATE = """<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="{name}">

    <gui fullscreen="0">
      <plugin filename="MinimalScene" name="3D View">
        <ignition-gui>
          <title>3D View</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">docked</property>
        </ignition-gui>
        <engine>ogre2</engine>
        <scene>scene</scene>
        <ambient_light>0.4 0.4 0.4</ambient_light>
        <background_color>0.8 0.8 0.8</background_color>
        <camera_pose>0 0 9 0 1.5707 0</camera_pose>
      </plugin>
      <plugin filename="GzSceneManager" name="Scene Manager">
        <ignition-gui>
          <property key="state" type="string">floating</property>
          <property type="bool" key="visible">false</property>
        </ignition-gui>
      </plugin>
    </gui>

    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system"
            name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-navsat-system"
            name="gz::sim::systems::NavSat"/>

    <!-- ArduPilot navsat/AHRS init anchor (TASK-021 21a fix). Без этого
         EKF/AHRS hang и arducopter не emit'ит MAVLink heartbeat. -->
    <spherical_coordinates>
      <latitude_deg>{lat}</latitude_deg>
      <longitude_deg>{lon}</longitude_deg>
      <elevation>{elev}</elevation>
      <heading_deg>0</heading_deg>
      <surface_model>EARTH_WGS84</surface_model>
    </spherical_coordinates>

    <light type="directional" name="main_light">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1.0 1.0 1.0 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <direction>0.1 0.1 -1</direction>
    </light>

    <!-- Floor {room_size}×{room_size} m -->
    <model name="floor">
      <static>true</static>
      <pose>0 0 {floor_z} 0 0 0</pose>
      <link name="link">
        <collision name="c">
          <geometry><box><size>{room_size} {room_size} {floor_thick}</size></box></geometry>
        </collision>
        <visual name="v">
          <geometry><box><size>{room_size} {room_size} {floor_thick}</size></box></geometry>
          <material>
            <ambient>0.85 0.80 0.65 1</ambient>
            <diffuse>0.85 0.80 0.65 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>

{walls}

    <!-- Drone spawn — point in spawn_zone — z = {drone_z_spawn} m (clearance),
         takeoff_node поднимет на operational z = {drone_z_op} m. -->
    <include>
      <uri>model://iris_claudedrone</uri>
      <pose>{spawn_x} {spawn_y} {drone_z_spawn} 0 0 0</pose>
    </include>

  </world>
</sdf>
"""

WALL_TEMPLATE = """    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {z} 0 0 0</pose>
      <link name="link">
        <collision name="c">
          <geometry><box><size>{xs} {ys} {zs}</size></box></geometry>
        </collision>
        <visual name="v">
          <geometry><box><size>{xs} {ys} {zs}</size></box></geometry>
          <material>
            <ambient>{r} {g} {b} 1</ambient>
            <diffuse>{r} {g} {b} 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""


def generate_sdf(room: Room) -> str:
    walls_xml = "\n\n".join(
        WALL_TEMPLATE.format(
            name=w.name,
            x=w.x_center, y=w.y_center, z=WALL_Z_CENTER,
            xs=w.x_size, ys=w.y_size, zs=WALL_HEIGHT_M,
            r=w.color[0], g=w.color[1], b=w.color[2],
        )
        for w in room.walls
    )
    # Spawn point — центр диапазона (deterministic).
    spawn_x = (room.spawn_x_range[0] + room.spawn_x_range[1]) / 2.0
    spawn_y = (room.spawn_y_range[0] + room.spawn_y_range[1]) / 2.0
    return SDF_TEMPLATE.format(
        name=room.name,
        lat=APARDU_LAT, lon=APARDU_LON, elev=APARDU_ELEV,
        room_size=ROOM_SIZE_M, floor_z=FLOOR_Z_CENTER, floor_thick=FLOOR_THICKNESS,
        walls=walls_xml,
        spawn_x=spawn_x, spawn_y=spawn_y,
        drone_z_spawn=DRONE_Z_SPAWN_M, drone_z_op=DRONE_Z_OPERATIONAL,
    )


# ============================================================================
# Metadata
# ============================================================================

def generate_metadata(room: Room, mask: np.ndarray) -> dict:
    wall_count = int((mask == 255).sum())
    free_count = int(GRID_SIZE * GRID_SIZE - wall_count)
    # obstacles_count = walls минус 4 outer (учитываем только inner obstacles)
    obstacles_count = max(0, len(room.walls) - 4)
    return {
        "name": room.name,
        "description": room.description,
        "room_size_m": ROOM_SIZE_M,
        "cell_size_m": CELL_SIZE_M,
        "grid_size": GRID_SIZE,
        "obstacles_count": obstacles_count,
        "wall_cells_count": wall_count,
        "free_count": free_count,
        "spawn_zone": {
            "x_range": list(room.spawn_x_range),
            "y_range": list(room.spawn_y_range),
        },
        "drone_z_m": DRONE_Z_OPERATIONAL,
        "expected_cov_3k": list(room.expected_cov_3k),
        "difficulty": room.difficulty,
        "image_convention": {
            "iy_axis": "iy=0 at top of image == y_gazebo=-3.15 (south side)",
            "ix_axis": "ix=0 at left of image == x_gazebo=-3.15 (west side)",
            "matches": "sprint plan v2 VisitedGridBuilder offset (y_offset=y+room_size/2)",
        },
    }


# ============================================================================
# Main
# ============================================================================

def write_room(room: Room, out_dir: Path, *, preview: bool = False) -> None:
    room_dir = out_dir / room.name
    room_dir.mkdir(parents=True, exist_ok=True)

    mask = rasterize(room)
    Image.fromarray(mask, mode="L").save(room_dir / "free_mask.png")

    sdf = generate_sdf(room)
    (room_dir / "world.sdf").write_text(sdf, encoding="utf-8")

    meta = generate_metadata(room, mask)
    (room_dir / "metadata.json").write_text(
        json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8"
    )

    wall_cells = int((mask == 255).sum())
    free_cells = GRID_SIZE * GRID_SIZE - wall_cells
    print(f"[{room.name}] wall={wall_cells} free={free_cells} → {room_dir}")
    if preview:
        print(ascii_preview(mask))
        print()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--out",
        default=str(Path(__file__).resolve().parent.parent / "src/drone_sim/worlds/rl_rooms"),
        help="output directory (default: simulation/src/drone_sim/worlds/rl_rooms)",
    )
    parser.add_argument("--room", help="generate only this room by name")
    parser.add_argument("--preview", action="store_true", help="ASCII preview to stdout")
    args = parser.parse_args()

    out_dir = Path(args.out)
    rooms = [r for r in ROOMS if (args.room is None or r.name == args.room)]
    if not rooms:
        raise SystemExit(f"no rooms match --room={args.room}; available: {[r.name for r in ROOMS]}")

    for room in rooms:
        write_room(room, out_dir, preview=args.preview)


if __name__ == "__main__":
    main()
