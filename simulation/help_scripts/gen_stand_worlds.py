#!/usr/bin/env python3
"""gen_stand_worlds.py — генератор 4 тестовых миров «Стенд»-спринта (Aleks 2026-06-08).

Эмитит rl_rooms/<world>/world.sdf + симлинк worlds/<world>.sdf для:
    rl_room_L_corridor   — L-коридор, 2 отрезка 3×8 м @90° (corner-nav стресс)
    rl_room_multi_pillar — 6×6 м, 3 столба 0.5×0.5 м (мульти-препятствие)
    rl_room_large        — пустая 10×10 м (OOD: модель обучена на 6.4)
    rl_room_apartment    — 3 комнаты 5×5+4×4+4×4, doorways 1.4 м (multi-room)

СТАНДАРТ МИРА (Alignment Блок 1): wall_height ≥ 4.0 м; bbox центрирован в (0,0)
(visited-grid offset = room/2); doorways ≥ 1.4 м; размеры кратны 0.1 м;
spawn — геом.центр (single-room) или центр комнаты 1 (multi-room).

Заголовок (gui/physics/plugins/light/spherical_coordinates) — точная копия
rl_room_pillar_center (ArduPilot navsat/AHRS anchor обязателен).

free_mask + metadata после генерации: help_scripts/gen_world_free_mask.py --world <w>.

Usage:
    python3 help_scripts/gen_stand_worlds.py            # все 4
    python3 help_scripts/gen_stand_worlds.py L_corridor # один (без префикса rl_room_)
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

SIM_ROOT = Path(__file__).resolve().parent.parent
WORLDS_DIR = SIM_ROOT / "src" / "drone_sim" / "worlds"
RL_ROOMS = WORLDS_DIR / "rl_rooms"

WALL_H = 4.0          # высота стен (стандарт ≥4.0)
WALL_T = 0.1          # толщина стен
Z = WALL_H / 2.0      # центр стены по z
CEIL_Z = WALL_H       # потолок на верхушке стен

# Палитра стен (RGB) — для читаемости видео.
WALL_RGB = "0.7 0.78 0.88"
OBST_RGB = "0.5 0.4 0.3"

HEADER = """<?xml version="1.0" ?>
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
        <camera_pose>0 0 {cam_z} 0 1.5707 0</camera_pose>
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
      <latitude_deg>-35.363262</latitude_deg>
      <longitude_deg>149.165237</longitude_deg>
      <elevation>584.0</elevation>
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
"""

FOOTER = """
    <!-- Drone spawn — z=0.2 м (clearance); takeoff_node поднимет на z=0.5. -->
    <include>
      <uri>model://iris_claudedrone</uri>
      <pose>{sx} {sy} 0.2 0 0 0</pose>
    </include>

  </world>
</sdf>
"""


def box_model(name: str, px, py, sx, sy, sz, rgb, z=Z, alpha="1") -> str:
    """Статический box-model (стена/столб/пол/потолок)."""
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{px} {py} {z} 0 0 0</pose>
      <link name="link">
        <collision name="c">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        </collision>
        <visual name="v">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material>
            <ambient>{rgb} {alpha}</ambient>
            <diffuse>{rgb} {alpha}</diffuse>
          </material>
        </visual>
      </link>
    </model>"""


def floor_ceiling(bx: float, by: float) -> str:
    """Пол (z=0) + прозрачный потолок (z=WALL_H, alpha 0.3) на bbox bx×by."""
    floor = f"""
    <model name="floor">
      <static>true</static>
      <pose>0 0 0.0 0 0 0</pose>
      <link name="link">
        <collision name="c">
          <geometry><box><size>{bx} {by} 0.05</size></box></geometry>
        </collision>
        <visual name="v">
          <geometry><box><size>{bx} {by} 0.05</size></box></geometry>
          <material>
            <ambient>0.85 0.80 0.65 1</ambient>
            <diffuse>0.85 0.80 0.65 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>"""
    # Потолок: держит дрон от вертикального ухода; alpha 0.3 → дрон виден сверху.
    # Parity-нейтрален (obs = 2D scan на высоте дрона, потолок z=4 не виден).
    ceil = f"""
    <model name="ceiling">
      <static>true</static>
      <link name="ceiling_link">
        <collision name="ceiling_col">
          <pose>0 0 {CEIL_Z} 0 0 0</pose>
          <geometry><box><size>{bx} {by} 0.05</size></box></geometry>
        </collision>
        <visual name="ceiling_vis">
          <pose>0 0 {CEIL_Z} 0 0 0</pose>
          <geometry><box><size>{bx} {by} 0.05</size></box></geometry>
          <material>
            <ambient>0.8 0.8 0.8 0.3</ambient>
            <diffuse>0.8 0.8 0.8 0.3</diffuse>
          </material>
        </visual>
      </link>
    </model>"""
    return floor + ceil


# ─────────────────────────── миры ───────────────────────────
# Каждый возвращает (bbox_x, bbox_y, [box-models], spawn_xy).

def world_L_corridor():
    """L: 2 отрезка 3×8 @90°. bbox 8×8 центр (0,0). Вертикальный рукав
    x[-4,-1] y[-4,4]; горизонтальный x[-4,4] y[-4,-1]; общий угол x,y[-4,-1].
    Spawn в углу-локте (-2.5,-2.5) — самый центральныйflyable, обе ветки
    требуют прохода угла. geom.центр bbox (0,0) НЕ flyable (вырез)."""
    H = 4.0
    walls = [
        box_model("wall_bottom", 0.0, -(H - WALL_T / 2), 8.0, WALL_T, WALL_H, WALL_RGB),
        box_model("wall_left", -(H - WALL_T / 2), 0.0, WALL_T, 8.0, WALL_H, WALL_RGB),
        # верх вертикального рукава (cap), x[-4,-1] → center -2.5, len 3
        box_model("wall_vtop", -2.5, (H - WALL_T / 2), 3.0, WALL_T, WALL_H, WALL_RGB),
        # правый торец горизонтального рукава, y[-4,-1] → center -2.5, len 3
        box_model("wall_hright", (H - WALL_T / 2), -2.5, WALL_T, 3.0, WALL_H, WALL_RGB),
        # внутренняя вертикальная стена x=-1, y[-1,4] → center 1.5, len 5
        box_model("wall_vinner", -1.0, 1.5, WALL_T, 5.0, WALL_H, WALL_RGB),
        # внутренняя горизонтальная стена y=-1, x[-1,4] → center 1.5, len 5
        box_model("wall_hinner", 1.5, -1.0, 5.0, WALL_T, WALL_H, WALL_RGB),
    ]
    return 8.0, 8.0, walls, (-2.5, -2.5)


def world_multi_pillar():
    """6×6, 3 столба 0.5×0.5 в фикс. позициях (разнесены, не блокируют друг
    друга, центр свободен). Spawn геом.центр (0,0)."""
    h = 3.0 - WALL_T / 2  # 2.95
    walls = [
        box_model("wall_north", 0.0, h, 6.0, WALL_T, WALL_H, WALL_RGB),
        box_model("wall_south", 0.0, -h, 6.0, WALL_T, WALL_H, WALL_RGB),
        box_model("wall_east", h, 0.0, WALL_T, 6.0, WALL_H, WALL_RGB),
        box_model("wall_west", -h, 0.0, WALL_T, 6.0, WALL_H, WALL_RGB),
    ]
    pillars = [
        box_model("pillar_1", 1.5, 1.5, 0.5, 0.5, WALL_H, OBST_RGB),
        box_model("pillar_2", -1.7, 0.5, 0.5, 0.5, WALL_H, OBST_RGB),
        box_model("pillar_3", 0.5, -1.6, 0.5, 0.5, WALL_H, OBST_RGB),
    ]
    return 6.0, 6.0, walls + pillars, (0.0, 0.0)


def world_large():
    """Пустая 10×10 (OOD: модель обучена на 6.4). Spawn центр (0,0)."""
    h = 5.0 - WALL_T / 2  # 4.95
    walls = [
        box_model("wall_north", 0.0, h, 10.0, WALL_T, WALL_H, WALL_RGB),
        box_model("wall_south", 0.0, -h, 10.0, WALL_T, WALL_H, WALL_RGB),
        box_model("wall_east", h, 0.0, WALL_T, 10.0, WALL_H, WALL_RGB),
        box_model("wall_west", -h, 0.0, WALL_T, 10.0, WALL_H, WALL_RGB),
    ]
    return 10.0, 10.0, walls, (0.0, 0.0)


def world_apartment():
    """3 комнаты 5×5+4×4+4×4, doorways 1.4. bbox 9×9 центр (0,0).
    A(5×5) x[-4.5,0.5]y[-4.5,0.5] центр; B(4×4) x[0.5,4.5]y[-4,0] восток;
    C(4×4) x[-4,0]y[0.5,4.5] север. Doorway A|B y[-2.7,-1.3], A|C x[-2.7,-1.3].
    Spawn — комната 1 (A) центр (-2,-2) (multi-room конвенция, как indoor_room)."""
    W = []
    # Room A
    W.append(box_model("a_south", -2.0, -4.5, 5.0, WALL_T, WALL_H, WALL_RGB))
    W.append(box_model("a_west", -4.5, -2.0, WALL_T, 5.0, WALL_H, WALL_RGB))
    # A east x=0.5 с проёмом к B y[-2.7,-1.3]: сегменты y[-4.5,-2.7] и y[-1.3,0.5]
    W.append(box_model("a_east_lo", 0.5, -3.6, WALL_T, 1.8, WALL_H, WALL_RGB))
    W.append(box_model("a_east_hi", 0.5, -0.4, WALL_T, 1.8, WALL_H, WALL_RGB))
    # A north y=0.5 с проёмом к C x[-2.7,-1.3]: сегменты x[-4.5,-2.7] и x[-1.3,0.5]
    W.append(box_model("a_north_w", -3.6, 0.5, 1.8, WALL_T, WALL_H, WALL_RGB))
    W.append(box_model("a_north_e", -0.4, 0.5, 1.8, WALL_T, WALL_H, WALL_RGB))
    # Room B (восток)
    W.append(box_model("b_east", 4.5, -2.0, WALL_T, 4.0, WALL_H, WALL_RGB))
    W.append(box_model("b_south", 2.5, -4.0, 4.0, WALL_T, WALL_H, WALL_RGB))
    W.append(box_model("b_north", 2.5, 0.0, 4.0, WALL_T, WALL_H, WALL_RGB))
    # Room C (север)
    W.append(box_model("c_north", -2.0, 4.5, 4.0, WALL_T, WALL_H, WALL_RGB))
    W.append(box_model("c_west", -4.0, 2.5, WALL_T, 4.0, WALL_H, WALL_RGB))
    W.append(box_model("c_east", 0.0, 2.5, WALL_T, 4.0, WALL_H, WALL_RGB))
    return 9.0, 9.0, W, (-2.0, -2.0)


WORLDS = {
    "rl_room_L_corridor": world_L_corridor,
    "rl_room_multi_pillar": world_multi_pillar,
    "rl_room_large": world_large,
    "rl_room_apartment": world_apartment,
}


def emit(world_name: str) -> None:
    bx, by, models, (sx, sy) = WORLDS[world_name]()
    cam_z = max(bx, by) * 1.4 + 1.0   # камера сверху, чтоб bbox влезал
    sdf = HEADER.format(name=world_name, cam_z=round(cam_z, 2))
    sdf += floor_ceiling(bx, by)
    sdf += "".join(models)
    sdf += FOOTER.format(sx=sx, sy=sy)

    out_dir = RL_ROOMS / world_name
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "world.sdf").write_text(sdf)

    link = WORLDS_DIR / f"{world_name}.sdf"
    rel = os.path.join("rl_rooms", world_name, "world.sdf")
    if link.is_symlink() or link.exists():
        link.unlink()
    link.symlink_to(rel)
    print(f"emit: {out_dir}/world.sdf  (bbox {bx}×{by}, spawn {sx},{sy})  "
          f"+ symlink {link.name} -> {rel}")


def main() -> None:
    argv = sys.argv[1:]
    if argv:
        names = [a if a.startswith("rl_room_") else f"rl_room_{a}" for a in argv]
    else:
        names = list(WORLDS)
    for n in names:
        if n not in WORLDS:
            raise SystemExit(f"неизвестный мир: {n} (есть: {sorted(WORLDS)})")
        emit(n)


if __name__ == "__main__":
    main()
