"""Pytest конфиг — устанавливает PYTHONPATH для bridge unit + integration тестов.

Tests run via `.venv-policy/bin/python3 -m pytest test/` (isolated venv, не --system).
sys.path порядок:
    1. test/ (this file's dir)
    2. simulation/src/policy_bridge/policy_bridge/ — for `from policy_bridge.* import ...`
    3. $RL_LAB_ROOT — для `from envs import Drone2DEnv` (через mock_env.py)
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
PKG_DIR = HERE.parent  # simulation/src/policy_bridge/
SIM_ROOT = PKG_DIR.parent.parent  # simulation/
RL_LAB_ROOT = Path(os.environ.get("RL_LAB_ROOT", "/data/git/rl-lab"))

# Bridge package — import as `policy_bridge.*`.
sys.path.insert(0, str(PKG_DIR))
# rl-lab — для DroneCombinedExtractor + MockDroneEnv + Drone2DEnv.
sys.path.insert(0, str(RL_LAB_ROOT))
# Mock env живёт в export/sweep02 (rl-lab); добавим прямо.
sys.path.insert(0, str(RL_LAB_ROOT / "export" / "sweep02"))
