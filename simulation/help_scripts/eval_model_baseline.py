#!/usr/bin/env python3
"""eval_model_baseline.py — coverage модели в РОДНОМ Drone2DEnv (v2 ревью Aleks).

Вопрос ревью: «какой coverage та же модель набирает в DroneEnv на сопоставимой
комнате?» Без этого числа live-coverage не интерпретируем: 0.11 за 340 шагов —
это «модель работает, но медленно» или «модель не перенеслась»?

Запуск (из simulation/):
    PYTHONPATH=/data/git/rl-lab ./.venv-policy/bin/python3 \
        help_scripts/eval_model_baseline.py [--episodes 5]

Печатает coverage на чекпоинтах шагов + долю действий по типам.
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

RL_LAB = os.environ.get("RL_LAB_ROOT", "/data/git/rl-lab")
sys.path.insert(0, RL_LAB)

from stable_baselines3 import PPO  # noqa: E402
from envs.drone_2d_env import Drone2DEnv  # noqa: E402

MODEL = f"{RL_LAB}/export/sweep02/model.zip"
MAP = f"{RL_LAB}/maps/rl_rooms/rl_room_empty_6x6.npy"
CHECKPOINTS = (100, 338, 500, 1000, 2000, 3000)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=5)
    ap.add_argument("--deterministic", action="store_true",
                    help="по умолчанию False — как в bridge (predict det=False)")
    args = ap.parse_args()

    model = PPO.load(MODEL, device="cpu")
    cov_at: dict[int, list[float]] = {c: [] for c in CHECKPOINTS}
    action_hist = np.zeros(8, dtype=int)

    for ep in range(args.episodes):
        env = Drone2DEnv(map_path=MAP, seed=1000 + ep, max_steps=max(CHECKPOINTS))
        obs, _ = env.reset(seed=1000 + ep)
        coverage = 0.0
        for step in range(1, max(CHECKPOINTS) + 1):
            action, _ = model.predict(obs, deterministic=args.deterministic)
            action_hist[int(action)] += 1
            obs, _, terminated, truncated, info = env.step(int(action))
            coverage = info["coverage"]
            if step in cov_at:
                cov_at[step].append(coverage)
            if terminated or truncated:
                # эпизод кончился раньше — coverage фиксируем на остальных чекпоинтах
                for c in CHECKPOINTS:
                    if c > step:
                        cov_at[c].append(coverage)
                break
        print(f"episode {ep}: финальный coverage {coverage:.3f} на шаге {step}",
              flush=True)

    print("\n=== BASELINE: model.zip в Drone2DEnv (rl_room_empty_6x6) ===")
    print(f"эпизодов: {args.episodes}, deterministic={args.deterministic}")
    for c in CHECKPOINTS:
        vals = cov_at[c]
        if vals:
            print(f"  step {c:>5}: coverage mean {np.mean(vals):.3f} "
                  f"min {np.min(vals):.3f} max {np.max(vals):.3f}")
    total = action_hist.sum()
    names = ["fwd", "back", "s_left", "s_right", "rot+", "rot-", "scan", "fwd_wall"]
    print("  действия:", ", ".join(
        f"{n}={100 * v / total:.0f}%" for n, v in zip(names, action_hist)))


if __name__ == "__main__":
    main()
