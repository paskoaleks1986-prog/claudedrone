"""PhaseController — switch Wall Follower → RL Policy.

TASK-062 Задача 3 (2026-05-20 PATH B HYBRID). Управляет двумя фазами:

    PHASE_WALL_FOLLOW:
        - WallFollower computes target pos/yaw (deterministic right-hand rule)
        - WallMapBuilder accumulates wall mask
        - VisitedGridBuilder updates с drone pos (тоже, иначе RL начнёт с пустой visited)
        - Switch к PHASE_RL_EXPLORE по perimeter_complete

    PHASE_RL_EXPLORE:
        - Build obs (distances + servo_angle + visited_grid + optionally wall_mask)
        - policy.predict → action
        - action_executor.execute → target pos/yaw

Bridge'у не нужно знать фазу — он просто emit target от phase_controller.step().

Optional wall_mask в obs: rl-lab @TASK-063 sends HANDOFF подтверждение. Если
обновлённый obs_spec.md включает wall_mask channel, PhaseController._build_obs
добавит wall_mask в Dict. Если silent — без wall_mask (default).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Optional

import numpy as np

from policy_bridge.wall_follower import WallFollower, WallFollowerCmd, Pose2D as WFPose
from policy_bridge.wall_map_builder import WallMapBuilder, Pose2D as WMPose


class Phase(Enum):
    WALL_FOLLOW = "wall_follow"
    RL_EXPLORE = "rl_explore"


@dataclass
class PhaseControllerCmd:
    """Result of step(): either target pose (wall follow / RL action target) OR
    raw RL action for action_executor processing.

    For PHASE_WALL_FOLLOW: target_pos + target_yaw set directly.
    For PHASE_RL_EXPLORE: action_int set, target_pos/yaw left None — bridge
    routes via action_executor.execute(action, ...).

    type: passthrough from WallFollowerCmd.type (forward/turn_left/turn_right/
    adjust_left/adjust_right/corner_resume/find_wall_forward/approach_wall);
    бridge uses to decide rotation-only setpoint emission (Risk #2 fix).
    """
    phase: str
    target_x: float | None = None
    target_y: float | None = None
    target_yaw: float | None = None
    action: int | None = None      # for RL phase
    debug_tag: str = ""
    type: str = ""


class PhaseController:
    """Coordinator. Bridge calls .step(...) каждый tick.

    Constructor injection: wall_follower + wall_map_builder + visited_grid (writes
    visited cells during wall_follow phase) + policy_predict_fn для RL phase.
    """

    def __init__(
        self,
        wall_follower: WallFollower,
        wall_map_builder: WallMapBuilder,
        visited_update_fn: Callable[[float, float], None],  # signature как VisitedGridBuilder.update
        policy_predict_fn: Optional[Callable] = None,
        node_logger=None,
    ) -> None:
        self.wf = wall_follower
        self.wmb = wall_map_builder
        self.visited_update = visited_update_fn
        self.policy_predict = policy_predict_fn
        self.logger = node_logger
        self.phase = Phase.WALL_FOLLOW
        self._switched_at_step: int | None = None

    @property
    def current_phase(self) -> Phase:
        return self.phase

    def step(
        self,
        distances: list[float],
        pose_x: float,
        pose_y: float,
        pose_yaw: float,
        step_count: int,
    ) -> PhaseControllerCmd:
        """One step of the controller.

        distances: 6× VL53L0X raw meters (ch0..ch5)
        pose_x, pose_y, pose_yaw: drone current pose (world ENU)
        step_count: bridge step counter (для logging)
        """
        if self.phase == Phase.WALL_FOLLOW:
            # Update visited_grid from drone pos (важно — RL phase будет видеть прогресс)
            self.visited_update(pose_x, pose_y)
            # Update wall map
            wm_pose = WMPose(pose_x, pose_y, pose_yaw)
            cells_added = self.wmb.update(wm_pose, distances)

            # Check perimeter complete
            wf_pose = WFPose(pose_x, pose_y, pose_yaw)
            if self.wf.check_perimeter_complete(wf_pose):
                self._switch_to_rl(step_count)
                # Continue with RL phase в this same tick? Skip — wait next tick для RL init.
                # Return a hold command (target = current pos/yaw)
                return PhaseControllerCmd(
                    phase=self.phase.value,
                    target_x=pose_x,
                    target_y=pose_y,
                    target_yaw=pose_yaw,
                    debug_tag="phase_switch_hold",
                )

            # Wall following step
            cmd = self.wf.step(distances, wf_pose)
            return PhaseControllerCmd(
                phase=self.phase.value,
                target_x=cmd.target_x,
                target_y=cmd.target_y,
                target_yaw=cmd.target_yaw,
                debug_tag=f"wf_{cmd.state}_{cmd.type}",
                type=cmd.type,
            )

        # PHASE_RL_EXPLORE
        if self.policy_predict is None:
            # No policy provided — hold position
            return PhaseControllerCmd(
                phase=self.phase.value,
                target_x=pose_x,
                target_y=pose_y,
                target_yaw=pose_yaw,
                debug_tag="rl_no_policy_hold",
            )

        # Bridge will handle obs build + policy.predict + action_executor.execute
        # via its own pipeline. PhaseController just signals phase.
        return PhaseControllerCmd(
            phase=self.phase.value,
            action=None,  # bridge handles policy internally
            debug_tag="rl_delegate_to_bridge",
        )

    def _switch_to_rl(self, step_count: int) -> None:
        if self.phase == Phase.RL_EXPLORE:
            return  # already there
        wall_cells = self.wmb.wall_cells_total
        self.phase = Phase.RL_EXPLORE
        self._switched_at_step = step_count
        msg = (
            f"[PHASE] Wall following complete → RL explore "
            f"(step {step_count}, wall_cells={wall_cells}, "
            f"laps={self.wf.laps_completed})"
        )
        if self.logger is not None:
            self.logger.info(msg)
        else:
            print(msg)
