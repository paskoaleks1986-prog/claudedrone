# Current Sprint — Week 1 (May 5–11, 2026)

> **For agents:** this is the current week's task list. Check off items as completed. Update on Sunday for next week.

**Sprint goal:** Public foundation (GitHub mirror) + stop-scan ROS2 node skeleton.

**Hours target:** ~20-25 hours across the week.

---

## Tasks

### Public foundation (Mon-Tue)

- [ ] Create GitHub repository `claudedrone`
- [ ] Generate Personal Access Token (fine-grained, repo scope)
- [ ] Set up GitLab Push Mirror to GitHub
- [ ] Verify auto-sync with test commit
- [ ] Add About description, topics, social preview image on GitHub
- [ ] Public README v1 published (English, with mermaid architecture diagram)
- [ ] Public ROADMAP.md published

### Stop-scan ROS2 skeleton (Wed-Thu)

- [ ] Create `stop_scan_node` Python file in `simulation/src/drone_sim/drone_sim/`
- [ ] Implement FSM with states: HOVER, STOP, SCAN, MOVE
- [ ] State transitions on timer (configurable durations)
- [ ] Subscribe to `/mavros/state`
- [ ] Publisher for `/mavros/setpoint_position/local`
- [ ] Add to `CMakeLists.txt` install section
- [ ] Build + run test: node logs state transitions correctly

### SG90 control in simulation (Fri)

- [ ] Verify SG90 model in `iris_claudedrone/model.sdf`
- [ ] ROS2 node or topic interface to set servo angle
- [ ] Test: command angle 0° → 90° → 180° → 0°, observe servo movement in Gazebo
- [ ] Document the topic interface in node docstring

### Public posts (during the week)

- [ ] Twitter/X: GitHub launch announcement with simulation screenshot
- [ ] LinkedIn: same content adapted for business audience (1 post)

---

## Deliverables for this week

1. Public GitHub repo with mirror from GitLab
2. README.md and ROADMAP.md visible on GitHub
3. `stop_scan_node` skeleton compiles and runs (state transitions only — no actual scanning yet)
4. SG90 controllable from ROS2
5. 2 public posts (Twitter + LinkedIn)

## Blockers

None currently.

## Notes / observations

- _(Add notes during the week about anything unexpected, useful learnings, ideas for next week)_

---

## Sunday review checklist (May 11)

- [ ] All deliverables met? If not, why?
- [ ] Public posts published?
- [ ] GitHub commits visible to public?
- [ ] What was harder than expected?
- [ ] What was easier than expected?
- [ ] Update [`current-status.md`](current-status.md) with this week's progress
- [ ] Create `current-sprint.md` for Week 2 (move this file to `docs/planning/sprints-archive/2026-W18.md`)
