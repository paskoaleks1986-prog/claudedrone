# 05 — indoor.parm Cyrillic comments → ArduPilot crash-restart loop

**Date:** 2026-05-08
**Continues:** 04 (SITL no-heartbeat). Found one of the layered bugs.
**Owner:** simulation. Half of TASK-021 21a.

## TL;DR

`indoor.parm` had Cyrillic comments — many of them >40 ASCII-equivalent
chars. ArduPilot's defaults parser uses a 100-byte `fgets` buffer plus a
`line[0] == '#'` comment check. When a comment line is longer than 99 bytes,
`fgets` returns it in chunks: the first chunk has `#` and is dropped, but
the subsequent chunk starts with text → parsed as a bogus param name
(e.g. `а`, `в`). One unknown param keeps `done_all_default_params == false`,
which causes `reload_defaults_file()` to fire on every subsequent param load
during library init → infinite reload loop → arducopter never reaches the
GCS scheduler → no MAVLink heartbeat → MAVProxy stuck on
`Waiting for heartbeat`. This explained the 23-reload-in-25-seconds symptom
from dev-log/04.

Fixed in `indoor.parm` by rewriting all comments in ASCII and capping
lines at 78 chars.

**Note: this fix alone does not bring heartbeat up.** The reload loop is
gone, but arducopter still stalls right after `validate_structures` in both
our world and the canonical `ardupilot_gazebo/worlds/iris_runway.sdf`. The
remaining stall is upstream-version territory, not our params/world. See
"Remaining: post-validate stall" below.

## How I found it

Reproduced dev-log/04 symptom on a clean tree (no orphans, fresh launch).
Captured the ArduCopter sim_vehicle window output across `-S -3000` to see
both the startup phase and the loop body. Single-pass enabled
`ENABLE_DEBUG = 1` in `libraries/AP_Param/AP_Param.cpp` (line 62, 0→1),
rebuilt SITL, ran again. The debug build prints `Ignored unknown param X
in defaults file Y` — which immediately revealed:

```
Ignored unknown param а in defaults file .../indoor.parm
```

(`а` = single Cyrillic 'a' letter.) Reverted the patch + rebuilt clean
once the cause was nailed.

The parser (libraries/AP_Param/AP_Param.cpp):

- `parse_param_line()` line 2272: `if (line[0] == '#') return false;`
  — comments only detected at the very start of a line.
- `count_defaults_in_file()` line 2325: `char line[100];` buffer.
- `fgets(line, sizeof(line)-1, file)` reads ≤99 bytes per call. A comment
  line longer than 99 bytes is split into multiple `fgets` calls; only the
  first call has `#` at position 0.
- The reload-loop trigger: in `load_defaults_file()` line 2378 (the
  unknown-param branch sets `done_all = false`), and at the end of the
  function (line 2398) `done_all_default_params = done_all` is committed.
  Then `load_object_from_eeprom()` line 1722-1727 fires
  `reload_defaults_file(false)` whenever a new var subtree registers AND
  `done_all_default_params` is false. With one unknown param this is
  permanent, and the loop keeps firing every time a library lazily
  registers params during init.

UTF-8 makes Cyrillic doubly costly per char (2 bytes), so even 50-char
Russian comments overflow the 99-byte buffer easily. Pure ASCII comments
under ~80 chars fit safely.

## Companion finding: ARMING_CHECK was renamed to ARMING_SKIPCHK in 4.7

While debugging I tried a temp `diagnostic-minimal.parm` with
`ARMING_CHECK 0` (the historical name). ENABLE_DEBUG output flagged it as
unknown. Source check:

- `libraries/AP_Arming/AP_Arming.cpp:128` — `// 2 was the CHECK paramter`
- `libraries/AP_Arming/AP_Arming.cpp:201` — `AP_GROUPINFO("SKIPCHK", 13, ...)`
- `libraries/AP_Arming/AP_Arming.cpp` `init()` — has a
  `PARAM_CONVERSION - 4.7 CHECK -> SKIPCHK` migration.

Our `indoor.parm` already used the new `ARMING_SKIPCHK 1`, so this was a
non-issue for the production file — only my diagnostic file would have
created a second unknown-param. Documented for future param work.

## Remaining: post-validate stall (separate from this fix)

After the reload-loop fix, arducopter still stops emitting output after
`validate_structures: Validating structures` and never sends heartbeat.
Verified this also reproduces with the canonical
`ardupilot_gazebo/worlds/iris_runway.sdf` and **no `--add-param-file`**
(only the built-in `copter.parm` + `gazebo-iris.parm`). UDP recv queue on
the arducopter end shows ~1280 bytes pending — gz sim is sending JSON,
arducopter is not consuming.

State on D2 at the time:
- ArduPilot master: `416146b48f` (4.6.0-beta1 + 6466 commits ahead)
- ardupilot_gazebo: `082a0fe` (Iris: improve collisions)
- Gazebo Sim 8.11.0 (Harmonic), Linux 6.17

Hypotheses to explore in a follow-up session (best paired with
researchbest, since this is upstream library territory):

1. ArduPilot master regression — try checking out a stable tag
   (`Copter-4.6.0` or `Copter-4.5.x`) and rebuilding.
2. ardupilot_gazebo plugin behind ArduPilot — `git pull` the plugin and
   rebuild.
3. Mid-init wait state we don't see (no obvious panic in output) — try
   gdb attach + backtrace once arducopter parks in `poll_schedule_timeout`.
4. Missing `<spherical_coordinates>` in `worlds/indoor_room.sdf` may
   matter once heartbeat is live (navsat plugin needs an anchor) — not
   the cause of this stall (canonical world has it and stalls anyway),
   but should be added before we expect EKF to converge with GPS on.

This is out of scope for "bring-up bug in researchbest stack" — it's
an upstream version-pinning question. Suggest a research-tagged ticket
(simulation + researchbest) once Aleks decides on direction.

## Files touched

- `simulation/config/ardupilot/indoor.parm` — comments rewritten in
  ASCII, lines ≤78 chars (`wc -L = 78`).
- `simulation/config/ardupilot/diagnostic-minimal.parm` — kept as a
  short ASCII-only repro file for future debugging (only sets
  `ARMING_SKIPCHK 1` and `BATT_MONITOR 0`; the embedded note in the
  file warns about line-length).

## Acceptance for this slice (NOT full TASK-021 21a)

- [x] Root cause for crash-restart loop identified and fixed (Cyrillic
      comments in `indoor.parm`)
- [x] `done_all_default_params` mechanism documented for future
      reference
- [x] `ARMING_CHECK -> ARMING_SKIPCHK` 4.7 rename documented
- [x] Verification: with cleaned `indoor.parm`, `Loaded defaults from`
      prints exactly twice (normal startup) instead of N-per-second loop;
      zero `Ignored unknown` in ArduCopter window
- [ ] Heartbeat reliable — STILL NO. Stall is post-validate, upstream
      issue, not in our params/world

## Что для researchbest

- Cleanup helper + cleaned `indoor.parm` reduce the symptom space —
  reload-loop is no longer in the way.
- Heartbeat is still blocked; do not start 21b/21c integration work
  until the post-validate stall is resolved (separate session).
- When you do start, the recommendation in dev-log/04 about always
  running `d2_sitl_cleanup.sh --dry-run` after `daemon.sh stop` still
  stands — orphan UDP-9002 races are a real second class of failure.
