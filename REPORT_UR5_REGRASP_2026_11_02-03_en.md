# Report 2026-11-02 and 2026-11-03: UR5 plug insertion + cable re-grasp

Task: a real UR5 (not the UR5e) with a Robotiq 2F-85 picks up a NEMA 5-15P plug at the end of a cable, threads it through three posts, inserts it into a wall socket, releases it, then comes back, grasps the cable, waves it through the posts and lays it on the floor. The plug must still be in the socket at the end. The plug and the cable are simulated two-way (the plug pulls the cable and the cable pulls the plug back), and the cable stiffness comes from Young's modulus.

Simulation stack: Newton/MuJoCo (robot, gripper, plug, posts, wall, rigid socket), Newton/VBD (cable), cuRobo (collision-free trajectories, separate process), Isaac Sim (video rendering).

Contents: 1. Summary, 2. Timeline, 3. Problems (problem, cause, fix, result), 4. Final results, 5. Open issues, 6. Process notes, 7. How to re-run.

---

## 1. Summary

| | 2 Nov | 3 Nov |
|---|---|---|
| Starting point | Plug can be inserted in earlier versions; no cable re-grasp | Full chain runs but the cable wave pulls the plug 3 mm at gap 2-3 |
| Main result | Full chain works: insert, release, re-grasp at arc 0.78 m, wave through post 1, gap 1-2, post 2, gap 2-3, lay the cable along the plug's line, return home | Wrist turning removed during the wave: plug displacement at gap 2-3 drops from 3.05 mm to 0.79 mm. Accepted version: V4full |
| Plug still in socket at the end | Yes (displacement 1.2 to 3.6 mm, varies per run) | Yes (displacement 0.47 to 0.48 mm, max 1.07 to 1.09 mm) |
| Cable dropped from the gripper | Fixed in V1 | No |
| Not achieved | Pin lateral error after release 0.43 to 0.68 mm (limit 0.2 mm) | Same; the 0.2 mm check was dropped from the test by the new plan |

Naming note: "V5" in the 2 Nov log means the brass spring-contact socket model, which was dropped on request. "V5" in the 3 Nov log means a different experiment (path height 0.20 m). They are unrelated.

---

## 2. Timeline

**2 Nov**
1. Base version `ur5_regrasp`: new layout (posts R = 5 cm, 30 cm high, robot to post 1 = 0.40 m, recessed socket), rigid socket without spring leaves, cable E = 5e6 Pa, grip force raised, new `regrasp_phase`, a rigid proxy body for the gripped cable section.
2. Afternoon update: straight Cartesian crossings at the gaps, sweep of 9 post/wall layouts, posts moved to 0.26 m spacing.
3. Final run of the base version: waves through both gaps, cable 1.6 m, cable laid along the plug's line.
4. V1: cable no longer slips out of the gripper at gap 2-3, arm no longer makes extra turns.

**3 Nov**
1. V3 (copy of V1) full run: plug stays inserted, but the old 0.2 mm assertion fails.
2. V4 / V4full: wrist yaw during the wave set to 0, the check reduced to "plug still in socket and cable not dropped". Run and accepted.
3. V5 experiment (path height 0.20 m): failed at planning.
4. Render V6: option to hide the blue trajectory line.

---

## 3. Problems: problem, cause, fix, result

### 2 Nov

| # | Problem | Cause | Fix | Result |
|---|---|---|---|---|
| 1 | Plug jams at about 9 mm depth and rotates 13 degrees inside the pads during insertion | Insertion needs a force of about 36 N or more, and the pad friction at the old grip effort (1.8 N m) was not enough to hold the plug (the original log quotes about 45 N as the friction capacity; the figures are copied from it, not re-derived) | Grip effort raised to 3.2 N m (about 55 N per pad). For gripping the cable a separate value is used (`--cable-grip-effort`, 0.9 then 2.2) | Plug inserts fully |
| 2 | cuRobo cannot plan to the grasp point chosen at arc 0.6 m | That point lies right next to post 1 | Grasp point must be on the floor and at least 13 cm from every post axis (later 10 cm) | Plans found |
| 3 | Hand hits the plug and pulls it 7 mm during the wave | The wave included "post 3", which is where the plug sits | Post 3 removed from the wave | No collision |
| 4 | Wave through gap 2-3 with the grasp point at arc 0.72 m: cable almost taut, plug pulled out from 0.2 mm to 14 mm, then NaN | Geometric: the full wave path is longer than the 1.2 m cable with one end fixed at the plug | Cable lengthened to 1.6 m (64 segments); grasp point is the farthest valid cable point (arc 0.78 m, segment 32); wave: post 1, gap 1-2, post 2, gap 2-3, then straight along the plug's y line and lay the cable aligned with the plug's ground projection | Wave through both gaps completes |
| 5 | Planner IK fails at random, results change slightly between runs | Random IK seeds | Number of tries raised to 200 | More stable planning |
| 6 | Plug travels over the top of the posts at the gaps (z up to 0.32 m, post top 0.30 m; x up to 0.557 m, gap centre 0.51 m) | cuRobo interpolates in joint space, so the TCP path arcs | Every gap crossing (gap_in to gap_out) is a straight Cartesian line by IK at constant yaw and height (`is_cross`); posts described to cuRobo as thin boxes inside the cylinder | Gap 1-2: x = 0.530 m (gap centre), z = 0.264 to 0.267 m. Gap 2-3: x = 0.797 m, z = 0.265 to 0.268 m. All below the post top |
| 7 | Clearance negative at 0.22 m post spacing (hand touches post at gap 1-2 by 2.0 mm, at gap 2-3 by 12.8 mm) | Gaps too narrow for the 89 mm wide gripper | Swept 9 layouts (table below). Chosen: post spacing 0.26 m, wall gap 0.08 m | Insert and transport through 3 posts succeed |
| 8 | Re-grasp approach fails at point (0.234, 0.105) | Point too close to the robot base, cuRobo finds no path | Grasp point radius filter r > 0.30 m (point (0.294, 0.095) planned alone, then used) | Re-grasp plans |
| 9 | Replaying the planned path backwards for the return pulled the plug out 11.9 mm, then NaN in one run | The cable drags the plug during the reversed motion | Reversed path removed; return goes via a clearance step and a joint-space line | No NaN in the final run |
| 10 | Cable slips out of the gripper before gap 2-3 (z from 0.26 m down to 0.02 m, pad gap closes to 12.5 mm) | Grip effort 0.9 N m and proxy friction 0.7 too low | Cable grip effort 0.9 to 2.2 N m, proxy friction 0.7 to 1.2, new `hold_check` that monitors proxy to TCP distance (warn above 3 cm, `--test` fails if the cable drops) | Cable no longer drops (max offset from the grip centre 21 mm) |
| 11 | Arm spins the base joint 286 degrees when approaching the cable; total base rotation 687 degrees in the re-grasp phase | cuRobo picks its own IK solution for `plan_pose`, possibly a full turn away | `ik_candidates` generates several IK solutions sorted by the largest joint travel, cuRobo plans only to the nearest ones; return to Home uses an "unwrapped" goal and a joint-space line first | Base rotation on approach 23 degrees, total base rotation 240 degrees, return path max 77 degrees on one joint |
| 12 | Simulation blows up (NaN): one cuRobo branch swings the gripper 3.04 m up to z = 1.14 m, jerking the cable and pulling the plug out | Branch chosen without regard to path length | Among cuRobo options pick the shortest gripper path | Chosen branch 1.0 m, z max 0.38 m |
| 13 | cuRobo "Start or End state in collision" at approach and return | Start of the return path is considered in collision | Near IK branch, plus a 15 cm sideways step away from the posts (`rg_clear`) before planning the return | Plans found |
| 14 | Syntax error: `or` applied to numpy arrays | Bug | Fixed | |

Layout sweep (minimum clearance gripper/arm to posts, mm):

| Post spacing (m) | Wall gap (m) | Gap 1-2 / gap 2-3 | Insert (arm to post 3) |
|---|---|---|---|
| 0.22 (old) | 0.08 | -2.0 / -12.8 (touches post) | +22 |
| **0.26** | **0.08** | **+18 / +2.1** | **+1.2** |
| 0.27 | 0.08 | +23 / +6.3 | -27.6 |
| 0.26 | 0.09 to 0.12 | +18 / +2.1 | -12 to -40 (arm hits post 3) |
| 0.26 | 0.14 | UR5 cannot reach | |

Note: moving the posts apart makes the cable path longer, so it does not add slack.

### 3 Nov

| # | Problem | Cause | Fix | Result |
|---|---|---|---|---|
| 15 | Plug displaced 3.05 mm at gap 2-3 and 3.88 mm after laying the cable | Wrist yaw of -90 / +60 degrees at the gaps (taken from the insertion path) twists and pulls the cable while the plug is already fixed in the socket and the hand holds the cable at arc 0.78 m of 1.6 m. Wrist joint 6 turned about 106 degrees on the way into gap 2-3 | `--wave-yaw-deg 0` (default in V4): no wrist turn during the re-grasp wave. cuRobo still found collision-free plans | Displacement at gap 2-3: 3.05 mm to 0.79 mm. After laying the cable: 3.88 mm to 0.48 mm. Cable to TCP offset: 17.9 mm to 6.5 mm. Final housing-to-face gap: 3.74 mm to 0.84 mm |
| 16 | `--test` failed on the 0.2 mm pin tolerance although the plug stayed in | The new plan (`plans/UR5_regrasp.md`) no longer prescribes a pin tolerance | `--tol` off by default; the test checks the plug stays in the socket (housing gap below 9 mm) and the cable is not dropped | Test passes |
| 17 | V5 (path height WEAVE_Z 0.27 to 0.20 m, to make the cable visibly go between the posts) fails: `cuRobo failed to plan 'weave leg 3.0 (-> post2)'` | Not verified. Hypothesis: at low height the gripper body and forearm get closer to post 2 (the 2 Nov log measured only 30 mm clearance at z = 0.17 m during insertion) | Not fixed. Not used | Open |
| 18 | Blue trajectory line drawn over the cable between 59 and 62 s (phase `rg_lower`), cable looks faded | The render draws the plug trajectory in blue | Render V6 with `--no-trajectory` | Cable data in that period is continuous (29 to 35 segments in gap 1-2, no NaN, no jumps); the video has all frames |
| 19 | Render V6 does not run | It imports `ur5_regrasp_geom_V4`, which was deleted during cleanup | Not fixed yet | Open |

---

## 4. Final results

**2 Nov, base version, final run (94.5 s simulated, 1292 s wall clock)**

| Item | Value |
|---|---|
| Insert, before release | pin error 0.112 / 0.086 mm (limit 0.2 mm, passes) |
| Insert, after release | 0.441 / 0.418 mm (does not pass) |
| Re-grasp | plug max displacement 1.21 mm, final 1.27 mm, housing gap 1.29 mm, still inserted |
| Pin error at the end of the test | 0.638 mm, tilt 4.2 degrees |
| Run-to-run variation (same configuration, non-deterministic GPU) | plug displacement 1.2 to 3.6 mm; one run 11.9 mm then NaN (with the reversed return path) |

**2 Nov, V1 (95.1 s simulated, resumed from the saved pre-insertion state, so the video shows insertion and re-grasp only)**

| | Base | V1 |
|---|---|---|
| Cable in the gripper through gap 2-3 | slips, sometimes NaN | does not drop, max offset 21 mm |
| Base joint rotation at approach | 286 degrees | 23 degrees |
| Total base rotation in re-grasp | 687 degrees | 240 degrees |
| Plug max displacement | 1.2 to 3.6 mm | 4.0 mm (still inserted, gap 3.76 mm) |
| Final pin error | 0.64 mm | 0.68 mm (still above 0.2 mm) |

**3 Nov, V3 (wrist yaw) vs V4full (no wrist yaw)**

| | V3 | V4full |
|---|---|---|
| Plug displacement entering gap 2-3 | 3.05 mm | 0.79 mm |
| Plug displacement after laying the cable | 3.88 mm | 0.48 mm (max 1.09 mm during re-grasp) |
| Max cable to TCP offset | 17.9 mm | 6.5 mm |
| Final housing-to-face gap | 3.74 mm | 0.84 mm |
| Final pin error | 0.66 mm | 0.37 mm (no longer a criterion) |
| Cable dropped | no | no |
| Final check (plug in socket) | passes (old 0.2 mm assertion fails) | passes |

V4full: 959 s wall clock, 72.2 s simulated. A repeat run (`_V4full2`) gave nearly the same numbers (displacement 0.474 mm, gap 0.86 mm, max deviation 1.07 mm).

Accepted video: `video_ur5_regrasp_V4full.mp4`.

---

## 5. Open issues

1. Pin lateral error after release is 0.37 to 0.68 mm, above the 0.2 mm of the earlier plan. Cause: soft penalty contacts (ke = 1e5) and a tilt of 2.5 to 4 degrees. Ideas: higher ke, chamfered socket mouth, V-grooved pads.
2. Cable section in the gripper is kinematic (a proxy body), so there is no cable force on the pads. Two-way coupling is full only at the plug.
3. One run per configuration; the GPU is not deterministic, so results vary by 0.01 to 0.1 mm (and up to a few mm in earlier versions).
4. Posts are R = 5 cm per `plans/UR5_regrasp.md`, while the earlier screen task used 2.5 cm.
5. Not checked by eye: collisions between gripper and posts when the hand does not rotate.
6. Next: choose a direction for the path height (z = 0.22 to 0.24 m, or lowering z only while crossing a gap), fix render V6 to import `ur5_regrasp_geom_V4full`, grasp the cable near its tail or use a 2 m cable for more slack. Lesson: to gain slack, grasp farther from the plug, not nearer.

---

## 6. Process notes (mistakes and how they were handled)

- A suggestion "grasp the cable near the plug to get more slack" was wrong and was reversed (farther gives more slack).
- `ur5_regrasp_V3.py` was edited in place against the rule "every change is a new version"; the edit was reverted (0.2 mm tolerance restored, wave yaw option removed) and the change now lives in V4.
- A background wait loop matched its own command line and never ended; replaced by waiting on the process id. No file was affected.
- `ur5_regrasp_V4full_traj.npz` (written 22:03) is newer than the accepted video (20:02), so a later run overwrote it. Comparing 1444 video frames with a re-render from the current trajectory gave mean difference 0.16/255 (max 0.67, no frame above 2), so it is very likely the same trajectory, but there is no proof. Use a new `REGRASP_TAG` for each run to avoid overwriting.
- During cleanup, `ur5_regrasp_V4full.py` and the V4full video were moved to the trash by mistake and restored.

---

## 7. How to re-run

```
cd data/scripts
python ur5_build_assets_V4.py                                  # once
REGRASP_TAG=_V4full3 python ur5_regrasp_V4full.py --test       # new tag so results are not overwritten
python ur5_regrasp_render_isaac_V4full.py --stride 3 \
    --traj ../results/ur5_regrasp_V4full3_traj.npz \
    --shapes ../results/ur5_regrasp_V4full3_shapes.pkl \
    --out ../results/video_ur5_regrasp_V4full3.mp4
```

Useful options: `--save-state` / `--resume` (run only insertion and re-grasp from a saved pre-insertion state), `--regrasp-near-start`, `--wave-yaw-deg`, `--cable-grip-effort`, `--regrasp-arc`.
