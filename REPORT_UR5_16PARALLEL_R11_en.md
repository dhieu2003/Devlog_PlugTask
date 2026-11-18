# UR5 + Robotiq 2F-85 regrasp task on 16 parallel worlds — round 11 (r11) report

Results: `data/results/ur5_16parallel_r11/` (per-scenario folders, `summary.json`); 4x4 video table: `data/results/video_ur5_16parallel_r11.mp4`
(columns: cable shape | post position | socket position | all three together; one row per trial).

## 1. Summary
**9 of 16 trials succeeded (56.25 %).** Success = the sequence completes, the plug stays in the socket (gap < 9 mm), the cable is not dropped and the cable simulation has no numerical blow-up (no NaN, cable speed < 5 m/s).

| Group | Success | Succeeded | Failed (reason) |
|---|---|---|---|
| Cable shape | 2/4 | shape2, shape3 | shape1 (cable slipped out of the gripper after gap 1-2), shape4 (slipped after post 2) |
| Post position | 2/4 | posts1, posts2 | posts3 (slipped while lowering), posts4 (no cable grasp point left: cuRobo planning fails for every candidate) |
| Socket position | 2/4 | socket1, socket4 | socket2 (slipped after post 1), socket3 (right-hand socket: plug inserted, but cuRobo cannot plan the cable re-grasp approach) |
| All three | 3/4 | all1, all2, all3 | all4 (slipped after post 1) |

Plug-to-socket gap of the successful trials: 0.67-1.04 mm. No trial ended with a numerical error.

## 2. Changes between trials
Common set-up: 3 posts (radius 5 cm, height 30 cm), wall 8 cm behind post 3, cable 1.6 m, plug picked at (0.20, 0.26). Parameters not listed for a group stay at the base value (posts (0.35,0), (0.60,0), (0.85,0); socket (y, z) = (0.20, 0.27); straight cable).

### 2.1 Cable shape (group shape; 4 random seeds 2025-2028)
Random smooth curve inside the camera box (x -0.05..0.80, y 0.17..0.80), curvature <= 14 rad/m, crossing segments lifted by 1.1 cm.

| Trial | Self-crossings | x range [m] | y range [m] | Free end (x, y) |
|---|---|---|---|---|
| shape1 | 6 | 0.05..0.34 | 0.26..0.74 | (0.08, 0.38) |
| shape2 | 13 | -0.03..0.20 | 0.24..0.75 | (-0.02, 0.37) |
| shape3 | 4 | -0.03..0.25 | 0.26..0.75 | (0.08, 0.69) |
| shape4 | 13 | -0.01..0.20 | 0.26..0.73 | (0.03, 0.43) |

### 2.2 Post position (group posts; straight cable, base socket) - cases from `instructions/guide_16parallel.md`
| Trial | Post 1 (x, y) | Post 2 (x, y) | Post 3 (x, y) | Note |
|---|---|---|---|---|
| posts1 | (0.35, 0) | (0.60, 0) | (0.85, 0) | as in guide |
| posts2 | (0.35, 0) | (0.60, **0.04**) | (0.85, 0) | guide: y2 = 0.15; moved to 0.04 |
| posts3 | (0.35, 0) | (0.60, -0.15) | (0.85, 0) | as in guide |
| posts4 | (0.35, 0.15) | (0.60, **0.04**) | (0.85, 0) | guide: y2 = 0.15; moved to 0.04 |

Why y2 was moved (permitted by the user for cases 2 and 4): with y2 = 0.15 cuRobo cannot plan the leg after gap 2-3, because the hand collision sphere above the post top (z = 0.36 m vs post height 0.30 m) gets only 13 mm clearance from post 2. Clearance vs y2: 49.7 mm (0), 36.7 (0.04), 31.0 (0.06), 25.8 (0.08), 13.3 (0.15). Planning passed for y2 <= 0.04 and failed for y2 >= 0.06; the 31-37 mm threshold is inferred from this sweep, not read from cuRobo's code.

### 2.3 Socket position (group socket; straight cable, base posts)
Socket centre on the wall (y, z); r10 only varied z within 0.279-0.284, r11 spans 0.15 m in z and includes one socket to the right of post 3.

| Trial | Socket y [m] | Socket z [m] | Side of post 3 |
|---|---|---|---|
| socket1 | 0.12 | 0.33 | left |
| socket2 | 0.28 | 0.24 | left |
| socket3 | **-0.15** | 0.21 | **right** |
| socket4 | 0.20 | 0.36 | left |

The weave height is now decoupled from the socket height (`weave_z` = 0.28 m, below the post tops), so the plug is lifted/lowered only on the last leg.

### 2.4 All three changed together (group all)
Cable from seeds 2029-2032 (random), posts from the guide (y2 = 0.04 in cases 2 and 4, as in 2.2 - this change was made without the user's explicit approval for this group), socket chosen from placements that cuRobo could plan.

| Trial | Post 2 (x, y) | Post 1 (x, y) | Socket (y, z) | Cable crossings | Free end |
|---|---|---|---|---|---|
| all1 | (0.60, 0) | (0.35, 0) | (0.16, 0.30) | 2 | (0.18, 0.41) |
| all2 | (0.60, 0.04) | (0.35, 0) | (0.26, 0.26) | 5 | (0.03, 0.44) |
| all3 | (0.60, -0.15) | (0.35, 0) | (0.14, 0.34) | 5 | (0.12, 0.38) |
| all4 | (0.60, 0.04) | (0.35, 0.15) | (0.22, 0.30) | 4 | (0.01, 0.52) |

Post 3 is always (0.85, 0). z range of the sockets in this group is only 0.08 m: sockets with z <= 0.25 could not be planned when post 2 is at y = 0.04 (tested 0.22, 0.24, 0.25).

## 3. Why the right-hand socket was hard, and the fix
Diagnostic runs (left vs right socket, with and without the cable, up to the end of the carrying phase):
- For a right socket the plug must go to post 1 at y = -0.2, far from the lift pose (y = +0.26). The inverse-kinematics routine picked the branch with the smallest joint-space norm, which is an "over the back" branch (shoulder 155 deg, lift joint -184 deg, wrist -251 deg).
- The next leg then rotated the shoulder 210 deg and the wrist 205 deg in 8 s (left socket: 2 s). The plug slid 2-4 cm down in the pads (plug-to-TCP offset 1.2 -> 3.6-5 cm) and tilted 14 deg (no cable) / 35 deg (with cable), which ended in `insertion IK failed`.
- Fix: `UR5P_NATURAL=1` rejects IK branches whose shoulder lift joint is outside (-180, 0) deg. The shoulder rotation of that leg drops to 19 deg, the wrist to 3 deg, and the plug tilt at the socket to 1.5 deg. In r11, socket3 inserted the plug (1.7 mm deviation after release).
- The share of the cable force in the tilt (14 -> 35 deg) comes from comparing runs with and without the cable on the same path, not from a separate measurement.

## 4. Why the trials failed
1. **Cable slipped out of the gripper (shape1, shape4, posts3, socket2, all4: 5 trials).** The parallel-grip pads hold the cable by friction; when the cable is blocked by a post or dragged while lowering, it slides. The simulation stops when the cable point nearest the gripper is more than 10 cm from the TCP. Without that stop the cable snags on the posts, stretches (1.57 -> 1.9-2.2 m) and the coupled Newton/VBD system diverges. A grip effort of 3.2 N m (56 N) was used; whether it is sufficient has not been proven.
2. **cuRobo cannot plan the re-grasp (posts4, socket3: 2 trials).** posts4: every cable candidate fails (straight cable offers few grasp points near the posts, and the return-to-Home plan fails); socket3: the approach to every one of the 3 candidates fails. The cause of the socket3 failure was not investigated (the low, right-hand socket is a hypothesis only).

## 5. Open issues
- Cable slip remains the main failure mode (5 of 7 failures).
- Re-grasp after insertion for the right-hand, low socket.
- Sockets with z <= 0.25 together with a displaced post 2.
- Each trial was run once; cuRobo and VBD are stochastic, so the spread is unknown.
- The 16 "worlds" are 16 independent processes, not one multi-world Newton model.

Files: `data/scripts/ur5_16parallel_scenarios_r11.py` (new), `ur5_16parallel_geom.py` (`weave_z`), `ur5_16parallel_curobo.py` (`UR5P_NATURAL`). Devlog: `data/devlog/DEVLOG_UR5_16PARALLEL_R11_2026_11_17.md`.
