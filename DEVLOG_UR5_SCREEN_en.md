# DEVLOG UR5_SCREEN (V0 → V3) – UR5e + Robotiq 2F-85 inserting a NEMA 5-15P plug through 3 posts

**Task** (`plans/UR5_Screen.md`): a UR5 with a Robotiq 2F-85 grasps the plug at the end of a VBD cable lying on the floor, lifts it, carries it past a row of 3 obstacle posts, plugs it into a wall socket (perpendicular to the floor), releases, and returns Home. The Isaac Sim video shows an OXYZ gizmo (bottom-left) and a gravity vector P at the middle of the arm.

**Stack**: Newton + `SolverMuJoCo` (robot, plug, contacts; 16 substeps per 60 Hz frame), `SolverVBD` for the cable (one-way: the cable does not act back on the plug), cuRobo 0.8 for collision-free planning, headless Isaac Sim for video.

## Results at a glance

| Version | Main setup | Plug inserted & self-holding | Pin error / tilt | Cable |
|---|---|---|---|---|
| V0 | 3 posts r=2 cm, H=40 cm side by side, passes through gap between posts 2–3 | Yes, housing face 0.49 mm from socket | 0.19 / 0.11 mm, ~1.0° | Follows plug, no wrapping (video only) |
| V2 | 3 posts D=5 cm, H=30 cm in line robot–wall, staggered weave | Yes, 0.45 mm | 0.19 / 0.12 mm, ~1° | Continuous, but penetrates post 2 by up to 17.8 mm |
| V3 | Same as V2, cable contact stiffness changed | Yes, 0.45 mm | 0.19 / 0.12 mm, ~1° | No post penetration, max stretch ratio 1.005 |

The pin-position requirement of < 0.07 mm is **not met** in any version.

---

## V0 – `ur5_screen.py`: baseline pick-and-insert pipeline

### Design
| Component | Approach |
|---|---|
| Robot | UR5e (Menagerie; no plain UR5 available) + Robotiq 2F-85 |
| Plug | Free rigid body, 0.1 kg: 50×22×30 mm housing, two 18×6.35×1.524 mm flat blades (12.7 mm apart), round pin Ø4.76×21 mm offset 7.9 mm below (spec) |
| Socket | 6.5×1.8 mm slots, 17.5 mm deep; Ø4.9 mm round hole, 21.5 mm deep; clearances 0.075 / 0.07 mm; built entirely from boxes (no concave mesh) |
| Posts | 3 cylindrical posts r = 20 mm, 0.40 m tall, aligned along Y at x = 0.58 m; 0.14 m clear gap |
| Cable | VBD, 45 segments, 0.9 m long; first 4 segments rigidly attached to the plug |
| Planning | cuRobo `plan_pose` / `plan_cspace` (avoids floor, wall, posts, self-collision, plug bounding sphere); straight segments along the gripper axis use damped IK |
| In-hand perception | Measure plug pose relative to TCP, then recompute a straight insertion path |

Self-made assumptions (pending confirmation): cable 0.9 m (spec's "10–20 m" is too large for the scene); plug passes through the gap between posts 2–3; plug stands on its narrow edge, grasped with a 70° tilt; Pose A = (0.30, 0) m, socket at (0.78, 0.09, 0.28) m. Right-handed frame used (spec says Y points "from inside to outside", which conflicts with X right, Z up) – needs confirmation.

### Symptom → hypothesis → change → result

| # | Symptom | Hypothesis (cause) | Change | Result |
|---|---|---|---|---|
| 1 | cuRobo: `Numpy is not available` | torch 2.11 needs numpy 2, but the `cable` env uses numpy 1.26 (Isaac Sim/Newton) | Install numpy 2.2.6, scipy 1.14.1, cuda-core into `~/.cache/cable_task/np2`; run cuRobo in a child process with `PYTHONPATH` + `LD_LIBRARY_PATH` to `nvrtc` 13 | cuRobo runs; conda untouched |
| 2 | cuRobo has no UR5e config | Only UR10e is provided | Write own URDF from Menagerie + gripper collision spheres | FK matches Newton (home TCP = (0.492, 0.134, 0.3465)) |
| 3 | Plug jams at the socket face | Geometry bug: wall slab covered the socket window | Build wall from 4 boxes around the window + socket block with slots cut from a box mesh | Plug enters the socket |
| 4 | Plug slips and drops during transport | cuRobo path went through posts: cylinder posts ignored in the cuRobo scene; Home was also inside a post | Use bounding boxes for posts, move posts/wall to x = 0.58/0.78, add independent `clearance()` check, add a waypoint at the gap | Transport through the gap without dropping |
| 5 | Plug stops 46 mm short of the socket | `wrist_1_link` (z≈0.48 m) hits a post mid-push; straight segments aren't arm-collision-checked | Lower posts to 0.40 m, plan the "through gap" segment with cuRobo (arm checked), add `debug_contacts` to print contacts | Plug reaches the socket |
| 6 | Wall rendered in the wrong place in the video | USD applied scale before transform, multiplying coordinates | Apply transform first, then scale | Wall correct |
| 7 | Changing shape `ke` had no effect | MuJoCo contact stiffness comes from `solref/solimp`, not `ke` | Tried `solref`=0.004 and tight `solimp` | **Plug jams** (3.4° tilt, 21 mm short): in-hand perception is only ~0.5° accurate (≈0.2 mm at the pin tip), larger than the 0.07 mm clearance. Disabled by default (`SCREEN_STIFF=1` to enable) |

### V0 result (`python ur5_screen.py --test`)
- Full sequence Home → Approach → Grasp → Lift → Transport (gap between posts 2–3) → Plug → Release → Retreat → Return runs; plug fully inserted, self-holding after release, housing face **0.49 mm** from the socket (design 0.5 mm).
- Grip force 26–28 N per pad (recommended 12–30 N); TCP speed while carrying ≤ 0.25 m/s, acceleration ≤ 0.8 m/s².
- **Not met**: pin axis offset 0.19 mm at the socket face, 0.11 mm at the blade tip, ~1.0° tilt (requirement < 0.07 mm). Insertion succeeds thanks to MuJoCo's default contact softness (~0.1 mm penetration).
- Limitations: insertion force only indirect (~7 N), no grip-force control; cuRobo uses bounding boxes for posts; one-way cable; "no wrapping" checked by video only; repeatability not tested.

### Issue leading to V2
After `plans/guide/spec` were corrected, it became clear that **V0 misread the environment**: the 3 posts must be in line along the X axis (robot → posts → wall), and the cable must be threaded around the posts, not just pass through one gap.

---

## V2 – `ur5_screen_V2.py`: redo per the corrected description (staggered weave through 3 posts)

### Changes vs V0
| Item | V0 | V2 |
|---|---|---|
| Posts | 3 posts r=2 cm, H=40 cm, side by side | 3 posts D=5 cm, H=30 cm, in line robot–wall, post 1 at 0.50 m from robot base, 0.20 m apart |
| Trajectory | Through gap between posts 2–3 | Staggered: post 1 (+Y) → gap → post 2 (−Y) → gap → post 3 (+Y) → socket; each leg planned by cuRobo (joint-space, continuous IK); wrist rotates about Z by 90° at gap 1–2 and 60° at gap 2–3 |
| Cable | 0.9 m, tail extends far away | 1.2 m, tail folded into two parallel runs joined in a U so it gets dragged along the weave path |
| Wall / socket | 1.6×0.9 m | 1.0×0.6 m, socket face at x = 1.04 m, socket at y = +0.11 m, z = 0.27 m |
| Gripper | 70° tilt | 40° tilt (reaches the far wall; wrist clears post tops) |
| Video | Fixed camera | Far camera, plug trajectory trail (cyan) |

### Symptom → hypothesis → change → result
| # | Symptom | Hypothesis | Change | Result |
|---|---|---|---|---|
| 1 | Cable "snaps" when touching post 2 | `--stretch-stiffness` = 10 too soft: when taut (dragged to the post, floor friction) segments stretch apart and look broken; a long tail makes it tauter | Raise to 1e6 (as in Newton's cable examples), fold the tail | Continuous cable in V2 video *(but causes the V3 bug)* |
| 2 | cuRobo can't pass between two posts | 11 cm plug + 9 cm wide gripper exceeds the gap | Rotate hand about Z through the gap (plug along travel direction), posts 0.2 m apart | Passes the gap |
| 3 | cuRobo fails although gripper spheres look fine | Wrist sphere (z ≈ 0.33–0.35 m) hits post top (0.30 m); `clearance()` only considered the gripper | Raise plug height while weaving to 0.27 m, tilt 40°; `clearance()` uses finite cylinders; probe with cuRobo spheres | Planning succeeds |
| 4 | IK has no solution at gap 2–3 with +90° rotation | Outside reach (front branch) | Use 60° rotation at gap 2–3 | Solution found |
| 5 | IK goal lands on another branch / exceeds joint limits | Local IK | `ik_near`: multiple seeds, choose the solution closest to the previous configuration within joint limits; plan with `plan_cspace` | Continuous solutions |

### V2 result (`python ur5_screen_V2.py --test`, 41.5 s sim, 586 s wall-clock)
- Full sequence Home → Approach → Grasp → Lift → weave posts 1-2-3 → Plug → Release → Return runs; plug fully inserted, self-holding, housing face 0.45 mm from the socket. Grip ≈ 27 N/pad, TCP ≤ 0.25 m/s.
- **Not met**: pin error 0.19/0.12 mm, ~1° tilt.
- **New bug seen in the video**: while the plug is being inserted, the cable segment wrapped around post 2 suddenly passes through the post → V3.

---

## V3 – `ur5_screen_V3.py`: fix cable passing through post 2

### Symptom
Measured on the V2 trajectory: the cable penetrates post 2 by up to **17.8 mm** (post radius 25 mm) during t = 24.4–25.5 s; posts 1 and 3 are unaffected.

### Hypothesis
Cable–post collision in Newton/VBD is penalty contact with `ke = 1e3` N/m. In V2, to stop the cable "snapping", the stretch stiffness was raised from 10 to 1e6 ⇒ the cable barely stretches; when the plug tensions the cable around post 2, the tension far exceeds the maximum push-back of the soft contact ⇒ the cable is forced through. The two parameters are mismatched: **stiff cable, soft contact**.

### Verification
Replay the exact V2 plug trajectory through the cable model only (temporary script, outside the project) with different `ke`:

| ke | Post 2 penetration |
|---|---|
| 1e3 (V2) | 22.8 mm (bug reproduced) |
| 1e5 | None (min gap −0.2 mm) |
| 1e7 | None (+0.1 mm) |

Hypothesis confirmed.

### Change
- Expose cable contact stiffness/damping as parameters: `--cable-contact-ke` (default **1e5**, was 1e3) and `--cable-contact-kd` (default **10**). 1e5 chosen over 1e7 to avoid numerical instability from overly stiff contact.
- Robot trajectory, cuRobo, and geometry unchanged from V2.

### V3 result (`python ur5_screen_V3.py --test`, 41.5 s sim)
| Post | Min cable–post gap | Frames with penetration > 2 mm |
|---|---|---|
| 1 | 41.4 mm | 0 |
| 2 | −0.2 mm (touching) | 0 |
| 3 | 68.1 mm | 0 |

No post penetration; max segment stretch ratio 1.005. Plug fully inserted, self-holding (0.45 mm). Pin error 0.19/0.12 mm and ~1° tilt are still **not met**.

---

## Remaining limitations (as of V3)
- Pin error > 0.07 mm; insertion succeeds only through MuJoCo's contact softness.
- Cable acts one-way on the plug (no back-pull); cable tension not measured; 1e5 N/m contact still allows ~0.2 mm penetration under high tension (not visible).
- Cable is 1.2 m (spec's "10–20 m" can't be simulated); UR5e instead of UR5; post D = 5 cm (smallest allowed by spec).
- Straight segments (Descend, Lift, Insert, Retreat) are not arm-collision-checked by cuRobo; weave legs stop at each intermediate point.
- Repeatability across runs not tested.

## Future work
1. Force/vision servoing during insertion (find the hole by force) to get < 0.07 mm; stiff contact + feedback alignment.
2. 1.5 mm × 45° lead-in chamfer at slot/hole mouths (per spec) to tolerate ±5° misalignment.
3. Feedback-controlled grip force (15–30 N) to avoid the 12 mm slip on the first lift.
4. cuRobo checks on straight segments; two-way cable–plug coupling; measure cable–post distance and cable tension; continuous trajectory without stops between waypoints.

## How to re-run
```
cd data/scripts
python ur5_screen.py --test            # V0 (video: ur5_screen_render_isaac.py)
python ur5_screen_V2.py --test         # V2
python ur5_screen_V3.py --test         # V3
python ur5_screen_render_isaac_V3.py   # data/results/video_ur5_screen_V3.mp4
```
Options: `--no-cable`, `--grip-effort`, `--v-carry`, `--replan`, `--cable-contact-ke`, `--cable-contact-kd` (V3).
