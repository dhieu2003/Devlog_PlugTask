# DEVLOG 2026-11-11 → 2026-11-14 – ur5_regrasp: self-collision avoidance, gripper-jaw offset when grasping the cable, and V6

This document merges the work of 11, 12, 13 and 14 November 2026. For every fault it records: **symptom → cause (measured / only a guess) → fix → result**. "Measured" means numbers read from the simulation logs; "guess" means not yet proven.

(English version of `DEVLOG_UR5_REGRASP_2026_11_11-14.md`; all source-code excerpts have been removed, the file names in the text point to the code.)

## 0. Summary

| Fault | Cause | Fix | Result |
|---|---|---|---|
| **A. Arm self-collision** (V1, V2; V4 was only "collision-free" because the IK branches happened to be suitable) | cuRobo **does** check self-collision (spheres of `ur5.yml`, binary constraint), but only for the segments it optimises itself; straight IK segments (`pose_line`, `line_segment`), the choice of IK branch and the simulation (`enable_self_collisions=False`) check nothing (sections 1.2–1.3, evidence in 1.1) | A `self_clearance` function (collision spheres of `ur5.yml`) filters IK solutions and straight paths, threshold 2 mm | Every segment of V5full/V1new/V2new/V6: clearance ≥ 4.1 mm (a structural value) |
| **B. Jaws off-centre when grasping the cable** (V1, V4) | The four-bar linkage **of one finger collapses** when the `spring_link` spring is too weak (k ≤ 0.125 N·m/rad) and the pad touches the cable at one point only | Grasp modes per spec 2.1.8: the cable is grasped in **Parallel (k = 0.2)**, the plug in **Encompassing (k = 0.05)** | V6: jaw offset **0.01–0.02 mm** (V4: ≈ 5 mm), the cable grasp point stays as in V4 (no 7 mm lift as in V5full) |

## 1. 11 Nov – arm self-collision avoidance (V5full, V1new, V2new, V6)

Terminology: a **joint** is a rotation axis; a **link** is the rigid body between two joints. The UR5 has 6 joints and 6 collision links (shoulder, upper_arm, forearm, wrist_1, wrist_2, wrist_3). Both cuRobo and my code check collisions between **links**, not between joints.

### 1.1 Symptom (with evidence)
**Initial observation:** in the V1 video the arm passes through itself while waving the cable; in V4 it does not. The simulation has `enable_self_collisions=False`, so the physics reports nothing and it is only visible in the video.

**Re-measured with numbers** (my `self_clearance`, on the original V1 plan `ur5_regrasp_V1_plan_regrasp.npz`; each segment is a sequence of joint configurations):

| V1 segment | Points | Penetrating points | Minimum clearance | Penetrating link pair | Worst configuration q [deg] |
|---|---|---|---|---|---|
| wave1 | 140 | 105 | **−64.1 mm** | forearm – wrist_3 | [26, −38, 34, 122, **−226**, −53] |
| wave2 | 80 | 80 | **−64.1 mm** | forearm – wrist_3 | same configuration (first point of wave2) |
| wave3 | 140 | 7 | **−10.3 mm** | forearm – wrist_3 | [−14, −59, 60, 73, **−222**, −111] |
| other segments | — | 0 | 4.1 mm | — | — |

The original V2 shows a similar problem at the cable-pick step: base-joint branch 294° and the straight descent to the cable have a clearance of **−19.4 mm** (measured the same way).

### 1.3 Cause
Since the forearm–wrist_3 pair is in the group checked by cuRobo, the problem is not that cuRobo checks wrongly but that **the penetrating segments never pass through cuRobo's checker**.

**Check 1 – cuRobo scoring the V1 plan also reports collisions.** I ran cuRobo's own checker (`RobotCollisionChecker.get_scene_self_collision_distance_from_joints`, same `ur5.yml`) on the points of the V1 plan (script `data/scripts/check_curobo_self_collision_V1.py`, run with `PYTHONPATH=~/.cache/cable_task/np2` because cuRobo needs numpy 2):

| V1 segment | Points | Points flagged by cuRobo | Points where my function gives < 0 |
|---|---|---|---|
| approach | 121 | 0 | 0 |
| wave1 | 140 | 108 | 105 |
| wave2 | 80 | 80 | 80 |
| wave3 | 140 | 10 | 7 |

(The "penetration depth" values of the two checkers use different scales and are not compared directly; only the number of flagged points is comparable.) The two checkers agree: **the checker is not wrong, but the final plan still contains points it flags as in collision**.

**Check 2 – those segments were not optimised by cuRobo.** In `ur5_regrasp_curobo_V1.py` each wave leg is produced in this order: `pose_line` (TCP straight-line interpolation with chained IK at every point), `line_segment` (straight IK segment), or a reversed earlier segment; only when those fail is cuRobo's `plan_cspace` called. Segments optimised by cuRobo have a point count of the form 20k + 1 (101, 121, 141, 161 in the V1new log), whereas wave1 = 140, wave2 = 80, wave3 = 140 points do not. This is an **inference from point counts and code structure**; I did not re-run V1 to confirm each segment.

In V1, `pose_line`, `line_segment`, `joint_line` and the IK-branch selection (`ik_near`, `ik_candidates`, which use my numerical IK `G.ik`) only check the IK error, the joint jump and the clearance to posts/wall/floor, and **do not check self-collision**. In summary:

| Component | Checks self-collision? |
|---|---|
| `plan_pose`, `plan_cspace` (cuRobo trajopt and internal IK) | **Yes** (binary constraint, `ur5.yml` spheres) |
| `pose_line`, `line_segment`, `joint_line` (my code) | **No** (V1, V4) |
| `ik_near`, `ik_candidates` (IK-branch selection, my code) | **No** (V1, V4) |
| Newton/MuJoCo simulation | No (`enable_self_collisions=False`) |

**What does "V4 is collision-free" mean?** I once said "V4 being collision-free may be luck". Re-measured on 1782 configurations taken from the V4 plans: minimum clearance 4.1 mm (forearm–wrist_2 pair). V4 really is collision-free, but **no mechanism guarantees it**; the V4 IK branches happen not to fall into the penetrating region, so changing the IK branch could cause penetration as in V1.

### 1.4 Fix
Put cuRobo's own measurement (same spheres, same ignore table) **in the places cuRobo does not cover**, then reject every violating solution/segment.

**Equations.** For a joint configuration q, sphere i of link a has centre c_i(q) (world frame, from forward kinematics `link_frames(q)`) and radius r_i. The clearance between two links a, b and the clearance of the whole arm:

$$ d_{ab}(q) = \min_{i \in a,\ j \in b} \left( \|c_i(q) - c_j(q)\| - r_i - r_j \right) $$

$$ d_{\min}(q) = \min_{(a,b) \in P} d_{ab}(q) $$

Notation:
- `a, b`: two different links; `i`, `j` run over the spheres of a and b.
- `c_i(q)`, `c_j(q)`: sphere centres; `r_i`, `r_j`: radii; `||.||`: Euclidean distance. The bracketed expression is the clearance between **two sphere surfaces**: positive = separated, negative = penetrating.
- `P` = `SELF_PAIRS` = the 9 pairs of the "checked" table in section 1.2 (the 6 ignored pairs are not in P and were **copied from `self_collision_ignore` in `ur5_build_assets_V4.py`** so that both sides use the same rule). While carrying the plug, P gets 3 more pairs: (shoulder, plug), (upper_arm, plug), (forearm, plug).

Acceptance rule (δ = `SELF_MARGIN` = 2 mm):

$$ d_{\min}(q) \ge \delta \quad \text{(configuration)}, \qquad \min_k d_{\min}(q_k) \ge \delta \quad \text{(path } q_1 \dots q_N \text{)} $$

Why δ = 2 mm: the smallest structural value of the UR5 is 4.1 mm (forearm–wrist_2), so δ = 2 mm rejects genuinely folded postures (−10 to −68 mm) without blocking normal ones; a δ larger than 4.1 mm would block even the Home posture.

**Where the check is applied** (functions in `ur5_regrasp_geom_V6.py` and `ur5_regrasp_curobo_V6.py`):
- **Measurement function** (`ur5_regrasp_geom_V6.py`): `self_clearance(q, carrying)` implements the two equations above using the sphere model and the 9-pair table (plus the 3 plug pairs when carrying), with `SELF_MARGIN = 0.002`; `path_self_clearance` takes the minimum over a whole path.
- **IK-solution filter** (`ur5_regrasp_curobo_V6.py`, in `ik_near`; `ik_candidates` has the same condition): a candidate solution is kept only if d_min ≥ δ, in addition to the existing IK-error and joint-limit conditions.
- **Rejecting penetrating straight-IK paths / joint paths** (end of `pose_line`; `joint_line` does the same): this is where V1 had no check. If `path_self_clearance` of the path is below `SELF_MARGIN`, the function returns `None` and the planner tries another segment or branch.
- **Final guard for every segment**: after the plan is built, the planner prints `self-collision clearance` for each segment and raises `RuntimeError` if it is < 0, so the plan can never silently pass a penetrating segment.

**Negative-case verification** (the function must catch real collisions): the folded posture `[0, −0.1, −3.1, −1.5, 0, 0]` gives −68 mm, `[0, −2.9, −3.0, 0, 0, 0]` gives −40 mm.

### 1.5 Result
Minimum clearance of each segment (from the `[check] ... self-collision clearance` lines in the logs):

| Version | Before | After |
|---|---|---|
| V1 → V1new | wave1 −64.1 mm, wave2 −64.1 mm, wave3 −10.3 mm | all three **4.1 mm** (different IK branch: wrist_1 turned 171°, wrist_3 up to −291°) |
| V2 → V2new | cable-pick step −19.4 mm (base-joint branch 294°) | **4.1 mm** (cuRobo returns a base-joint branch ≈ 11°) |
| V4 → V5full / V6 | 4.1 mm (no mechanism guaranteeing it) | 4.1 mm in every segment, **with a mechanism** (the plan is stopped if < 0) |

Consequences to note:
- In V2 the only collision-free branch is the non-spinning branch; "no more collision" and "no more extra spin" are one and the same change. But with that branch the cable-laying segment `lay` makes the arm **touch a post (−10.5 mm)** for all 10 nearest IK solutions, so V2new only runs until the cable has been picked.
- V1new runs to the end but **the cable slips out at gap23_in** (cable–TCP distance 5.3 → 7.4 → 7.1 → 9.6 mm, then lost). It is not known whether this is a pre-existing V1 fault or caused by the new IK branch (the original V1 would have to be run for comparison).
- The 4.1 mm clearance is a structural value, not a safety margin; δ = 2 mm is a manually chosen threshold.

## 2. 11–12 Nov – investigating the off-centre jaws when grasping the cable

### 2.1 Symptom (measured)
When grasping the cable, the **two driver angles are equal** (difference 0.0000 rad) but the two pads are not symmetric about the cable. The offset index:

$$ o = R_w^{T} \left( \frac{p_R + p_L}{2} - p_w \right) - t_{TCP}, \qquad \Delta_{jaw} = o_x $$

where $p_R$, $p_L$ are the positions of the two pads; $p_w$, $R_w$ are the position and rotation matrix of wrist_3; $t_{TCP}$ is the TCP offset in the wrist_3 frame; o is a 3-component vector in the wrist_3 frame and o_x is the component along the closing direction of the two jaws. Grasping the plug: o_x = 0.03 mm. Grasping the cable in V4: o_x ≈ 4.7–5.2 mm (pads +36.7 / −27.4 mm); **the side that is off changes between runs**.

### 2.2 Hypotheses rejected by measurement
| Hypothesis | Experiment | Result |
|---|---|---|
| The two PD controllers of the two jaws lose synchronisation | one PD for both jaws (right finger ×2, left finger ke = kd = 0) | offset unchanged |
| Closing amount / grip force | close 0.79 instead of 0.8; cable grip force 0.8 instead of 2.2 N·m | unchanged |
| Fingertip touches the floor | contact measurement | the symmetric case touches the floor even more |
| The cable deforms after insertion so the cable TCP is higher (user's hypothesis) | read the code: the grasp point is computed from the **current** cable position; the cable is only 4 mm above rest | wrong |
| The cable position on the pad decides the symmetry | T3 (lift 7 mm) and T4 (old height) have the cable at the same position relative to the TCP but give different results | wrong (section 2.3) |

### 2.3 Variable-separation experiments (11–12 Nov, `--regrasp-only`, cable placed at a given position)
| | Condition | Right / left pad | Conclusion |
|---|---|---|---|
| T1 | old height, no cable | +25.3 / −25.3 | symmetric |
| T2 | cable placed at z = −16 mm | — | **invalid**: the cable is under the floor and gets pushed away |
| T3 | lift 7 mm, cable y = −1, z = −5 | +11.7 / −11.7 | symmetric, 3 contact points per side |
| T4 | old height, cable y = −1, z = −5 | **+36.4 / −26.6** | off-centre, right finger 1 contact point |
| T5 | old height, cable y = +12 | **+26.2 / −36.5** | off-centre at the **left** finger (1 contact point) |
| T6 | old height, cable z = +2 | — | **invalid**: the cable is pushed out |
| T7 | lift 3 mm | **+36.2 / −23.5** | off-centre, right finger 1 contact point |

Measured rule: **the offending finger is always the finger that touches the cable at only 1 point**, and which side is off changes. Joint angles of the offending finger: coupler ≈ −0.70, spring_link ≈ 0.155, **follower = 0.873**; the other finger: −0.52, 0.38, 0.28.

### 2.4 Temporary fix in V5full
Lifting the cable grasp point by 7 mm → offset 0.03 mm, but this differs from V4 and the cause is unexplained (lifting 3 mm still gives a 6 mm offset; 5 mm and above is symmetric).

## 3. 12–13 Nov – cause: the `spring_link` spring is too weak

### 3.1 Finger structure (read from `2f85.xml` of MuJoCo Menagerie)
Each finger is a closed four-bar linkage with two branches that meet at the pad:
- the **driving branch**: base → driver → coupler (the motor acts on the driver);
- the **passive branch**: base → spring_link (the sprung joint) → follower → pad (the pad touches the object); the follower is joined to the coupler by a `connect` constraint.

The spring_link joint: range [−0.297, 0.8] rad, `stiffness` = 0.05 N·m/rad, `springref` = 2.62 rad (outside the range, so the spring always pushes spring_link towards the upper limit). The spring torque:

$$ \tau_s = k \, (q_{ref} - q_s), \qquad q_{ref} = 2.62 \ \mathrm{rad} $$

At q_s = 0.4 rad: k = 0.05 gives tau_s = 0.05 × (2.62 − 0.4) ≈ 0.11 N·m; k = 0.2 gives tau_s ≈ 0.44 N·m (driver torque limit 3.2 N·m). The follower has limits ±0.872664 rad.

### 3.2 Hypothesis and how it was tested
Hypothesis: when one finger touches the cable at one point only, the off-centre force produces a moment on the passive linkage; the k = 0.05 spring is too weak to hold it, so spring_link falls to ≈ 0 and the follower rotates to its limit: the finger **collapses into a second configuration**, the pad moves back → jaw offset. Tested by overriding `dof_passive_stiffness` of the two spring_link joints and re-running case T4.

### 3.3 Stiffness sweep, cable grasp (old-height case, `--regrasp-only`)
| k [N·m/rad] | Right / left pad [mm] | Result |
|---|---|---|
| 0.02 | +36.7 / −27.3 | off-centre |
| 0.05 (original) | +36.4 / −26.6 | off-centre |
| 0.1 | +36.5 / −26.1 | off-centre |
| 0.12 | +37.0 / −26.7 | off-centre |
| 0.125 | +34.1 / −30.1 | slightly off-centre |
| **0.15** | **±30.05** | **symmetric**, 1 contact point per side, 42 N |
| **0.2** | **±28.85** | **symmetric**, 1 contact point per side, 39 N; unchanged with the cable at y = +12 (±29.1), y = −1 (±28.7) or lifted 3 mm |
| 0.3 | ±38 | cannot hold the cable (driver 0.255, pad presses the floor with 55 N) |
| 0.5 / 1 / 5 / 50 | ±36 / ±25 / ±5.7 / ±4.7 | cannot hold the cable, pad presses the floor with 38–146 N |

→ **The hypothesis is partly right**: k ≥ 0.15 prevents the offset, but too high a k destroys the grasp posture (window k ≈ 0.15–0.2).

### 3.4 The price of k = 0.2 at the plug-grasp stage
Running the whole task with a fixed k = 0.2 → **the plug insertion fails** (axis offset 7.8 mm, tilt 33° before release; the plug flies off and the simulation produces NaN). A stiffness sweep for the plug insertion only (`--no-regrasp`):

| k | Axis offset before release / tilt | |
|---|---|---|
| 0.05 | 0.112 mm / 0.41° | good |
| 0.1 | 0.112 mm / 0.34° | good |
| 0.12 | 0.81 mm / 3.35° | worse |
| 0.125 | 1.43 mm / 8.2° | failed |
| 0.15 | 5.15 mm / 23.8° | failed |
| 0.2 | 7.84 mm / 33.3° | failed |

Cause (measured, k = 0.2 vs 0.05 when grasping the plug): the driver stops at 0.351 instead of 0.544 rad; the finger moves like a rigid parallelogram (coupler ≈ 0 instead of −0.33); at the end of the grasp step **the two pads have not yet touched the plug** (they only press the floor with 13 and 129 N, compared with 55.7 N on the plug at k = 0.05); the plug is held 9 mm off in y and 6 mm off in z relative to the original. Why the offset holding position makes the insertion fail has **not been measured** (possibly the plug rotates in the gripper under the insertion force).

→ **No single stiffness satisfies both**: the plug needs k ≤ 0.1, the cable needs k ≥ 0.15.

### 3.5 k = 0 (Encompassing in the first version of the spec) is not usable either
With k = 0 when grasping the plug: the left finger collapses (spring_link 0.053; follower 0.873), the plug is held tilted by 35°, `insertion IK failed`. So finger collapse happens **also with the plug** when there is no spring; k = 0.05 is enough to keep it stable for the plug (large central grip force, many contact points). The spec was later corrected by the user to k = 0.05.

### 3.6 Relaxing the follower limit (13 Nov)
If the finger were only "stuck at the 0.873 stop", relaxing the limit should remove the offset. Result:

| Follower limit | follower / spring_link of the offending finger | Result |
|---|---|---|
| ±0.873 (original) | 0.873 / 0.156 | off-centre, still holds the cable |
| ±1.2 | 1.19 / 0.068 | worse offset, **does not hold the cable** |
| ±1.57 | 1.34 / 0.032 | worse offset, does not hold the cable |

→ The 0.873 limit is not the cause; it only **stops the collapse from going further**. A collapsed finger does not stop when the stop is removed: spring_link → 0 and the follower keeps rotating. This is an instability of the linkage, not friction or a hard stop.

## 4. 14 Nov – V6: grasp modes per spec.md section 2.1.8

### 4.1 Formula
Each finger makes one friction contact, so for an object of mass m not to slip under acceleration a:

$$ 2 \mu F \ge m (g + a) \quad \Rightarrow \quad F_{sw} = \frac{m (g + a)}{2 \mu} \, S $$

With m = 2.5; g = 9.81; a = 0.8 m/s² (`--a-carry`); mu = 0.7 (`PAD_MU`, rubber/PVC); S = 2.5:

$$ F_{sw} = \frac{2.5 \times (9.81 + 0.8)}{2 \times 0.7} \times 2.5 = 47.37 \ \mathrm{N} $$

- F < F_sw: **Parallel Grasp**, spring_link spring k = 0.2 N·m/rad.
- F ≥ F_sw: **Encompassing Grasp**, k = 0.05 N·m/rad.

### 4.2 Grip force F per finger
Not measured directly at decision time; estimated from the driver torque limit, calibrated on the plug grasp measurement (56.4 N at 3.2 N·m):

$$ F = 17.6 \cdot \tau_{max} \qquad \Rightarrow \qquad F_{plug} = 56.4 \ \mathrm{N}, \quad F_{cable} = 38.8 \ \mathrm{N} $$

where tau_max is the driver torque limit (plug: 17.6 × 3.2 = 56.4 N; cable: 17.6 × 2.2 = 38.8 N).

| Object | F | F_sw | Mode | k |
|---|---|---|---|---|
| Plug | 56.4 N | 47.37 N | Encompassing | 0.05 |
| Cable | 38.8 N | 47.37 N | Parallel | 0.2 |

Contact force measured when grasping the cable at k = 0.2: 39–43 N per side, consistent with the 38.8 N estimate.

### 4.3 How it is built into V6
- `grasp_mode(effort, a_acc)` computes F_sw and F and returns (mode, k, F, F_sw); the constants are K_PARALLEL = 0.2, K_ENCOMPASSING = 0.05, GRASP_S = 2.5, GRASP_MASS = 2.5 and F_PER_NM = 56.4 / 3.2.
- `Example.__init__`: the model is built with the k of the **plug** mode (written into `mujoco:dof_passive_stiffness` of the two spring_link joints).
- `regrasp_phase`: right before approaching the cable, `grasp_mode` is called for the **cable** and then `set_spring_k(k)` rewrites the stiffness table and calls `notify_model_changed(JOINT_DOF_PROPERTIES)` so the solver reloads `jnt_stiffness`.
- The cable grasp point **stays as in V4** (no 7 mm lift); the follower limit **stays at 0.873**.

### 4.4 V6 result (one run of `ur5_regrasp_V6.py`, re-run after source clean-up with the same numbers)
| Metric | V4 / V5full | **V6** |
|---|---|---|
| Jaw offset when grasping the plug | 0.03 mm | 0.03 mm |
| Jaw offset when grasping the cable | V4: ≈ 5 mm; V5full: 0.03 mm (lift 7 mm) | **0.01–0.02 mm** (no lift) |
| Driver angle when grasping the cable (R / L) | 0.776 / 0.776 | 0.652 / 0.652 |
| Plug insertion: axis offset before release / tilt | 0.112 mm / 0.41° | 0.111 mm / 0.414° |
| Maximum plug displacement while waving the cable | 0.95 mm (V5full) | 1.058 mm |
| Plug displacement after laying the cable | 0.51 mm (V5full) | 0.620 mm |
| Cable–TCP distance while grasping | 19.6–20.2 mm (V5full) | 21.1–21.5 mm |
| Cable dropped | no | no |
| Self-collision clearance | 4.1 mm | 4.1 mm |

## 5. My mistakes (so you know when re-reading)
- Saying that V4 avoids self-collision "maybe by luck" before measuring → wrong (section 1.3).
- Repeatedly suspecting "fingertip touches the floor" and "the two PDs are not synchronised" as the cause of the offset → both wrong (section 2.2).
- The hypothesis "the cable position on the pad decides" → refuted by T3/T4 (section 2.3). Experiments T2 and T6 were invalid because the cable was placed under the floor / outside the gripper.
- On 12 Nov a background wait process used `pgrep -f` that matched itself and **never terminated**, wasting more than an hour of waiting before the user reminded me.
- I first said my configuration differs from Menagerie in the `solref` of the driver constraint; on re-reading the file the value 0.005 is **Menagerie's own**. The real difference is only two separate driver actuators (kp = 20, kd = 1) instead of one tendon actuator `split`, and the "single PD" experiment did not change the result.
- Running V6 with a fixed k = 0.2 for the whole task broke the plug (section 3.4); choosing k per stage is what works.
- At first I read the F_sw formula with m per object (plug 0.1 kg, cable ≈ 0.11 kg), giving F_sw ≈ 1–1.5 N and the cable ending up in Encompassing; the formula was corrected by the user to m = 2.5, S = 2.5.

## 6. Limitations and next steps
1. V6 has not been run repeatedly to measure stability; earlier runs were non-deterministic (plug displacement 1–4 mm, sometimes NaN).
2. There is no mechanical analysis of the stability condition of the four-bar linkage; only joint-angle measurements and a stiffness sweep. The window k ≈ 0.15–0.2 for cable grasping is very narrow (k = 0.3 already fails).
3. F is estimated from the torque limit; F should be measured directly at grasp time and used for the mode decision. For the cable the margin to the threshold is only 8.6 N (38.8 vs 47.37 N).
4. The friction mu in F_sw uses `PAD_MU = 0.7` for both the plug and the cable, while the cable proxy in the simulation has mu = 1.2.
5. Finger collapse can still occur if k or the grip force is changed; adding damping to spring_link is another way to prevent the collapse with little effect on plug grasping (not tried).
6. V1new: the cable slips at gap23_in (not known whether it already exists in V1). V2new: the cable-laying segment touches a post (−10.5 mm), only runs until the cable is picked.
7. The origin of the colliding V1 segments (wave1–wave3 not optimised by cuRobo) is inferred from the point counts of the segments and the V1 code structure; V1 was not re-run to confirm each segment.
8. The simulation still has `enable_self_collisions` off; self-collision safety relies on the sphere model, which has not been compared with meshes at extreme postures.

## 7. Files and how to re-run
| File (`data/scripts/`) | Content |
|---|---|
| `ur5_regrasp_V6.py` | main simulation (cleaned, commented line by line) |
| `ur5_regrasp_geom_V6.py` | geometry, `self_clearance`, `path_self_clearance` |
| `ur5_regrasp_curobo_V6.py` | cuRobo planner (self-collision filtering of IK solutions) |
| `ur5_regrasp_render_isaac_V6.py` | Isaac video rendering |

Video: `data/results/video_ur5_regrasp_V6.mp4`. Log: `data/results/run_V6.log`.

To re-run: from `data/scripts`, run `ur5_regrasp_V6.py` with the `--test` option for the full V6 sequence (about 16 minutes), then `ur5_regrasp_render_isaac_V6.py` with `--stride 3` for the video.

## 8. Where the code is (code excerpts removed)
The jaw-offset fix lives in `ur5_regrasp_V6.py`:
- `grasp_mode` and its constants: choose the mode from F and F_sw (sections 4.1–4.2).
- `Example.__init__`: sets the initial spring stiffness for the plug mode on the two spring_link degrees of freedom and prints the chosen mode.
- `set_spring_k`: rewrites the passive-stiffness table of the two spring_link degrees of freedom and notifies the MuJoCo solver of the joint-property change, at run time.
- `regrasp_phase`: picks the grasp mode for the cable and calls `set_spring_k` just before the jaws close on the cable.

The self-collision code is in `ur5_regrasp_geom_V6.py` (`SELF_IGNORE_PAIRS`, `SELF_PAIRS`, `SELF_MARGIN`, `self_clearance`, `path_self_clearance`) and `ur5_regrasp_curobo_V6.py` (`ik_near`, `ik_candidates`, `pose_line`, `joint_line`, final check); see section 1.4.
