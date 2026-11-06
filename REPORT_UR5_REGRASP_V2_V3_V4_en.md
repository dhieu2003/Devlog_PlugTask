# Comparison of V1, V2 and V3: UR5 plug insertion and cable re-grasp

Task: a real UR5 (not the UR5e) with a Robotiq 2F-85 gripper picks up a plug at the end of a cable, threads it through 3 posts, inserts it into a wall socket, releases it, then comes back, grasps the cable, waves it through post 1, gap 1-2, post 2 and gap 2-3, and lays it on the floor. At the end the plug must still be in the socket. 

The Re-grasp point: 

$$
0.15 \le d \le 0.62
$$

The three versions compared:

| Version | Status |
|---|---|
| **V1** | Faults when grasping the cable: the arm makes a needless full turn, and the cable slips out of the gripper at gap 2-3 |
| **V2** | Both faults above are gone, but the wrist turns at gap 2-3 and displaces the plug |
| **V3** | Finished and accepted |

---

## 1. V1: two faults while grasping and waving

[https://github.com/user-attachments/assets/7ce4fc58-5a05-49f0-a0ee-8c894b114966](https://github.com/user-attachments/assets/7ce4fc58-5a05-49f0-a0ee-8c894b114966)

### 1.1 Fault A: needless full turn of the arm

**Symptom.** When approaching the cable for the re-grasp, the base joint turns 286 degrees and wrist 6 turns 229 degrees. The total base rotation over the whole re-grasp phase is 687 degrees. The arm makes large unnecessary swings. While fixing this, the log also records one cuRobo branch that swung the gripper 3.04 m up to z = 1.14 m, jerked the cable and pulled the plug out, which blew up the simulation (NaN).

**Cause.** The same TCP pose has several joint solutions. Two sources of multiple solutions:

- The UR5 has several IK branches (shoulder, elbow, wrist flips).
- Every joint has a limit of $\pm 2\pi$, so $q_j$ and $q_j \pm 2\pi$ give the same pose. A solution can be off by exactly one full turn from the current pose.

In V1, `plan_pose` left cuRobo to solve the IK and choose the solution itself, without considering how far that solution is from the current pose. So it could pick a solution a full turn away.

**Fix.** Three changes. They solve two different problems, which are kept apart below:

- Changes 1 and 2 are for goals given as a **TCP pose** (position and orientation). Such a pose has many IK solutions, and we must choose among them.
- Change 3 is for goals given directly as **joint angles** (Home). There is no IK to solve and no set of solutions to rank. The only issue is that the stored angles may differ from the current ones by whole turns.

1. Generate many IK solutions and sort them by the largest joint displacement from the current configuration $q_{ref}$:

$$
D(q) = \max_{i=1..6} \left| q_i - q_{ref,i} \right|
$$

Where:
- $q = (q_1,\dots,q_6)$ is a candidate IK solution: the 6 joint angles of the UR5 [rad], ordered from the base (shoulder pan) to wrist 3.
- $q_{ref}$ is the current joint configuration of the robot (where the motion starts), also 6 angles [rad].
- $q_i - q_{ref,i}$ is the difference of joint $i$ between the candidate and the current configuration.
- $\max_i |\cdot|$ takes the joint that moves the most. The maximum (not the sum) is used because we want no joint to travel a large distance, and in particular no joint to be a full turn $2\pi$ away.
- $D(q)$ [rad] is the "largest travel of any single joint" needed to go from $q_{ref}$ to $q$. The solution with the smallest $D$ comes first in the list.

The solution with the smallest $D$ is the "no spin" branch. cuRobo only plans to the first solutions of the list (at most 10) and stops at the first solution with $D > 200$ degrees (later ones are even farther because the list is sorted).

2. Each IK solution is found with damped least squares (DLS), started from several points near $q_{ref}$:

$$
\Delta q = J_a^{\top}\left(J_a J_a^{\top} + \lambda^2 I\right)^{-1} e,\qquad \lambda^2 = 10^{-3}
$$

   Where:
   - $\Delta q$ (6 elements, [rad]) is the joint correction of one iteration; then $q \leftarrow q + \Delta q$ and everything is recomputed.
   - $J_a$ is the 6x6 Jacobian at the current $q$. Column $j$ tells how the TCP moves and rotates when joint $j$ turns by 1 more rad (the first 3 rows are position [m/rad], the last 3 rows are orientation). The code computes it by finite differences: each joint is perturbed on its own by $10^{-6}$ rad and forward kinematics is called again.
   - $J_a^{\top}$ is the transpose of $J_a$; $I$ is the 6x6 identity matrix.
   - $\lambda^2 = 10^{-3}$ is the damping factor (`damping`). It keeps $J_a J_a^{\top} + \lambda^2 I$ invertible and prevents $\Delta q$ from becoming huge near singularities (where the Jacobian degenerates).
   - $e$ is the 6-dimensional error: $e = [\,p_{tgt}-p \;;\; e_R\,]$, where $p_{tgt}$ is the target TCP position [m], $p$ is the current TCP position [m] (from forward kinematics of $q$), so $p_{tgt}-p$ is the vector from the current position to the target; $e_R$ is the orientation error (formula below).
   - The loop stops when $\|p_{tgt}-p\| < 2\cdot 10^{-6}$ m and $\|e_R\| < 10^{-5}$ rad, or when the maximum number of iterations is reached.

   The orientation error $e_R$ is taken from the skew-symmetric part of the missing rotation matrix:

$$
e_R = \tfrac12\mathrm{vee}\left(R_{t}R_{c}^{\top} - \left(R_{t}R_{c}^{\top}\right)^{\top}\right)
$$

   Where:
   - $R_t$ is the 3x3 target rotation matrix (desired hand orientation), $R_c$ is the current rotation matrix; $R_c^{\top}$ is the transpose of $R_c$, which is also its inverse.
   - $R_t R_c^{\top}$ is the rotation still to be applied to go from the current orientation to the target one. If the two orientations coincide it is the identity matrix.
   - $M - M^{\top}$ (with $M = R_t R_c^{\top}$) is the skew-symmetric part, which keeps only the information about the axis and angle of the rotation.
   - $\mathrm{vee}(\cdot)$ reads the skew-symmetric matrix $\begin{bmatrix}0&-c&b\\ c&0&-a\\ -b&a&0\end{bmatrix}$ as the vector $(a,b,c)$. In the code: $e_R = \tfrac12\,(M_{32}-M_{23},\; M_{13}-M_{31},\; M_{21}-M_{12})$.
   - The result is $\sin\theta\cdot \hat{n}$, where $\theta$ is the missing angle and $\hat{n}$ the unit rotation axis. For small angles it is $\approx \theta\,\hat{n}$ [rad], and it is 0 when the orientation is already right.

3. When returning to Home, the goal is already a fixed joint vector `HOME_Q = [0, -pi/2, pi/2, -pi/2, -pi/2, 0]`. The stored angles are one representation of the Home posture, but after the previous motions the current angle of a joint may differ from it by a whole turn (for example wrist at +300 deg while Home says -60 deg: the same posture, but going straight to -60 deg unwinds the wrist by about 360 deg). The goal is therefore replaced by its equivalent modulo $2\pi$ that is closest to the current angles:

$$
q_j' = \arg\min_{k\in\{-1,0,1\}} \left| q_j + 2\pi k - q_{cur,j} \right|,\qquad \left|q_j'\right| < 2\pi - 0.1
$$

Where:
- $q_j$ is the stored goal angle of joint $j$ [rad] (it may be a full turn away from the current one).
- $q_{cur,j}$ is the current angle of joint $j$ [rad].
- $k \in \{-1, 0, 1\}$ is the number of full turns ($2\pi$) added or subtracted. The angle $q_j + 2\pi k$ is the same physical joint posture, only with a different number of turns.
- $\arg\min$ picks the $k$ for which $q_j + 2\pi k$ is closest to $q_{cur,j}$, i.e. the joint takes the shortest way.
- The condition $|q_j'| < 2\pi - 0.1$ keeps the chosen angle inside the UR5 joint limit $\pm 2\pi$ (with a 0.1 rad margin). The selected $q_j'$ is the final goal angle of joint $j$.

**Result.**

| | V1 | V2 and V3 |
|---|---|---|
| Base joint rotation on approach | 286 deg | **23 deg** |
| Wrist joint 6 on approach | 229 deg | 171 deg (needed to orient the gripper) |
| Total base rotation in re-grasp | 687 deg | **240 deg** |
| Return to Home | the wrist may unwind about 300 deg | straight line in joint space, at most 77 deg on one joint |

### 1.2 Fault B: the cable slips out of the gripper at gap 2-3

**Symptom.** On the leg from post 2 to gap 2-3 the cable slowly slides out of the pads: the cable height drops from z = 0.26 m to 0.02 m and the pad gap closes to 12.5 mm. One run ended in NaN.

**Cause (according to the log).** A cable grip effort of 0.9 N m and a friction coefficient of 0.7 on the proxy body were both too low, so the cable could not be held while it was pulled through the gap. The no-slip condition is that the tangential force on the cable does not exceed the friction of the two pads:

$$
F_{t} \le 2\,\mu\,F_n
$$

Where:
- $F_t$ [N] is the force along the cable (tangent to the pad surface) that the cable exerts on the grasp, that is, the pull that tries to slide the cable out of the pads while the hand moves through the gap.
- $F_n$ [N] is the normal force with which each pad presses on the cable. It is set by the torque limit of the gripper motor (`--cable-grip-effort`): the larger the torque, the harder the pressing.
- $\mu$ (dimensionless) is the friction coefficient between the pads and the pinched cable section (in the code it is set on the cable proxy body: 0.7 in V1, 1.2 in V2 and V3).
- The factor $2$ is there because there are two pads, each giving a maximum friction force of $\mu F_n$.
- The right-hand side $2\mu F_n$ [N] is the largest friction force that keeps the cable from sliding. If $F_t$ exceeds it, the cable slides (slips out).

The two changes (grip effort and friction) raise both factors $F_n$ and $\mu$ of the right-hand side at the same time.

**Fix.**

- Cable grip effort 0.9 to 2.2 N m (`--cable-grip-effort`), proxy friction 0.7 to 1.2.
- Added monitoring: measure the distance between the proxy body and the TCP; if it is above 3 cm the cable is considered slipped and `--test` fails.

**Result.** The cable no longer slips in V2 and V3. Maximum offset between the cable and the grasp centre: V2 17.9 mm, V3 6.5 mm (both below the 3 cm threshold of `hold_check`).

**Limits of this analysis.** The two changes (grip effort and friction) were applied together, so how much each one contributes is not known. The cable force on the plug that was measured is only about 0.4 to 3.5 N, well below the estimated friction capacity of the two pads, so the static force balance above does not fully explain why the cable slipped in V1. Separating the effects would need runs with each change on its own.

---

## 2. V2: the wrist turns at gap 2-3 and displaces the plug

[https://github.com/user-attachments/assets/ce6821e9-a67b-48cd-8ddf-77620a30ea3a](https://github.com/user-attachments/assets/ce6821e9-a67b-48cd-8ddf-77620a30ea3a)

**Symptom.** V2 has none of the V1 faults and runs the whole sequence, the plug stays inserted. But the plug is displaced 3.05 mm entering gap 2-3 and 3.88 mm after the cable is laid, with a final housing-to-face gap of 3.74 mm. The old test (which demands a 0.2 mm pin error) fails.

**Cause.** The wave waypoints are taken from the transport path used for the insertion, where the hand has a yaw $\psi = -90^\circ$ at gap 1-2 and $+60^\circ$ at gap 2-3. The hand orientation function is:

$$
R_{tool}(\psi,\theta) = R_z(\psi)\,R_y(\theta)\,R_0 ,\qquad \theta = 40^\circ\ (\text{tilt})
$$

Where:
- $R_{tool}$ is the 3x3 rotation matrix from the wrist 3 frame to the world frame; it tells how the hand is oriented.
- $\psi$ (yaw) is the angle by which the whole hand is turned about the vertical world Z axis [rad]. $\psi = 0$ means the hand points straight toward +X.
- $\theta$ (tilt) is the angle by which the tool axis is pitched down from the +X direction, equal to $40^\circ$ (the constant `GRASP_TILT`).
- $R_z(\psi)$ and $R_y(\theta)$ are the elementary rotation matrices about the Z and Y axes.
- $R_0 = \begin{bmatrix}0&1&0\\-1&0&0\\0&0&1\end{bmatrix}$ is the base orientation of wrist 3 (the columns are the wrist x, y, z axes in the world frame) when there is no yaw and no tilt.

So $\psi$ is the quantity that was changed: V1 and V2 use $\psi=-90^\circ$ at gap 1-2 and $+60^\circ$ at gap 2-3, while V3 uses $\psi = 0$.

During the insertion the hand holds the plug and the cable still has slack, so turning the yaw is harmless. During the re-grasp it is different: the plug is already fixed in the socket and the hand grips the cable at arc 0.78 m out of 1.6 m. Turning the yaw makes wrist 6 rotate about 106 degrees on the leg into gap 2-3, which twists and pulls the cable, and that force reaches the plug. The cable has the following stiffness (E = 5·10⁶ Pa, radius 5 mm, 64 segments of length $\ell = 25$ mm):

$$
k_{stretch} = \frac{EA}{\ell} \approx 1.57\cdot 10^{4}\ \mathrm{N/m},\qquad
k_{bend} = \frac{EI}{\ell} = \frac{E\,\pi r^4/4}{\ell} \approx 0.098\ \mathrm{N\,m/rad}\ (\text{per joint})
$$

Where:
- $E = 5\cdot 10^{6}$ Pa is the Young's modulus of the cable (material stiffness).
- $r = 5$ mm is the cable radius; $A = \pi r^2 \approx 7.85\cdot 10^{-5}$ m² is the cross-section area, so $EA \approx 393$ N.
- $I = \pi r^4/4 \approx 4.91\cdot 10^{-10}$ m⁴ is the second moment of area of the circular cross-section, so $EI \approx 2.45\cdot 10^{-3}$ N m² (bending stiffness).
- $\ell = 1.6/64 = 25$ mm is the length of each cable segment (a 1.6 m cable split into 64 segments).
- $k_{stretch}$ [N/m] is the stretching stiffness of one segment: stretching it by 1 m would produce 15 700 N (linear estimate).
- $k_{bend}$ [N m/rad] is the bending stiffness of one joint between two segments: bending it by 1 rad produces a moment of 0.098 N m.

These two numbers match the values used by the VBD simulator (stated in the log): the cable is rather soft in bending and very stiff in stretching. They are material constants that give the reader a feeling for the cable stiffness; they are not a calculation that directly proves the plug displacement.

The causal evidence is fairly strong: between V2 and V3 only the wrist yaw (and the test criterion) changed, so the drop in plug displacement is due to removing the yaw. The specific mechanism (twisting and pulling the cable) is the explanation given in the log and was not isolated by a separate experiment.

**Fix.** Remove the wrist yaw while waving during the re-grasp. The wave waypoints use yaw 0 instead of the yaw of the transport path (the transport path used for the insertion keeps its yaw):

The gap crossings remain straight Cartesian segments at constant height and orientation:

$$
p(s) = p_0 + s\,(p_1 - p_0),\qquad R(s) = \mathrm{Slerp}(R_0, R_1; s),\qquad s\in[0,1]
$$

Where:
- $s$ is a dimensionless interpolation parameter going from 0 (start) to 1 (end); it is not the arc length of the cable.
- $p_0$ and $p_1$ are the TCP positions at the start and the end of the gap crossing [m]; $p(s)$ is the TCP position at $s$, a point on the straight segment joining them, so the height stays constant.
- $R_0$ and $R_1$ are the hand orientations at the start and the end; $R(s)$ is the orientation at $s$.
- $\mathrm{Slerp}$ (spherical interpolation) turns evenly from $R_0$ to $R_1$ following $R(s) = R_0\exp\!\big(s\,\log(R_0^{\top}R_1)\big)$, that is, a rotation about a fixed axis at constant angular speed. In V3 both ends have the same yaw, so the orientation stays unchanged along the segment.
- For every $s$ the code solves IK for the joint angles, starting from the solution of the previous $s$, so it does not jump between branches.

Measured result: at gap 1-2 the plug passes at x = 0.530 m (gap centre) and z = 0.264 to 0.267 m, at gap 2-3 at x = 0.797 m and z = 0.265 to 0.268 m, all below the post top at 0.30 m. cuRobo still finds collision-free plans when the hand does not turn.

**Result (V2 vs V3).**

| | V2 | V3 |
|---|---|---|
| Plug displacement entering gap 2-3 | 3.05 mm | **0.79 mm** |
| Plug displacement after laying the cable | 3.88 mm | **0.48 mm** (max 1.09 mm during re-grasp) |
| Max cable offset from the TCP | 17.9 mm | **6.5 mm** |
| Final housing-to-face gap | 3.74 mm | **0.84 mm** |

---

## 3. V3: the finished version

[https://github.com/user-attachments/assets/0e6b3166-2709-4389-bab7-b95fe0e6647e](https://github.com/user-attachments/assets/0e6b3166-2709-4389-bab7-b95fe0e6647e)

V3 is V2 plus two changes: the wrist yaw while waving is set to 0, and the test drops the 0.2 mm pin tolerance (the new plan `plans/UR5_regrasp.md` no longer prescribes it). The test checks only two things: the cable did not slip out of the gripper and the plug is still in the socket (housing-to-face gap smaller than half the blade length):

V3full result: 959 s wall clock, 72.2 s simulated. A repeat run (`_V3full2`) gave almost the same numbers: plug displacement 0.474 mm, gap 0.86 mm, max deviation 1.07 mm. Accepted video: `video_ur5_regrasp_V3full.mp4`.

V3 sequence: Home, approach, grasp the plug, lift, transport through the 3 posts, insert, release, retreat, re-grasp the cable (grasp at arc 0.78 m), wave through post 1, gap 1-2, post 2, gap 2-3, lay the cable aligned with the ground projection of the plug, return to Home.

---
## 4. Summation

| Item | V1 | V2 | V3 |
|---|---|---|---|
| Cable in the gripper through gap 2-3 | slips (pad gap closes to 12.5 mm), one run ended in NaN | does not drop, max offset 17.9 mm | does not drop, max offset 6.5 mm |
| Base joint rotation when approaching the cable | 286 deg | 23 deg | 23 deg |
| Total base rotation during re-grasp | 687 deg | 240 deg | 240 deg |
| Cable grip effort | 0.9 N m, proxy friction 0.7 | 2.2 N m, proxy friction 1.2 | 2.2 N m, proxy friction 1.2 |
| IK solution choice | cuRobo's own | `ik_candidates`, least-rotation solution | same as V2 |
| Wrist yaw while waving through the gaps | yes (-90 / +60 deg) | yes (-90 / +60 deg), wrist 6 turns about 106 deg into gap 2-3 | **0 deg** (`--wave-yaw-deg 0`) |
| Plug displacement entering gap 2-3 | 1.2 to 3.6 mm (unstable) | 3.05 mm | **0.79 mm** |
| Plug displacement after laying the cable | | 3.88 mm | **0.48 mm** (max 1.09 mm) |
| Final housing-to-face gap | | 3.74 mm | **0.84 mm** |
| Final pin lateral error | 0.64 mm | 0.66 mm | 0.37 mm |
| Test criterion | still demands 0.2 mm | still demands 0.2 mm (fails) | plug still in socket and cable not dropped (passes) |
| Cable dropped from the gripper | yes | no | no |

The V1 plug displacement comes from runs of the original build (1.2 to 3.6 mm between runs, one run 11.9 mm then NaN). Empty cells mean the log has no value.

---

## 5. What is still open in V3

1. The pin lateral error after release is still 0.37 mm, above the 0.2 mm of the old plan (tilt about 2.5 degrees, soft penalty contacts with ke = 1e5). It is simply no longer a criterion of the test.
2. The cable section in the gripper is kinematic (a single proxy body), so there is no cable force on the pads. Full two-way coupling exists only at the plug.
3. Each configuration was run once; the GPU is non-deterministic, so results differ by 0.01 to 0.1 mm (a few mm in older versions).
4. Collisions between the gripper and the posts when the hand does not turn were not checked by eye.
5. Lowering the path height to z = 0.20 m (so the cable looks as if it goes between the posts) failed at planning (`cuRobo failed to plan 'weave leg 3.0 (-> post2)'`); the cause has not been verified.
