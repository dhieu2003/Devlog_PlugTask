# STUDY – Tìm hiểu mã nguồn UR5 + Robotiq 2F-85 (ur5_screen V3)

Tổng hợp 5 câu hỏi về cách dựng robot, lập kế hoạch, hình học và cách tái lập mô phỏng. Nội dung suy ra từ việc đọc code (`data/scripts/`), `data/assets/ur5e_curobo/`, `Check_version.py` và `pip list` trong env `cable`; chưa chạy lại gì.

## Mục lục
1. [Đổi từ Franka sang UR5 + 2F-85 cần sửa gì](#1-đổi-từ-franka-sang-ur5--2f-85-cần-sửa-gì)
2. [Robot lấy từ spec.md hay từ thư viện?](#2-robot-lấy-từ-specmd-hay-từ-thư-viện)
3. [Lập kế hoạch và cách hoạt động của curobo_V3.py](#3-lập-kế-hoạch-và-ur5_screen_curobo_v3py)
4. [Vì sao cần geom.py / geom_V3.py](#4-vì-sao-cần-ur5_screen_geompy--geom_v3py)
5. [Chỉ số và phiên bản để tái lập](#5-chỉ-số-và-phiên-bản-để-tái-lập)
6. [Giải thích rõ hơn: geom.py và cách tính kế hoạch (có số liệu)](#6-giải-thích-rõ-hơn-geompy-và-cách-tính-kế-hoạch-có-số-liệu)
7. [cuRobo tìm đường như thế nào](#7-curobo-tìm-đường-như-thế-nào)
8. [cuRobo kiểm tra va chạm: nhiều cầu và các hộp](#8-curobo-kiểm-tra-va-chạm-nhiều-cầu-và-các-hộp)
9. [Link và khớp là gì](#9-link-và-khớp-là-gì)

---

## 1. Đổi từ Franka sang UR5 + 2F-85 cần sửa gì

So sánh `franka_arm_V1.py` với `UR5_2F85_V3.py` / `ur5_screen_V3.py`:

| Hạng mục | Franka | UR5 + 2F-85 |
|---|---|---|
| Nạp robot | `add_urdf(download_asset("franka_emika_panda")/...fr3_franka_hand.urdf)` | `add_mjcf` UR5e `ur5e.xml` (xoay π quanh Z), rồi `add_mjcf` `2f85.xml` gắn vào `wrist_3_link` (`parent_body=ee_body`); cả hai từ MuJoCo Menagerie |
| Bậc tự do | 7 khớp tay + 2 ngón (`[:7]`, `[7:9]`) | 6 khớp tay (`[:6]`) + 2 khớp `right/left_driver_joint` |
| Gain | ke 400, kd 80 (theo IsaacLab) | ke 4500, kd 450, armature 0,2; tay kẹp ke 20, kd 1 |
| Điều khiển tay kẹp | Ngón tịnh tiến 0,04 → 0 m, lực 1500 N | Khớp truyền động xoay 0 (mở 85 mm) → 0,8 rad (đóng); lực đặt qua `joint_effort_limit` (`--grip-effort`, mặc định 1,5 N·m) |
| IK | IK GPU của Newton | FK/IK numpy tự viết trong geom + cuRobo |
| Hệ tọa độ | Bề mặt làm việc | Gốc ở chân đế, Z lên (theo `spec.md`) |
| Tay kẹp | – | `ctrl_source` chỉ cho 6+1 DoF đầu; tìm body `right_pad/left_pad` để đặt μ = 0,7 |

Phải tính lại các hằng số `GRASP_TILT`, `TCP_OFFSET_WRIST`, `TCP_IN_PLUG` theo hình học tay kẹp mới.

---

## 2. Robot lấy từ spec.md hay từ thư viện?

**Mô hình robot lấy từ thư viện ngoài, không dựng từ spec.**
- `_menagerie_path("universal_robots_ur5e", "ur5e.xml")` và `("robotiq_2f85", "2f85.xml")`: tải từ MuJoCo Menagerie, ghim commit `da76818e269b82289eba39808e2fb91d679d6994` (`MENAGERIE_REF`).
- Không thấy robot nào lấy từ thư viện asset của Isaac Sim; Isaac Sim chỉ dùng để dựng video.

| | Nội dung |
|---|---|
| **Có dùng từ spec** | Kích thước phích và ổ (mục 3); μ = 0,7 cho má kẹp (comment ghi "spec 2.4"); tốc độ mang phích 0,25 m/s; hệ tọa độ; cột D 5 cm × H 30 cm |
| **Lệch spec** | Menagerie là **UR5e**, không phải UR5 (tầm với 850 mm như nhau, nhưng elbow ±π so với ±6,28 rad trong spec); `joint_effort_limit` 150 N·m cho cả 6 khớp (URDF cổ tay là 28); không thấy kiểm payload 4,075 kg hay khối lượng 18,4 kg (phích chỉ 0,1 kg); lực kẹp đặt gián tiếp bằng mô-men 1,5 N·m, không bằng "15–30 N" (devlog V0 đo ~26–28 N) |

---

## 3. Lập kế hoạch và `ur5_screen_curobo_V3.py`

**Ý tưởng**: chia nhiệm vụ thành các đoạn (segment), mỗi đoạn là một mảng góc khớp theo thời gian:

`Home → Approach → Descend → Lift → weave (luồn cột) → Pre-insert → Insert → Retreat → Return`

**Tiến trình riêng**: cuRobo cần numpy ≥ 2, còn Newton/Isaac Sim trong env `cable` dùng numpy 1.26. `ur5_screen_V3.py` chạy file này với bản numpy 2 cô lập ở `~/.cache/cable_task/np2`.

**Các bước trong file**
1. `write_robot_files()` tạo `ur5e.urdf` và `ur5e_screen.yml`: thêm `plug_link` cố định vào cổ tay cùng các cầu va chạm bao phích, để cuRobo coi phích đang cầm là một phần của robot. FK của URDF sao từ Menagerie nên khớp mô hình Newton.
2. Dựng bộ lập kế hoạch: `MotionPlannerCfg.create(...)` với cảnh `G.curobo_scene()` (sàn, tường, 3 cột bằng hộp bao), sai số 1 mm / 0,02, 32 seed, `interpolation_dt` 0,02 s.
3. Hai loại đoạn:
   - *Chuyển động tự do* (Approach, weave, Return): cuRobo `plan_pose` / `plan_cspace`. Với weave, `ik_near` thử 40 hạt giống IK, chọn nghiệm gần cấu hình trước nhất (giữ cùng nhánh tay), rồi cuRobo lập đường trong không gian khớp, tránh va chạm.
   - *Đoạn thẳng ngắn theo trục tay kẹp* (Descend, Lift, Insert, Retreat): `line_segment`, IK giảm chấn nối tiếp, hướng không đổi.
4. Kiểm tra: `G.clearance()` in khoảng cách nhỏ nhất giữa tay kẹp (+phích) và cột, tường, sàn.
5. Xuất `.npz` (`q_<tên đoạn>`) để `ur5_screen_V3.py` phát lại.

**Điểm yếu** (đã ghi ở devlog): cuRobo chỉ kiểm va chạm cánh tay trên các đoạn do nó lập; các đoạn thẳng thì không.

---

## 4. Vì sao cần `ur5_screen_geom.py` / `ur5_screen_geom_V3.py`

Đây là module **numpy thuần, không import warp / newton / torch**, nên ba chương trình dùng chung được mà không xung đột môi trường:
- mô phỏng Newton (`ur5_screen_V3.py`),
- bộ lập kế hoạch cuRobo (tiến trình riêng, numpy 2),
- dựng video Isaac Sim.

Lý do chính: bố cục (cột, tường, ổ, phích) và động học chỉ được định nghĩa **một nơi**, tránh lệch giữa ba nơi dùng.

Nội dung module:
- **Bố cục nhiệm vụ**: `POSTS`, `WALL_XF`, `WEAVE_OFFSET`, `WEAVE_Z`, vị trí phích ban đầu và khi cắm hết cỡ.
- **Hình học phích và ổ** (theo spec), `socket_boxes()` dựng ổ bằng các hộp.
- **FK/IK UR5e**: `fk`, `ik` (DLS warm-start) cho đoạn thẳng và `ik_near`.
- **Quỹ đạo khung**: `weave_waypoints`, `tool_rotation`, `tcp_for_plug`.
- **Kiểm va chạm**: `clearance`, `plug_spheres`, `end_effector_spheres`, `curobo_scene()`.

**Về hậu tố phiên bản**: các file `_V1`, `_V2`, `_V3` và bản gốc gần như giống nhau (`geom_V2` và `geom_V3` chỉ khác docstring). Mỗi phiên bản có bản sao riêng để giữ lịch sử, nên khi sửa hằng số phải sửa đúng bản đang dùng.

---

## 5. Chỉ số và phiên bản để tái lập

### Phần mềm (Python 3.12, env `cable`)
| Thư viện | Phiên bản |
|---|---|
| Isaac Sim | 6.1.0.0 |
| Newton | 1.7.0 |
| warp-lang | 1.18.0 |
| mujoco / mujoco-warp | 3.15.0 / 3.15.0 |
| nvidia-curobo | 0.8.0.post1.dev43 |
| torch | 2.11.0 |
| numpy / scipy (env chính) | 1.26.4 / 1.12.0 |
| numpy / scipy (cho cuRobo, `np2`) | 2.2.6 / 1.14.1 (theo devlog) |
| usd-core | 26.8 |
| Menagerie (UR5e, 2F-85) | commit `da76818e269b82289eba39808e2fb91d679d6994` |

`Check_version.py` ghi nhãn Newton "1.7.0.dev0", nhưng `pip` báo 1.7.0; lấy số của `pip`.

### Tham số mô phỏng (mặc định trong `ur5_screen_V3.py`)
| Nhóm | Giá trị |
|---|---|
| Bước thời gian | 60 Hz; 16 substep MuJoCo/khung; 10 substep dây |
| `SolverMuJoCo` | solver `newton`, integrator `implicitfast`, cone `elliptic`, 100 iter, 30 line-search iter, `impratio` 10 |
| Khớp tay | ke 4500, kd 450, armature 0,2, effort 150 |
| Tay kẹp | ke 20, kd 1, đóng 0,8 rad, effort 1,5 N·m, nghiêng 40° |
| Tiếp xúc | `CONTACT_KE` 1e5; μ má kẹp 0,7; μ sàn 0,6; dây: `--cable-contact-ke` 1e5, `-kd` 10 |
| Dây (VBD) | dài 1,2 m, 48 đoạn, bán kính 5 mm, stretch 1e6, bend 5e-3, damping 2e-3, 20 iter, 4 đoạn đầu gắn cứng vào phích |
| Chuyển động | v_free 0,5; v_carry 0,25 m/s; a_carry 0,8 m/s²; v_insert 0,02 m/s |
| cuRobo | sai số 1 mm / 0,02; 32 seed; `dt` 0,02 s; `max_acceleration` 6; `max_jerk` 500; `collision_sphere_buffer` 0,002; `CU_ACT` 0,01 |
| Cảnh | phích 0,1 kg; cột D 5 cm, H 30 cm tại x = 0,50 / 0,70 / 0,90 m; ổ tại x = 1,04 m |

### Biến môi trường ảnh hưởng kết quả
`SCREEN_TILT`, `SCREEN_CONTACT_KE`, `SCREEN_STIFF=1` (contact cứng, sẽ làm phích kẹt), `SCREEN_SOLREF`, `CU_ACT`.

### Tính tất định
- `ik_near` cố định seed `default_rng(0)`, nên phần IK lặp lại được.
- Quỹ đạo cuRobo được lưu ở `data/results/ur5_screen_V3_plan.npz`. Để tái lập đúng video, dùng file này, **không** dùng `--replan`.
- Chưa kiểm cuRobo có cho cùng kết quả khi lập lại trên máy khác; cũng không thấy chỗ nào đặt seed cho torch.

### Lệnh chạy lại
```
cd data/scripts
python ur5_screen_V3.py --test
python ur5_screen_render_isaac_V3.py     # data/results/video_ur5_screen_V3.mp4
```

---

## 6. Giải thích rõ hơn: geom.py và cách tính kế hoạch (có số liệu)

Các số dưới đây lấy từ việc chạy thử các hàm trong `ur5_screen_geom_V3.py`.

### Vì sao cần `geom.py`
**Vấn đề 1 – hai "ngôn ngữ" khác nhau.** Nhiệm vụ được mô tả bằng *vị trí trong không gian* ("phích ở (0,22; 0,26; 0,015) m, ổ ở x = 1,04 m"), còn Newton và cuRobo chỉ nhận *6 góc khớp*. Cần lớp chuyển đổi hai chiều:
- **FK** (góc khớp → vị trí tay): `fk(q)`.
- **IK** (vị trí tay → góc khớp): `ik(p, R, q0)`.

Ví dụ: `fk(HOME_Q)` cho TCP tại (0,492; 0,134; 0,346) m, khớp TCP home trong devlog. Để gắp phích, TCP phải ở (0,216; 0,26; 0,023); `ik` tìm ra `q = [-1,152; -2,153; 3,933; -3,802; -0,796; -0,965]` rad, sai số 1,1e-6 m, và `fk(q)` cho lại đúng điểm đó.

**Vấn đề 2 – ba chương trình ở ba môi trường khác nhau** (mô phỏng numpy 1.26, cuRobo numpy ≥ 2 ở tiến trình riêng, video Isaac Sim) không import lẫn nhau được. Mọi con số của bài toán nằm trong một file numpy thuần; cả ba `import ur5_screen_geom_V3 as G`. Nếu mỗi nơi tự định nghĩa, chỉ cần lệch một hằng số là cuRobo lập đường qua chỗ mô phỏng có cột.

**Vấn đề 3 – cuRobo không biết hình học của nhiệm vụ.** `curobo_scene()` tạo hộp cho sàn, tường, 3 cột; `plug_spheres()` tạo các cầu bao phích; `clearance()` kiểm tra lại độc lập sau khi lập kế hoạch.

### Các bước tính kế hoạch
1. **Từ vị trí phích suy ra vị trí tay kẹp.** `plug_resting_pose() = (0,22; 0,26; 0,015)`, cộng `TCP_IN_PLUG = (-0,004; 0; 0,008)` → TCP đích `(0,216; 0,26; 0,023)`.
2. **Chọn hướng tay kẹp.** `tool_rotation()` chúc trục tay kẹp xuống 40° so với trục X; vector hướng là (0,766; 0; −0,643). Dùng nó để lùi 8 cm ra điểm pre-grasp.
3. **IK** (`ik()`, damped least squares): lặp từ cấu hình ban đầu, tính sai lệch tư thế rồi chỉnh các khớp để giảm sai lệch; lấy cấu hình trước làm điểm xuất phát nên nghiệm luôn gần cấu hình trước.
4. **Các đoạn thẳng** (Descend, Lift, Insert, Retreat): `line_segment()` chia đoạn thẳng thành 25–40 điểm và giải IK cho từng điểm, hướng giữ nguyên.
5. **Các mốc luồn cột** từ `weave_waypoints()` (tọa độ phích, góc xoay tay quanh Z):

| Mốc | Phích (x, y, z) m | Xoay tay |
|---|---|---|
| post1 | (0,50; +0,11; 0,27) | 0° |
| gap12_in → out | (0,60; +0,11 → −0,11; 0,27) | −90° |
| post2 | (0,70; −0,11; 0,27) | 0° |
| gap23_in → out | (0,80; −0,11 → +0,11; 0,27) | 60° |
| post3 | (0,90; +0,11; 0,27) | 0° |

   Phích đi qua cột 1 ở phía +Y, qua khe 1–2 để sang −Y, qua cột 2 phía −Y, qua khe 2–3 để về +Y. Độ cao 0,27 m nằm dưới đỉnh cột 0,30 m; mỗi lần qua khe tay xoay để phích dọc hướng đi. Khe chỉ rộng 0,20 − 0,05 = 0,15 m, hẹp hơn bề ngang bàn kẹp nếu để ngang.
6. **Nối các mốc bằng cuRobo.** Với mỗi mốc: (a) `ik_near()` chạy IK từ 40 hạt giống quanh cấu hình trước, chọn nghiệm gần nhất và trong giới hạn khớp – giữ cánh tay trên cùng một nhánh, tránh lật khuỷu; (b) `plan_cspace(q_prev, q_mốc)` để cuRobo lập đường tránh va chạm.
7. **Kiểm tra và xuất.** `clearance()` tính khoảng cách nhỏ nhất của tay kẹp (+phích) với cột, tường, sàn, rồi lưu `.npz`. Mô phỏng phát lại, tốc độ giới hạn bởi `--v-carry 0,25` m/s khi mang phích (phần thời gian hóa nằm trong `ur5_screen_V3.py`, chưa đọc kỹ).

Lưu ý: `clearance` tại tư thế gắp là −1,7 mm (gần chạm sàn) – hợp lý vì phích chỉ cao 3 cm và má kẹp xuống sát sàn.

---

## 7. cuRobo tìm đường như thế nào

Đọc từ `plan_cspace` trong `curobo/_src/motion/motion_planner.py`.

**Đầu vào cuRobo cần** (với `plan_cspace` chỉ cần xuất phát và đích dạng góc khớp):

| Cần | Ai chuẩn bị | Nội dung |
|---|---|---|
| Robot | `write_robot_files()` + `ur5e.urdf/.yml` | Chuỗi khớp, giới hạn khớp, cầu va chạm từng link, `plug_link` |
| Thế giới | `G.curobo_scene()` | Hộp sàn, tường, 3 cột |
| Xuất phát | `q_prev` | 6 góc khớp hiện tại |
| Đích | `ik_near(...)` dùng `G.ik` | 6 góc khớp suy ra từ mốc trong `weave_waypoints()` |

**cuRobo làm gì**
- **a) Tối ưu hóa quỹ đạo (chính).** Coi cả đường đi là chuỗi cấu hình khớp theo thời gian, tối ưu để nối xuất phát với đích, đủ mượt và tránh va chạm. Chạy song song 32 hạt giống (`num_trajopt_seeds=32`) trên GPU, giữ đường tốt nhất, rồi tinh chỉnh tối đa 3 lần với bước thời gian nhỏ lại 0,75 lần.
- **b) Kiểm tra va chạm bằng cầu và hộp** (xem mục 8). Vùng bắt đầu tính chi phí va chạm là `optimizer_collision_activation_distance` = 0,01 m (biến `CU_ACT`). Khi mang phích, `enable_link_collision(["plug_link"])` bật thêm cầu bao phích.
- **c) Dự phòng bằng graph planner.** Nếu lần đầu thất bại, từ lần thứ hai (`enable_graph_attempt=1`) cuRobo gọi `graph_planner.find_path` lấy các đường gợi ý làm hạt giống cho bước (a). Tối đa `max_attempts=40`; hết mà không thành công thì code ném `RuntimeError("cuRobo failed to plan ...")`.
- **d) Nội suy** `get_interpolated_plan()` lấy mẫu mỗi 0,02 s (`interpolation_dt`) – đó là các hàng `q_weave*` trong `.npz`.

**Ví dụ một đoạn weave** (từ `post1` đến `gap12_in`): `tcp_for_plug` + `tool_rotation(yaw)` cho TCP đích → `ik_near` cho `q_goal` → `plan_cspace(q_prev, q_goal)` chấm 32 đường theo khoảng cách cầu–hộp (cột 1, cột 2, tường, sàn, tự va chạm, phích) → trả chuỗi `q` tránh cột → `clearance(q, carrying=True)` kiểm tra lại độc lập (lớp kiểm thứ hai, vì từng có lỗi cuRobo bỏ qua cột trụ – devlog V0).

**Hạn chế**
- cuRobo chỉ "thấy" các cầu và hộp bao; cột là hộp vuông 5 × 5 cm nên bảo thủ hơn cột trụ thật.
- Các đoạn thẳng (Descend, Lift, Insert, Retreat) không qua cuRobo nên không được kiểm va chạm cánh tay.
- Chưa đọc chi tiết hàm chi phí và siêu tham số tối ưu bên trong cuRobo; mục (b) là cách hoạt động chung.

---

## 8. cuRobo kiểm tra va chạm: nhiều cầu và các hộp

Hiểu "robot là một khối cầu, cột là khối hộp" là **gần đúng nhưng chưa đủ**: robot được phủ bằng **nhiều quả cầu**.

- Cột (cũng như sàn, tường) là khối hộp trong `curobo_scene()`; cột là hộp 5 × 5 × 30 cm tại tọa độ cột.
- Mỗi link của cánh tay được phủ nhiều quả cầu xếp dọc theo link (`ur5e_screen.yml`):
  - `upper_arm_link`: 5 cầu, bán kính 0,058 m, cách nhau 0,1 m.
  - `forearm_link`: 5 cầu, bán kính 0,05 m.
  - `wrist_3_link` (cổ tay + tay kẹp): 10 cầu, bán kính 0,016–0,052 m.
  - `plug_link` (phích đang cầm): 5 cầu, bán kính 0,011–0,0165 m.
- Mỗi tư thế khớp cho FK ra vị trí từng quả cầu; cuRobo tính khoảng cách của từng cầu tới từng hộp và lấy giá trị nhỏ nhất. Khoảng cách nhỏ hơn bán kính cầu cộng khoảng đệm thì coi là va chạm hoặc sắp va chạm. Như vậy cả cánh tay được kiểm tra, không chỉ một điểm.
- Các link kề nhau không bị tính là tự va chạm nhờ `self_collision_ignore` trong file yml.
- Trong Newton, cột là **hình trụ** bán kính 2,5 cm; hộp bao 5 × 5 cm lớn hơn trụ thật, nên cuRobo hơi bảo thủ và có thể từ chối đường thực ra đi qua được.
- **Hệ quả**: phần nào không được phủ cầu thì không bị kiểm tra. Đó là lý do ở V2 (lỗi 3 trong devlog) cầu cổ tay chạm đỉnh cột phải sửa độ cao phích và góc nghiêng tay kẹp.

(Suy ra từ file cấu hình và code; không kiểm từng dòng trong lõi tính khoảng cách của cuRobo.)

---

## 9. Link và khớp là gì

**Link** là một đoạn cứng (một khúc xương) của cánh tay; **khớp** (joint) là chỗ nối hai link kề nhau, cho phép xoay tương đối.

UR5e có 6 khớp xoay nên có 7 link nối tiếp (theo `ur5e.urdf`):

| Link | Là phần nào | Khớp nối vào nó |
|---|---|---|
| `base_link` | Chân đế, gắn cố định xuống đất | – |
| `shoulder_link` | Khối vai | `shoulder_pan_joint` (xoay quanh trục dọc) |
| `upper_arm_link` | Cánh tay trên (đoạn dài 0,425 m) | `shoulder_lift_joint` |
| `forearm_link` | Cẳng tay (đoạn dài 0,392 m) | `elbow_joint` |
| `wrist_1_link` | Cổ tay 1 | `wrist_1_joint` |
| `wrist_2_link` | Cổ tay 2 | `wrist_2_joint` |
| `wrist_3_link` | Cổ tay 3, gắn tay kẹp Robotiq | `wrist_3_joint` |

Trong URDF còn các link phụ không có thân thật: `tool0`, `tcp` (điểm giữa hai má kẹp, cách `wrist_3_link` 0,2415 m theo trục Y) và `plug_link` (phích đang cầm, thêm bằng code).

Liên hệ với mục 8: cuRobo phủ các quả cầu lên **từng link** (`collision_link_names`). Khi khớp quay, các cầu gắn trên link đó chuyển động theo. Hình dung: **khớp** là chỗ xoay, **link** là thanh cứng giữa hai chỗ xoay, còn **các cầu va chạm** là lớp "áo giáp" xấp xỉ bao quanh mỗi thanh.
