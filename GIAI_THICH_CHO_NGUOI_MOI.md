# GIẢI THÍCH CHO NGƯỜI MỚI: Robot UR5 + Robotiq 2F-85 cắm phích điện qua 3 cột (V3)

Tài liệu này dành cho người mới học **Python** và **sim2real**. Nó giải thích **ý tưởng**, **cách code được tổ chức** và **các thuật toán** của chương trình `ur5_screen_V3` trong thư mục `data/scripts/`.

Nội dung được viết sau khi đọc trực tiếp bốn file: `ur5_screen_V3.py`, `ur5_screen_geom_V3.py`, `ur5_screen_curobo_V3.py`, `ur5_screen_render_isaac_V3.py`, cùng `plans/UR5_Screen.md`, `instructions/spec.md`, `guide.md` và các devlog. Tôi **chưa chạy lại** chương trình. Các con số kết quả (sai số, thời gian...) lấy từ devlog, không phải do tôi đo lại. Chỗ nào là suy luận của tôi, tôi ghi rõ.

## Mục lục
1. [Bài toán là gì](#1-bài-toán-là-gì)
2. [Từ khóa nền tảng](#2-từ-khóa-nền-tảng)
3. [Bức tranh tổng thể: 4 chương trình phối hợp](#3-bức-tranh-tổng-thể-4-chương-trình-phối-hợp)
4. [Ý tưởng thiết kế và lý do](#4-ý-tưởng-thiết-kế-và-lý-do)
5. [Đọc code từng file](#5-đọc-code-từng-file)
6. [Thuật toán chi tiết](#6-thuật-toán-chi-tiết)
7. [Một lần chạy diễn ra như thế nào (theo thời gian)](#7-một-lần-chạy-diễn-ra-như-thế-nào-theo-thời-gian)
8. [Liên hệ với sim2real](#8-liên-hệ-với-sim2real)
9. [Kết quả và hạn chế](#9-kết-quả-và-hạn-chế)
10. [Python cho người mới: các kiểu viết trong code](#10-python-cho-người-mới-các-kiểu-viết-trong-code)
11. [Gợi ý học tiếp và bài tập](#11-gợi-ý-học-tiếp-và-bài-tập)

---

## 1. Bài toán là gì

Robot UR5 gắn tay kẹp Robotiq 2F-85 phải:

1. Gắp **đầu phích** (NEMA 5-15P, loại 2 chân dẹt + 1 chân tròn) của một sợi dây cáp đang nằm trên sàn.
2. Nhấc lên, **luồn dây qua 3 cột nhôm** (so le: cột 1 phía +Y, cột 2 phía −Y, cột 3 phía +Y) mà dây **không quấn** vào cột, robot và phích không đụng cột.
3. **Cắm phích vào ổ** gắn trên tường (tường vuông góc với sàn), nhả tay, ổ phải giữ được phích.
4. Về lại vị trí Home.

Sai số yêu cầu rất nhỏ: khe giữa chân phích và lỗ ổ chỉ khoảng **0,07–0,075 mm mỗi bên**. Ví dụ khe dẹt rộng 6,5 mm còn lưỡi phích rộng 6,35 mm, nên (6,5 − 6,35) / 2 = 0,075 mm.

Hệ tọa độ (theo `spec.md`): gốc ở chân đế robot, **Z hướng lên**, **X** hướng về phía tường (sang phải), **Y** vuông góc còn lại, trọng lực hướng −Z (g = 9,81 m/s²). Robot, 3 cột và tường nằm thẳng hàng dọc trục X:

```
 nhìn từ trên xuống (trục X sang phải, Y lên trên)

 y=+0.11        ● phích đi qua cột 1 bên +Y        ● cột 3 bên +Y ──► ổ cắm (y=+0.11, z=0.27)
 y= 0    [Robot]----(cột1)--------(cột2)--------(cột3)----|| tường x=1.04
 y=-0.11                         ● cột 2 bên −Y
        x=0        x=0.50       x=0.70       x=0.90
```

Phích nằm sẵn trên sàn ở Pose A = (0,22; 0,26) m, cạnh robot.

---

## 2. Từ khóa nền tảng

| Thuật ngữ | Nghĩa dễ hiểu |
|---|---|
| **Mô phỏng (simulation)** | Chương trình tính "nếu đẩy vật thế này thì vật chuyển động thế nào" bằng các phương trình vật lý. |
| **Sim2real** | Làm cho điều đã chạy được trong mô phỏng cũng chạy được trên robot thật. Khó vì mô phỏng luôn khác thực tế (ma sát, độ mềm, nhiễu cảm biến, độ trễ...). Xem mục 8. |
| **Khớp (joint)** và **link** | Khớp là chỗ xoay. Link là thanh cứng nối giữa hai khớp. UR5 có 6 khớp xoay. |
| **Cấu hình khớp q** | Mảng 6 số: góc của 6 khớp (đơn vị radian). Biết `q` là biết robot đang đứng dáng nào. |
| **TCP (Tool Center Point)** | Điểm "đầu ngón tay", ở đây là điểm giữa hai má kẹp. |
| **FK (Forward Kinematics, động học thuận)** | Từ 6 góc khớp tính ra vị trí và hướng của TCP. Luôn có đúng một đáp số. |
| **IK (Inverse Kinematics, động học ngược)** | Từ vị trí và hướng TCP mong muốn tìm 6 góc khớp. Có thể có nhiều đáp số hoặc không có. |
| **Motion planning (lập kế hoạch chuyển động)** | Tìm một đường đi từ cấu hình A đến B mà không va chạm. |
| **Quỹ đạo (trajectory)** | Đường đi cộng với thời gian: tại thời điểm t thì `q` bằng bao nhiêu. |
| **PD controller** | Bộ điều khiển kéo khớp về góc mục tiêu như lò xo (`ke`, độ cứng) có giảm chấn (`kd`). Code gán *mục tiêu* cho khớp, không đặt thẳng góc. |
| **Contact (tiếp xúc)** | Lực khi hai vật chạm nhau. Mô phỏng thường cho phép xuyên rất nhẹ rồi đẩy ra như lò xo (kiểu "penalty"); độ cứng càng lớn thì xuyên càng ít. |
| **VBD (Vertex Block Descent)** | Thuật toán mô phỏng vật mềm/dây của Newton, dùng cho sợi dây cáp. |

Các thư viện dùng:

| Thư viện | Vai trò trong dự án |
|---|---|
| **Newton** (xây trên **warp**) | Bộ mô phỏng vật lý chính. Chứa `SolverMuJoCo` (robot, phích, tiếp xúc) và `SolverVBD` (dây). |
| **MuJoCo Menagerie** | Kho mô hình robot có sẵn. UR5e và Robotiq 2F-85 lấy từ đây, **không dựng từ spec**. |
| **cuRobo** | Lập kế hoạch chuyển động tránh va chạm, chạy trên GPU. |
| **Isaac Sim** | Chỉ dùng để *dựng video* đẹp (render), không tính vật lý. |
| **numpy / scipy** | Tính toán mảng, ma trận xoay, nội suy. |

---

## 3. Bức tranh tổng thể: 4 chương trình phối hợp

```
                    ┌──────────────────────────────┐
                    │ ur5_screen_geom_V3.py        │  "Sách luật" dùng chung
                    │ hằng số, hình học, FK, IK,   │  (chỉ numpy, không import
                    │ cầu va chạm, điểm luồn cột   │   warp/newton/torch)
                    └──────┬────────────┬──────────┘
          import G         │            │ import G           import G
        ┌──────────────────┘            └────────────┐              │
        ▼                                            ▼              ▼
┌───────────────────────┐  chạy tiến trình con  ┌──────────────────┐ ┌──────────────────────────┐
│ ur5_screen_V3.py      │ ────────────────────► │ ur5_screen_      │ │ ur5_screen_render_       │
│ (CHƯƠNG TRÌNH CHÍNH)  │                       │ curobo_V3.py     │ │ isaac_V3.py              │
│ mô phỏng Newton       │ ◄──────────────────── │ lập kế hoạch     │ │ dựng video Isaac Sim     │
└──────────┬────────────┘  ur5_screen_V3_plan.npz└──────────────────┘ └────────────▲─────────────┘
           │ lưu                                                                     │ đọc
           └──► ur5_screen_V3_traj.npz + ur5_screen_V3_shapes.pkl ───────────────────┘
                                                                      └──► video_ur5_screen_V3.mp4
```

Thứ tự thực tế khi bạn gõ `python ur5_screen_V3.py --test`:

1. `launch_planner` chạy `ur5_screen_curobo_V3.py` như một **tiến trình con** để lập kế hoạch và lưu `plan.npz`. Nếu `plan.npz` đã có và mới hơn code thì bỏ qua bước này.
2. Chương trình chính nạp `plan.npz`, dựng thế giới vật lý, phát lại kế hoạch trong mô phỏng Newton và lưu `traj.npz`.
3. Bạn chạy riêng `ur5_screen_render_isaac_V3.py` để đọc `traj.npz` và quay video.

**Vì sao tách thành nhiều chương trình?** Do xung đột thư viện: cuRobo cần numpy ≥ 2, còn Newton và Isaac Sim trong môi trường `cable` cần numpy 1.26. Cách giải quyết là một bản numpy 2 cô lập ở `~/.cache/cable_task/np2`, chỉ tiến trình con của cuRobo thấy (qua biến môi trường `PYTHONPATH`). Đây là kỹ thuật thực tế khi hai thư viện đòi hai phiên bản khác nhau.

**Vì sao có file `geom`?** Ba chương trình cùng cần biết *cột ở đâu, ổ ở đâu, phích rộng bao nhiêu, FK của UR5e*. Nếu mỗi nơi tự ghi một bản, chỉ cần lệch một con số là cuRobo lập đường đi xuyên qua chỗ mô phỏng có cột. Vì thế mọi con số nằm đúng **một** chỗ (`import ur5_screen_geom_V3 as G`). Nguyên tắc này gọi là *single source of truth* (một nguồn sự thật).

---

## 4. Ý tưởng thiết kế và lý do

### 4.1. "Lập kế hoạch" tách khỏi "mô phỏng"

Chương trình làm theo hai pha:

- **Pha 1: nghĩ.** cuRobo và IK tính trước *toàn bộ* chuỗi góc khớp của robot (mảng `q` theo thời gian) cho từng đoạn.
- **Pha 2: làm.** Mô phỏng Newton đọc từng hàng `q`, đặt làm mục tiêu cho bộ PD của khớp, rồi để vật lý quyết định chuyện gì xảy ra (phích có bị tuột không, dây có vướng không...).

Đây là điều khiển **vòng hở** (open loop): kế hoạch không được sửa theo những gì xảy ra sau đó. Ngoại lệ duy nhất là bước "tri giác trong tay" (mục 6.8), nơi chương trình đo lại phích đang nằm thế nào trong tay rồi tính lại đoạn cắm.

### 4.2. Chia nhiệm vụ thành các đoạn (segment)

```
Home → Approach → Descend → Grasp(đóng kẹp) → Lift → Weave (nhiều chặng) → Pre-insert
     → Correct (chỉnh theo tri giác) → Insert → Release(nhả) → Retreat → Return → Home
```

Mỗi đoạn là một mảng `q` kèm giá trị đóng/mở tay kẹp. Có hai loại đoạn, chọn theo nhu cầu:

| Loại | Dùng cho | Công cụ | Lý do |
|---|---|---|---|
| **Di chuyển tự do** | Approach, Weave, Return | cuRobo | Cần tự tìm đường vòng tránh cột, tường, chính robot. |
| **Đoạn thẳng ngắn** | Descend, Lift, Insert, Retreat | IK nối tiếp | Cần đi *thẳng tắp*, giữ nguyên hướng tay (ví dụ cắm phích phải thẳng trục). cuRobo tối ưu đường cong nên không đảm bảo đi thẳng. |

### 4.3. Phích là vật rắn, dây bám theo phích

Dây rất mềm nên mô phỏng tốn kém. Dự án làm **hai mô hình riêng**:

- **Model A (MuJoCo):** robot + tay kẹp + phích + sàn + cột + tường/ổ. Phích là một vật rắn tự do nặng 0,1 kg.
- **Model B (VBD):** chỉ có dây, cùng sàn, cột và tường để dây va chạm. 4 đoạn dây đầu tiên (`anchor_segments=4`) bị **ghim cứng vào phích**: mỗi bước, vị trí phích lấy từ Model A, rồi đặt 4 đoạn đầu của dây đúng theo đó. Phần còn lại của dây tự rơi và trượt theo vật lý.

Hệ quả quan trọng: **tác động một chiều**. Phích kéo dây, nhưng dây *không kéo ngược* phích. Trong thực tế dây nặng và căng sẽ cản phích, còn ở đây thì không. Đây là một giản lược, được ghi ở mục hạn chế.

### 4.4. Tư thế gắp

Phích đặt **đứng** trên mặt nhỏ, hai mặt lớn của vỏ hướng ±Y. Tay kẹp nghiêng xuống 40° so với trục X (`GRASP_TILT`), tiến từ phía −X, hai má đóng theo trục Y và ôm hai mặt lớn của vỏ PVC. Đúng khuyến nghị trong spec: *gắp vào thân vỏ, tránh gắp chân đồng*. Góc 40° được chọn vì để với tới tường xa mà cổ tay không chạm đỉnh cột (devlog V2).

### 4.5. Lực kẹp

Spec yêu cầu 12–30 N. Tay kẹp trong mô phỏng là khớp xoay (driver joint) với mục tiêu "đóng 0,8 rad", nhưng giới hạn mô-men `joint_effort_limit = 1,5 N·m`. Khi má kẹp chạm vào phích, khớp không đi tiếp được và chỉ tác dụng đúng mô-men tối đa. Lực kẹp *gián tiếp* bị chặn bởi mô-men giới hạn. Devlog đo được khoảng 26–28 N mỗi má.

---

## 5. Đọc code từng file

### 5.1. `ur5_screen_geom_V3.py`: sách luật dùng chung

File này chỉ có hằng số và hàm numpy thuần.

**Bố cục nhiệm vụ** (các hằng số đầu file):

```python
HOME_Q = np.array([0.0, -np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, 0.0])   # tư thế Home
PLUG_A_XY = (0.22, 0.26)                                                # phích nằm đây
POSTS = ((0.50, 0.0), (0.70, 0.0), (0.90, 0.0))                         # 3 cột thẳng hàng
POST_RADIUS, POST_HEIGHT = 0.025, 0.30                                  # đường kính 5 cm, cao 30 cm
WEAVE_OFFSET, WEAVE_Z = 0.11, 0.27                                      # độ lệch bên và độ cao khi luồn
WALL_XF = 1.04                                                          # mặt ổ tại x = 1.04 m
```

Chú ý `WEAVE_Z = 0.27` thấp hơn đỉnh cột (0,30): dây quấn quanh thân cột chứ không bay qua đỉnh.

**Hình học phích** (`HOUSING`, `BLADE_*`, `GPIN_*`) lấy từ `spec.md`. Hình học ổ nằm trong `socket_boxes()`.

> Ổ cắm được dựng toàn bằng **các hộp chữ nhật** ghép lại (khoan "khe" bằng cách chừa chỗ trống giữa các hộp). Lỗ tròn của chân tiếp đất được xấp xỉ bằng đa giác 16 cạnh. Lý do (suy luận từ devlog V0): bộ va chạm hoạt động tốt với hình lồi đơn giản, còn mesh lõm thì dễ lỗi.

**Động học UR5e**: hằng số `_UR_CHAIN` mô tả sáu khớp, hàm `fk` và `ik` (xem mục 6.1, 6.2).

**Hàm tiện ích hình học xoay**: `rot_x`, `rot_y`, `rot_z` trả ma trận xoay 3×3. `tool_rotation(tilt, yaw)` trả hướng của bàn tay: trục kẹp hướng +X nghiêng xuống `tilt`, và `yaw` xoay cả bàn tay quanh trục Z (dùng khi luồn qua khe).

**Va chạm**: `plug_spheres`, `end_effector_spheres`, `clearance`, `curobo_scene` (mục 6.5).

**Điểm luồn cột**: `weave_waypoints()` (mục 6.6).

### 5.2. `ur5_screen_curobo_V3.py`: bộ lập kế hoạch

Thứ tự trong `main()`:

1. `write_robot_files()` tạo file mô tả robot cho cuRobo (URDF + YAML). Điểm đặc biệt: nó thêm một link giả `plug_link` gắn cố định vào cổ tay, cùng các quả cầu bao quanh phích. Nhờ đó cuRobo *coi phích đang cầm là một phần của robot* và tránh va chạm cho cả nó.
2. `MotionPlannerCfg.create(...)` khai báo bài toán: cảnh `G.curobo_scene()`, dung sai vị trí 1 mm, dung sai hướng 0,02, 32 hạt giống, bước nội suy 0,02 s.
3. Tính các điểm mốc: pre-grasp, grasp, lift, pre-insert, cuối đoạn cắm, retreat.
4. Gọi lần lượt các hàm lập đường đi (xem bên dưới) và nối các đoạn.
5. Tự kiểm bằng `G.clearance()` và in khoảng cách nhỏ nhất của từng đoạn.
6. Lưu mọi đoạn vào `plan.npz`.

Hai hàm lập kế hoạch bọc gọn cuRobo:

```python
def plan_pose(q_start, p, r, ...):     # đích cho bằng VỊ TRÍ + HƯỚNG của TCP
    res = planner.plan_pose(to_goal(p, r), start, max_attempts=40)
def plan_cspace(q_start, q_goal, ...): # đích cho bằng 6 GÓC KHỚP (c-space = configuration space)
    res = planner.plan_cspace(goal, start, max_attempts=40)
```

Đoạn luồn cột dùng `plan_cspace` chứ không phải `plan_pose`. Lý do: với `plan_pose`, cuRobo tự chọn một nghiệm IK nào đó, có thể là nhánh khác (khuỷu lật). Code tự tính đích bằng `ik_near` để *giữ cùng nhánh tay*, rồi mới nhờ cuRobo nối.

`line_segment` tạo đoạn thẳng: chia đường thẳng thành n điểm rồi giải IK cho từng điểm, mỗi điểm lấy nghiệm của điểm trước làm điểm xuất phát.

### 5.3. `ur5_screen_V3.py`: chương trình chính

Cấu trúc theo lớp `Example`:

| Phần | Hàm | Việc |
|---|---|---|
| Dựng robot, phích, cảnh | `_build_model_a`, `_build_solver_a` | Nạp UR5e + 2F-85 từ Menagerie, đặt gain PD, thêm sàn, cột, ổ, phích. |
| Dựng dây | `_cable_points`, `_build_model_b`, `_sync_anchors` | Tạo dây VBD, ghim đầu dây vào phích. |
| Chạy một khung hình | `step_frame` | Gán mục tiêu khớp, chạy MuJoCo rồi chạy VBD. |
| Thời gian hóa | `retime`, `line_time_path`, `hold` | Biến đường đi thành quỹ đạo có tốc độ hợp lệ. |
| Kịch bản | `load_plan`, `run`, `run_segment` | Phát lần lượt các đoạn. |
| Tri giác và cắm | `plan_insertion`, `measure_insertion` | Đo phích trong tay, tính lại đường cắm, đo sai số. |
| Lưu | `save` | Ghi quỹ đạo và hình dạng cho bước dựng video. |

**Cảnh Model A** (`_build_model_a`) gồm những điểm đáng chú ý:

- `b.add_mjcf(...)` nạp UR5e rồi xoay π quanh Z để quay robot theo đúng hướng hệ tọa độ. Tay kẹp được nạp tiếp và **gắn vào `wrist_3_link`** bằng tham số `parent_body`.
- Gain PD cho cánh tay: `ke=4500`, `kd=450`, `armature=0,2`. Gain tay kẹp: `ke=20`, `kd=1`.
- **Bù trọng lực** (`gravcomp = 1.0`) cho mọi body của robot và tay kẹp (vòng lặp chạy *trước* khi thêm phích). Robot do đó không bị võng vì trọng lượng bản thân, còn phích vẫn chịu trọng lực thật. Suy luận của tôi: mục đích là để PD bám kế hoạch chính xác hơn. Đây là một lựa chọn "dễ" so với robot thật, nơi bộ điều khiển phải tự bù trọng lực.
- Ma sát má kẹp `PAD_MU = 0,7` (cao su / PVC, spec 2.4).
- Phích được ghép từ các khối: một hộp (vỏ), hai hộp (lưỡi dẹt), một trụ (chân tròn).

**Bước thời gian** `step_frame(q_target, grip)`: mỗi khung hình dài 1/60 giây.

```python
for _ in range(self.substeps):                      # 16 bước nhỏ MuJoCo
    self.solver_a.step(self.s0, self.s1, self.control, None, self.sim_dt)
    self.s0, self.s1 = self.s1, self.s0             # hoán đổi trạng thái cũ / mới
...
for j in range(nb):                                 # 10 bước nhỏ dây VBD
    self._sync_anchors(...)                         # ghim đầu dây theo phích (nội suy)
    self.pipeline_b.collide(...)                    # phát hiện va chạm dây
    self.solver_b.step(...)
```

Mỗi khung chia thành nhiều bước con để tiếp xúc cứng không "nổ". Phích đi từ vị trí đầu khung đến cuối khung; vị trí giữa chừng được nội suy (vị trí tuyến tính, hướng bằng **Slerp** = nội suy mặt cầu của góc xoay).

---

## 6. Thuật toán chi tiết

### 6.1. FK: từ góc khớp ra vị trí tay

Robot là một chuỗi: mỗi khớp cộng thêm một phép tịnh tiến và một phép xoay lên khung tọa độ trước. `fk` trong `geom` làm đúng điều đó:

```python
r, p = np.eye(3), np.zeros(3)            # bắt đầu tại gốc, không xoay
for (t, ry, ax), qi in zip(_UR_CHAIN, q):
    p = p + r @ t                         # tịnh tiến theo khung hiện tại
    r = r @ rot_y(ry)                     # xoay cố định của link
    r = r @ (rot_z(qi) if ax == "z" else rot_y(qi))   # xoay do khớp (góc qi)
return p + r @ tcp, r                     # cộng thêm điểm TCP lệch khỏi cổ tay
```

`@` là phép nhân ma trận. `_UR_CHAIN` chép từ `ur5e.xml` của Menagerie nên cho **cùng kết quả với mô hình Newton**. Ví dụ `fk(HOME_Q)` cho TCP tại khoảng (0,492; 0,134; 0,346) m, khớp với devlog.

### 6.2. IK: damped least squares (DLS)

IK khó vì phương trình `fk(q) = đích` là phi tuyến. Cách giải: **lặp**. Mỗi vòng, xấp xỉ FK bằng phép tuyến tính quanh `q` hiện tại rồi bước một bước nhỏ.

1. Tính **sai lệch** `e` giữa đích và hiện tại: 3 số cho vị trí (`p_tgt − p`), 3 số cho hướng (`_rot_err`, xấp xỉ vector xoay còn thiếu).
2. Tính **Jacobian J** (ma trận 6×6) cho biết "khớp j nhích một chút thì TCP nhích thế nào". Code tính J bằng *sai phân hữu hạn*: nhích từng khớp 1e-6 rồi đo thay đổi.
3. Giải bước cập nhật:

   `Δq = Jᵀ (J Jᵀ + λI)⁻¹ e`    với λ = `damping = 1e-3`

   Số hạng `λI` ("giảm chấn") giúp nghiệm ổn định gần các tư thế kỳ dị (singularity), khi `J` gần mất hạng và phép nghịch đảo thuần túy sẽ cho bước nhảy khổng lồ.
4. Lặp đến khi sai vị trí < 2 µm và sai hướng < 1e-5, hoặc hết 80 vòng.

Nghiệm tìm được **phụ thuộc điểm xuất phát `q0`** (nó chui vào hố nghiệm gần nhất). Tính chất này được tận dụng ở hai nơi:

- Đoạn thẳng: mỗi điểm dùng nghiệm điểm trước làm `q0`, nên `q` biến đổi liên tục, không nhảy nhánh.
- `ik_near`: thử 40 điểm xuất phát (điểm gốc và 39 điểm được cộng nhiễu Gaussian σ = 0,4 rad quanh `q_prev`), giữ nghiệm **gần `q_prev` nhất** và nằm trong giới hạn khớp. Seed cố định (`default_rng(0)`) nên kết quả lặp lại được.

### 6.3. Quỹ đạo thẳng và hướng bàn tay

`line_segment` giải IK ở 25–40 điểm dọc đường thẳng với **hướng cố định** (`tool_rotation()`). Trục tay kẹp là +Y của `wrist_3`, trong thế giới chỉ về hướng (cos 40°, 0, −sin 40°) = (0,766; 0; −0,643). Lùi 8 cm theo trục đó từ điểm gắp ta được điểm pre-grasp, để robot lao vào theo hướng khép má kẹp chứ không đập xuống từ phía trên.

### 6.4. cuRobo tìm đường như thế nào

Tôi đã đọc phần gọi hàm, chưa đọc kỹ ruột tối ưu của cuRobo, nên phần này là mô tả khái quát.

- Coi đường đi là chuỗi cấu hình khớp theo thời gian, rồi **tối ưu** cho nó nối điểm đầu với điểm cuối, mượt, và **phạt va chạm**.
- Chạy song song **32 hạt giống** (`num_trajopt_seeds=32`) trên GPU, lấy đường tốt nhất.
- Nếu thất bại, các lần thử sau nhờ **graph planner** (tìm đường theo đồ thị) cho gợi ý khởi đầu. Tối đa `max_attempts=40`; hết mà vẫn thất bại thì code báo lỗi `RuntimeError`.
- Kết quả được nội suy mỗi 0,02 s (`interpolation_dt`).

### 6.5. Kiểm tra va chạm: nhiều quả cầu và các hộp

cuRobo không kiểm tra lưới tam giác thật của robot. Nó **phủ cánh tay bằng nhiều quả cầu** (file `ur5e_screen.yml`; ví dụ `upper_arm_link` có 5 cầu, `wrist_3_link` có 10 cầu, `plug_link` có 5 cầu), còn cột/tường/sàn là **hộp**.

```
   robot thật            robot trong cuRobo
   ┌────────┐             (○)(○)(○)(○)(○)   ← mỗi link phủ bằng chuỗi cầu
   thanh dài      ─►       cột = hộp 5×5×30 cm
```

Với mỗi tư thế, cuRobo tính FK ra tâm từng cầu rồi đo khoảng cách cầu tới hộp gần nhất; khoảng cách quá nhỏ bị phạt. Hệ quả cần nhớ:

- **Chỗ nào không có cầu thì không bị kiểm.** Ở V2, cầu cổ tay chạm đỉnh cột nên phải hạ độ cao phích và đổi góc nghiêng.
- Cột thật trong Newton là **hình trụ** bán kính 2,5 cm; hộp 5×5 cm bao ngoài nó nên cuRobo hơi bảo thủ (có thể từ chối đường thực ra đi được). Lý do dùng hộp: devlog V0 ghi cột trụ bị cuRobo bỏ qua.
- Do đó code còn có một lớp kiểm thứ hai độc lập là `G.clearance()`: tính khoảng cách tay kẹp (+phích) tới cột hữu hạn, tường và sàn rồi in ra.
- **Lỗ hổng:** các đoạn thẳng (Descend, Lift, Insert, Retreat) không đi qua cuRobo nên cánh tay không được kiểm va chạm trên các đoạn đó. Ở V0 từng có lỗi vì vậy (khớp `wrist_1` va cột lúc đẩy phích).

### 6.6. Luồn cột (weave)

`weave_waypoints()` sinh các mốc cho **tâm phích**, kèm góc xoay bàn tay quanh Z:

| Mốc | Phích (x, y, z) | Góc xoay tay | Ý nghĩa |
|---|---|---|---|
| post1 | (0,50; +0,11; 0,27) | 0° | Bên cạnh cột 1, phía +Y |
| gap12_in | (0,60; +0,11; 0,27) | −90° | Đầu vào khe giữa cột 1–2 |
| gap12_out | (0,60; −0,11; 0,27) | −90° | Đã chui qua khe sang phía −Y |
| post2 | (0,70; −0,11; 0,27) | 0° | Bên cạnh cột 2, phía −Y |
| gap23_in | (0,80; −0,11; 0,27) | 60° | Đầu vào khe 2–3 |
| gap23_out | (0,80; +0,11; 0,27) | 60° | Sang phía +Y |
| post3 | (0,90; +0,11; 0,27) | 0° | Bên cạnh cột 3, phía +Y |
| pre_insert | Cách ổ 6 cm | 0° | Sẵn sàng cắm |

Vì sao phải **xoay bàn tay** ở khe? Khe giữa hai cột chỉ rộng 0,20 − 0,05 = 0,15 m, mà phích dài khoảng 11 cm cộng với bàn kẹp rộng khoảng 9 cm. Nằm ngang thì không lọt. Xoay để phích dọc theo hướng đi thì lọt. Khe 2–3 chỉ xoay 60° thay vì 90° vì 90° nằm ngoài tầm với của UR5 tại đó (IK vô nghiệm, devlog V2 lỗi 4).

Mỗi chặng giữa hai mốc: `ik_near` tính cấu hình đích, `plan_cspace` nối từ cấu hình trước, sau đó `G.clearance` kiểm. Dây theo phích nên quấn quanh cột là hệ quả của quỹ đạo phích, chứ không có thuật toán "quấn dây" riêng. Chỉ có quỹ đạo phích được lập kế hoạch; dây thì được mô phỏng và quan sát.

### 6.7. Thời gian hóa (làm quỹ đạo đi đúng tốc độ)

Spec cấm chạy nhanh khi mang phích (< 0,3 m/s, gia tốc < 1 m/s²). Hai hàm xử lý:

**`retime`** (cho đoạn cuRobo): đo tốc độ và gia tốc lớn nhất của TCP trên quỹ đạo, tính hệ số kéo dãn

`s = max(1, v_max_thực / v_cho_phép, √(a_max_thực / a_cho_phép))`

rồi kéo giãn thời gian `s` lần (nội suy lại ở 60 Hz). Vì nếu kéo thời gian `s` lần thì vận tốc chia `s` còn gia tốc chia `s²`, nên mới có căn bậc hai cho gia tốc. Hàm này luôn làm chậm đi, không bao giờ làm nhanh lên (nhờ `max(1, ...)`).

**`line_time_path`** (cho đoạn thẳng): dùng luật thời gian **smoothstep** `s(u) = u²(3 − 2u)`, bắt đầu và kết thúc với vận tốc 0, nên robot nhích êm. Đạo hàm cực đại của smoothstep là 1,5 nên thời gian đoạn thẳng được đặt `dur = max(0,5; 1,5·L / vpeak)` để vận tốc đỉnh đúng bằng `vpeak`.

Các tham số mặc định: `v_carry = 0,25 m/s`, `a_carry = 0,8 m/s²`, `v_line = 0,06`, `v_lift = 0,08`, `v_insert = 0,02 m/s`.

### 6.8. Tri giác trong tay và cắm phích (`plan_insertion`)

Sau khi vận chuyển xong, phích có thể nằm lệch đôi chút trong tay kẹp (nghiêng, trượt vài mm). Nếu cứ cắm theo kế hoạch gốc thì chân phích lệch lỗ. Chương trình làm như sau:

1. **Đo** vị trí và hướng phích thật (`plug_pose()`, đọc từ trạng thái mô phỏng) và vị trí/hướng TCP từ FK.
2. Tính **tư thế của phích trong hệ TCP**: `r_rel = r_tcpᵀ · r_plug`, `p_rel = r_tcpᵀ · (p_plug − p_tcp)`.
3. Chọn **hướng mong muốn của phích** là căn thẳng theo trục thế giới (chân phích hướng +X, vuông góc ổ). Từ đó suy ra TCP phải đứng ở đâu và hướng nào: `p_goal_tcp = p_plug_mong_muốn − r_goal_tcp @ p_rel`.
4. Tạo hai đoạn: **`correct`** (nhích nhẹ về điểm cách ổ 6 cm, 20 điểm) rồi **`insert`** (đẩy thẳng 60 điểm, 0,02 m/s, vượt quá điểm sâu nhất 2 mm bằng `overshoot` để có lực đẩy).

Điều này tương đương một bước "căn chỉnh theo phản hồi" đơn giản. **Lưu ý khi so với thực tế:** `plug_pose()` đọc *giá trị thật* của mô phỏng. Robot thật không có thông tin này miễn phí mà phải có camera hoặc cảm biến lực (xem mục 8).

### 6.9. Tay kẹp, ma sát và lực

- Tay kẹp là một khớp đóng/mở điều khiển bằng vị trí (PD). Đóng = mục tiêu 0,8 rad, nhưng khi bị phích chặn lại thì mô-men bị giới hạn bởi `grip_effort`.
- Ma sát giữ phích là kiểu Coulomb: lực ma sát tối đa = μ × lực pháp tuyến. Spec tính lực kẹp tối thiểu F ≥ m·g / (2μ) × SF = 3,39 N, khuyến nghị 12–18 N. Mô phỏng cho 26–28 N, lớn hơn khuyến nghị nhưng dưới ngưỡng 30 N.
- Hàm `grip_force` đọc lực tiếp xúc trực tiếp từ dữ liệu MuJoCo (các contact có phích) để in ra.

### 6.10. Mô phỏng sợi dây (VBD)

Dây được chia thành 48 đoạn nhỏ (capsule bán kính 5 mm, tổng dài 1,2 m) nối với nhau bởi các ràng buộc **chống giãn** (`stretch_stiffness`) và **chống uốn** (`bend_stiffness`).

Hai thông số hay gây nhầm:

| Thông số | Giá trị mặc định | Ý nghĩa |
|---|---|---|
| `stretch_stiffness` | 1e6 | Dây gần như không giãn. Nhỏ quá thì dây "đứt" (các đoạn tách xa nhau, devlog V2 lỗi 1). |
| `cable_contact_ke` | 1e5 | Độ cứng tiếp xúc dây với cột, tường, sàn. |

Bài học V3: **hai độ cứng phải cân đối.** Dây rất cứng (1e6) nhưng tiếp xúc mềm (1e3) thì khi dây căng quanh cột, lực căng áp đảo lực đẩy của tiếp xúc và dây bị ép xuyên cột (đo được 17,8 mm). Tăng `ke` lên 1e5 hết lỗi. Đây là ví dụ điển hình của một lỗi mô phỏng: **chẩn đoán bằng cách đo, đặt giả thuyết, rồi kiểm chứng bằng thí nghiệm riêng** (devlog V3 phát lại quỹ đạo với `ke` = 1e3, 1e5, 1e7).

Dây ban đầu không nằm thẳng mà xếp thành hai đoạn song song nối chữ U ngay cạnh robot (`_cable_points`), để lúc phích di chuyển nó bị kéo theo đường luồn cột thay vì căng thẳng băng ngay lập tức.

### 6.11. Dựng video (`ur5_screen_render_isaac_V3.py`)

Không tính vật lý lần nữa, chỉ **phát lại** (replay):

1. Đọc `traj.npz` (pose của từng body ở từng khung) và `shapes.pkl` (hình dạng: hộp, trụ, mesh, capsule).
2. Dựng cảnh USD trong Isaac Sim: mỗi hình là một "prim" gắn vào body tương ứng.
3. Mỗi khung (`stride=2` nên cứ hai khung mô phỏng lấy một): đặt lại transform của mọi body, render RGB, vẽ chồng bằng Pillow:
   - Vệt quỹ đạo phích (màu cyan).
   - Gizmo trục **OXYZ** ở góc trái dưới (chiếu hướng các trục lên màn hình).
   - Mũi tên **trọng lực P** (vàng) đặt ở giữa cánh tay (trung bình vị trí các link từ vai đến cổ tay).
4. Đẩy ảnh vào `ffmpeg` để nén thành mp4 (libx264).

Hàm `project(p)` là camera lỗ kim: chiếu điểm 3D lên điểm ảnh 2D bằng tiêu cự. Nó giúp vẽ gizmo và mũi tên đè lên video.

---

## 7. Một lần chạy diễn ra như thế nào (theo thời gian)

Theo hàm `run()` trong `ur5_screen_V3.py`:

| Bước | Đoạn | Điều xảy ra |
|---|---|---|
| 1 | `home` (0,5 s) | Đứng yên ở Home, tay kẹp mở. |
| 2 | `approach` | cuRobo đưa TCP đến điểm cách phích 8 cm, hướng nghiêng 40°. |
| 3 | `descend` | Đi thẳng xuống điểm gắp. |
| 4 | `grasp`, `grasp_hold` | Đóng tay kẹp từ từ (smoothstep, 1,5 s), giữ 0,5 s. |
| 5 | `lift`, `lift_hold` | Nhấc thẳng lên độ cao 0,27 m. |
| 6 | `weave_*` | Các chặng luồn cột do cuRobo lập. |
| 7 | `settle` | Dừng 0,6 s cho phích và dây ổn định. |
| 8 | `correct`, `align_hold` | Đo phích trong tay, chỉnh tới điểm pre-insert, giữ 0,4 s. |
| 9 | `insert`, `push_hold` | Đẩy thẳng vào ổ ở 0,02 m/s, giữ 1 s. |
| 10 | `release` | Mở tay kẹp từ từ (2 s). Đo lại vị trí phích. |
| 11 | `retreat`, `return`, `end` | Lùi 8 cm, về Home. |

Sau đó `save()` ghi kết quả, và nếu có `--test` thì `assert lateral < 0.0005` (chỉ kiểm sai lệch trục dưới 0,5 mm).

> **Lưu ý trung thực:** kiểm tra `--test` trong code chỉ yêu cầu sai lệch ngang < **0,5 mm**, chứ không phải 0,07 mm như spec. Bản thân devlog cũng ghi rõ yêu cầu 0,07 mm **chưa đạt**.

---

## 8. Liên hệ với sim2real

**Sim2real gap** là khoảng cách giữa mô phỏng và thật. Dự án hiện chỉ có **phần sim**; chưa có bước nào chuyển sang robot thật. Để bạn học, dưới đây là những chỗ mà mô phỏng này "dễ hơn" thực tế (phần lớn là suy luận của tôi từ code):

| Chỗ trong code | Mô phỏng làm | Robot thật sẽ khác thế nào |
|---|---|---|
| `plug_pose()` đọc từ sim | Biết chính xác phích nằm đâu trong tay | Phải dùng camera hoặc cảm biến lực, có nhiễu. |
| `gravcomp = 1.0` | Robot không bị võng vì trọng lượng | Bộ điều khiển thật phải bù trọng lực (có sai số). |
| Bám quỹ đạo bằng PD lý tưởng | Không có trễ, không có nhiễu encoder | Có độ trễ, rung, sai số bám. |
| Tiếp xúc "mềm" (penalty, `ke`) | Phích vào được khe 0,07 mm vì tiếp xúc cho phép xuyên ~0,1 mm | Phải có tính thuận theo (compliance), cảm biến lực, hoặc vát dẫn hướng. |
| Dây tác động một chiều | Dây không cản phích | Dây thật nặng, ma sát, kéo ngược, quấn. |
| Dây ngắn 1,2 m | Đơn giản hóa | Spec ghi dây dài hơn nhiều. |
| Robot là UR5e | Mô hình có sẵn | Spec ghi UR5 (cùng tầm với 850 mm nhưng giới hạn khớp và thông số khác). |

Một nhận xét đáng chú ý từ `spec.md`: độ lặp lại (repeatability) của UR5 là ±0,1 mm, mà yêu cầu căn chỉnh là 0,07 mm. Nghĩa là về nguyên tắc, một robot thật không thể đạt sai số cỡ đó bằng cách chỉ chạy theo vị trí. Thực tế người ta thường dùng **vát dẫn hướng** (chamfer 1,5 mm trong spec ổ), **tìm lỗ bằng lực** (force/spiral search) hoặc **tay kẹp có độ mềm** để bù. Hướng phát triển trong devlog cũng đề cập vát dẫn hướng và servo lực.

Các kỹ thuật phổ biến để thu hẹp gap mà bạn nên biết tên (chưa có trong dự án):

- **System identification (nhận dạng hệ thống):** đo tham số thật (ma sát, khối lượng) rồi chỉnh sim cho khớp.
- **Domain randomization:** ngẫu nhiên hóa tham số (ma sát, vị trí đầu phích, độ cứng...) khi mô phỏng để kế hoạch/chính sách chịu được sai khác.
- **Điều khiển phản hồi:** dùng cảm biến lực/thị giác điều chỉnh liên tục thay vì phát lại quỹ đạo cố định.
- **Kiểm tra lặp lại nhiều lần:** devlog ghi chưa làm; đây là bước đầu tiên để biết kết quả có ổn định không.

---

## 9. Kết quả và hạn chế

### Kết quả V3 (theo devlog, chạy `python ur5_screen_V3.py --test`)

- Toàn bộ chuỗi Home → Approach → Grasp → Lift → luồn cột 1-2-3 → Cắm → Nhả → Return chạy đủ trong **41,5 s mô phỏng**.
- Phích cắm hết cỡ và tự giữ sau khi nhả (mặt vỏ cách mặt ổ 0,45 mm; thiết kế là 0,5 mm).
- Dây không xuyên cột nào; tỉ lệ giãn đoạn lớn nhất 1,005.
- Lực kẹp khoảng 27 N mỗi má, tốc độ TCP ≤ 0,25 m/s khi mang phích.

### Chưa đạt

- Sai số chân phích **0,19 mm** ở mặt ổ và 0,11–0,12 mm ở đầu lưỡi, nghiêng khoảng 1°. Yêu cầu là < 0,07 mm. Phích vào được là nhờ độ mềm của tiếp xúc, không phải nhờ đủ chính xác.
- Khi bật tiếp xúc cứng (`SCREEN_STIFF=1`), phích bị kẹt, vì độ chính xác của tri giác trong tay (~0,5°, tương đương ~0,2 mm ở đầu chân) lớn hơn khe hở.

### Các hạn chế khác

- Đoạn thẳng không được cuRobo kiểm va chạm cánh tay.
- Dây tác động một chiều lên phích; chưa đo lực căng dây.
- Chưa kiểm tính lặp lại; cũng không thấy chỗ nào đặt seed cho torch (cuRobo có thể không tái lập chính xác giữa các máy). Để tái lập đúng video, dùng file `plan.npz` đã lưu, không dùng `--replan`.

### Cách chạy lại

```bash
cd data/scripts
python ur5_screen_V3.py --test            # chạy mô phỏng (tự gọi cuRobo nếu cần)
python ur5_screen_render_isaac_V3.py      # tạo data/results/video_ur5_screen_V3.mp4
```

Tùy chọn hữu ích: `--no-cable` (bỏ dây cho nhanh khi gỡ lỗi), `--until grasp|transport|end` (dừng sớm), `--replan` (ép cuRobo lập lại), `--grip-effort`, `--v-carry`.

---

## 10. Python cho người mới: các kiểu viết trong code

| Đoạn code | Giải thích |
|---|---|
| `from __future__ import annotations` | Cho phép ghi kiểu dữ liệu (như `-> np.ndarray`) thoải mái. Chỉ là chú thích, không đổi cách chạy. |
| `import ur5_screen_geom_V3 as G` | Nạp file khác và đặt tên ngắn `G`. Dùng `G.fk(...)`. |
| `sys.path.insert(0, ...)` | Chỉ cho Python biết cần tìm file `geom` ở thư mục này. |
| `np.array([...])` | Mảng numpy: cho phép tính toán cả mảng cùng lúc, ví dụ `a + b` cộng từng phần tử. |
| `A @ B` | Nhân ma trận (khác với `A * B` là nhân từng phần tử). |
| `r.T` | Ma trận chuyển vị. |
| `x[:6]`, `x[:, 0]` | Cắt mảng: 6 phần tử đầu; cột đầu tiên. |
| `zip(a, b, strict=True)` | Duyệt hai danh sách song song, báo lỗi nếu độ dài khác nhau. |
| `f"{x:.2f}"` | f-string: nhúng giá trị vào chuỗi, `.2f` là hai chữ số thập phân. |
| `class Example:` + `self.` | Lớp (class) gom dữ liệu và hàm. `self.q_cmd` là biến thuộc về từng đối tượng. |
| `@staticmethod` | Hàm nằm trong lớp nhưng không dùng `self`. |
| `argparse` | Đọc tham số dòng lệnh (`--test`, `--until grasp`). |
| `os.environ.get("TÊN", mặc_định)` | Đọc biến môi trường, ví dụ `SCREEN_TILT`. |
| `pathlib.Path` | Xử lý đường dẫn kiểu hướng đối tượng: `HERE.parent / "results"`. |
| `np.savez` / `np.load` | Lưu nhiều mảng vào một file `.npz` rồi đọc lại. |
| `pickle` | Lưu một đối tượng Python bất kỳ (ở đây là danh sách hình dạng) vào file. |
| `subprocess.run([...])` | Chạy một chương trình khác (ở đây là bộ lập kế hoạch) như tiến trình con. |
| `assert điều_kiện, "thông báo"` | Dừng chương trình nếu điều kiện sai. Dùng để bắt lỗi sớm. |
| `if __name__ == "__main__":` | Phần chỉ chạy khi gọi file trực tiếp, không chạy khi file được `import`. |

Một bẫy quen thuộc: file này dùng `wp.quat(x, y, z, w)` của warp (thứ tự `xyzw`), trong khi cuRobo nhận quaternion thứ tự `wxyz` (xem `to_goal`: `[qw, qx, qy, qz]`). Nhầm thứ tự là lỗi cực phổ biến trong robotics.

---

## 11. Gợi ý học tiếp và bài tập

**Thứ tự đọc code đề xuất**
1. `ur5_screen_geom_V3.py`: đọc hằng số, rồi `fk`, `ik`. Tự chạy trong Python: `import ur5_screen_geom_V3 as G; print(G.fk(G.HOME_Q))`.
2. `ur5_screen_curobo_V3.py`: đọc `main()` từ dưới lên để thấy chuỗi đoạn.
3. `ur5_screen_V3.py`: đọc `run()` trước, sau đó `step_frame`, rồi quay lại `_build_model_a`.

**Bài tập nhỏ (không cần GPU)**
1. Gọi `G.fk` với vài `q` khác nhau; thử cho một khớp xoay từ 0 đến π và vẽ quỹ đạo TCP bằng matplotlib.
2. Chọn một điểm đích TCP, dùng `G.ik` từ hai điểm xuất phát `q0` khác nhau; so sánh hai nghiệm (để thấy IK có nhiều nghiệm).
3. Gọi `G.weave_waypoints()` và vẽ các mốc trong mặt phẳng XY cùng ba cột.
4. Viết lại `retime` trong ba dòng cho một đường đi giả và kiểm tra tốc độ sau khi kéo giãn.

**Thí nghiệm khi đã có môi trường chạy được**
1. Chạy với `--no-cable` để thấy mô phỏng nhanh hơn thế nào.
2. Đổi `--grip-effort` (ví dụ 0,5 và 3,0) và đọc lực kẹp in ra. Có tuột phích không?
3. Đổi `SCREEN_TILT` (góc nghiêng tay kẹp) và xem cuRobo có còn lập được đường không.
4. Thử `--cable-contact-ke 1e3` để tái hiện lỗi dây xuyên cột của V2.

**Từ khóa để tìm hiểu thêm:** *robot kinematics (Jacobian, DH parameters)*, *damped least squares IK*, *trajectory optimization*, *collision checking with sphere decomposition*, *contact models (penalty vs. complementarity)*, *domain randomization*, *peg-in-hole insertion with force control*.

---

*Tài liệu này mô tả `ur5_screen_V3`. Nếu code đổi sang phiên bản khác (V4...), nên cập nhật lại các hằng số và con số trong tài liệu.*
