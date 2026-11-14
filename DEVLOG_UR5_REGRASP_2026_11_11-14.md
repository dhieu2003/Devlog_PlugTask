# DEVLOG 2026-11-11 → 2026-11-14 – ur5_regrasp: chống tự va chạm, lệch má kẹp khi gắp dây, và V6

Tài liệu này gộp công việc của bốn ngày 11, 12, 13, 14/11/2026. Với mỗi lỗi ghi rõ: **hiện tượng → nguyên nhân (đã đo / chỉ là phỏng đoán) → giải pháp → kết quả**. Phần "đã đo" là số liệu đọc từ log mô phỏng; phần "phỏng đoán" chưa được chứng minh.

## 0. Tóm tắt

| Lỗi | Nguyên nhân | Giải pháp | Kết quả |
|---|---|---|---|
| **A. Cánh tay tự va chạm** (V1, V2; V4 chỉ "không va chạm" nhờ nghiệm phù hợp) | cuRobo **có** kiểm tra tự va chạm (cầu của `ur5.yml`, ràng buộc nhị phân) nhưng chỉ cho các đoạn nó tối ưu; các đoạn thẳng IK (`pose_line`, `line_segment`), chọn nhánh IK và mô phỏng (`enable_self_collisions=False`) không kiểm tra gì (mục 1.2–1.3, bằng chứng ở 1.1) | Hàm `self_clearance` (cầu va chạm của `ur5.yml`) để lọc nghiệm IK và đường thẳng, ngưỡng 2 mm | Mọi đoạn của V5full/V1new/V2new/V6: khoảng hở ≥ 4,1 mm (giá trị cấu trúc) |
| **B. Má kẹp lệch khi gắp dây** (V1, V4) | Cơ cấu bốn thanh **của một ngón bị gập** khi lò xo `spring_link` quá yếu (k ≤ 0,125 N·m/rad) và má chỉ chạm dây ở 1 điểm | Chế độ kẹp theo spec 2.1.8: gắp dây dùng **Parallel (k = 0,2)**, gắp phích **Encompassing (k = 0,05)** | V6: lệch má **0,01–0,02 mm** (V4: ≈ 5 mm), điểm kẹp dây giữ như V4 (không nâng 7 mm như V5full) |

Kết quả V6 đầy đủ (một lần chạy, 972 s thực / 72,2 s mô phỏng): cắm phích lệch trục 0,111 mm (giới hạn 0,2 mm), nghiêng 0,414°; dây không rơi; phích sau khi đặt dây dịch 0,620 mm so với vị trí cắm.

**Điều cần biết trước khi dùng kết quả**
- Mọi kết luận về V6 đến từ **một lần chạy đầy đủ** (và một lần chạy lại sau khi làm sạch mã nguồn cho cùng số liệu). Chưa chạy lặp để đo độ ổn định.
- Cơ chế "ngón bị gập" được chứng minh bằng số đo góc khớp và thí nghiệm nới giới hạn (mục 3.6), **chưa có phân tích cơ học** cho điều kiện ổn định của cơ cấu bốn thanh.
- Lực kẹp mỗi ngón F dùng để chọn chế độ là **ước tính** từ mô-men giới hạn (mục 4.2), không phải đo trực tiếp.
- Giá trị m = 2,5, S = 2,5, k = 0,2 và k = 0,05 trong spec 2.1.8 do người dùng đặt; tôi chỉ kiểm chứng chúng hoạt động.

## 1. Ngày 11/11 – chống tự va chạm của cánh tay (V5full, V1new, V2new, V6)

Thuật ngữ: **khớp** (joint) là trục quay; **khâu** (link) là đoạn cứng giữa hai khớp. UR5 có 6 khớp và 6 khâu va chạm (shoulder, upper_arm, forearm, wrist_1, wrist_2, wrist_3). cuRobo và tôi đều kiểm tra va chạm giữa các **khâu**, không phải giữa các khớp.

### 1.1 Hiện tượng (kèm bằng chứng)
**Quan sát ban đầu:** xem video V1 thấy cánh tay đâm xuyên chính nó khi vẫy dây; V4 thì không thấy. Trong mô phỏng `enable_self_collisions=False` nên vật lý không báo gì, chỉ nhìn thấy trên video.

**Đo lại bằng số** (`self_clearance` của tôi, trên kế hoạch V1 gốc `ur5_regrasp_V1_plan_regrasp.npz`, mỗi đoạn là một chuỗi cấu hình khớp):

| Đoạn V1 | Số điểm | Số điểm xuyên | Khoảng hở nhỏ nhất | Cặp khâu xuyên | Cấu hình xấu nhất q [độ] |
|---|---|---|---|---|---|
| wave1 | 140 | 105 | **−64,1 mm** | forearm – wrist_3 | [26, −38, 34, 122, **−226**, −53] |
| wave2 | 80 | 80 | **−64,1 mm** | forearm – wrist_3 | cùng cấu hình trên (điểm đầu của wave2) |
| wave3 | 140 | 7 | **−10,3 mm** | forearm – wrist_3 | [−14, −59, 60, 73, **−222**, −111] |
| các đoạn còn lại | — | 0 | 4,1 mm | — | — |

Ba điều đọc được từ bảng:
- Cả 192 điểm xuyên đều thuộc **một cặp duy nhất: forearm (cẳng tay) và wrist_3 (đầu cổ tay + tay kẹp)**.
- Cả hai cấu hình xấu nhất có khớp wrist_2 ở khoảng **−226° và −222°**, tức là vượt quá nửa vòng (±180°). Tôi chưa chứng minh mối liên hệ nhân quả giữa góc này và va chạm; đây chỉ là đặc điểm chung của hai cấu hình xấu nhất.
- Cấu hình xấu nhất của wave1 là điểm **cuối** của nó và cũng là điểm **đầu** của wave2 (toàn bộ 80 điểm của wave2 đều xuyên); cấu hình xấu nhất của wave3 là điểm đầu của wave3. Tức là cánh tay đi vào một tư thế xuyên và giữ trạng thái đó qua cả đoạn.

V2 gốc cũng có hiện tượng tương tự ở bước nhặt dây: nhánh khớp đế 294° và đoạn hạ thẳng xuống dây có khoảng hở **−19,4 mm** (đo cùng cách).

### 1.2 cuRobo kiểm tra những khâu nào, bỏ qua những khâu nào, và vì sao
**cuRobo có kiểm tra tự va chạm.** Cấu hình cuRobo cài trong môi trường: `MotionPlannerCfg.create(..., self_collision_check=True)` (mặc định), ràng buộc `self_collision_cfg` với `weight: 5000.0` và `convert_to_binary: True` trong `metrics_base.yml`, và khối `self_collision_cfg` có mặt ở cả bộ giải IK (`ik/lbfgs_ik.yml`) lẫn bộ tối ưu quỹ đạo (`trajopt/lbfgs_bspline_trajopt.yml`).

**Mô hình va chạm** là các quả cầu gắn vào từng khâu (`data/assets/ur5/ur5.yml`, mục `collision_spheres`): shoulder 2 cầu, upper_arm 5, forearm 5, wrist_1 2, wrist_2 2, wrist_3 10 (các cầu của wrist_3 đại diện cho cả tay kẹp Robotiq). Với 6 khâu có C(6,2) = 15 cặp khâu. cuRobo chia 15 cặp thành hai nhóm bằng bảng `self_collision_ignore`:

**Nhóm bỏ qua – 6 cặp** (không bao giờ kiểm tra):

| Cặp khâu | Khoảng hở giữa các cầu: tại Home | trên 300 cấu hình ngẫu nhiên trong giới hạn khớp (nhỏ nhất … lớn nhất) |
|---|---|---|
| shoulder – upper_arm | −107,2 mm | −107,2 … −107,1 |
| upper_arm – forearm | −43,3 mm | −43,3 … −43,3 |
| forearm – wrist_1 | −55,7 mm | −55,8 … −55,7 |
| wrist_1 – wrist_2 | −48,0 mm | −48,0 … −48,0 |
| wrist_2 – wrist_3 | −60,7 mm | −60,7 … −60,7 |
| wrist_1 – wrist_3 | +17,9 mm | −4,9 … +31,1 |

**Nhóm kiểm tra – 9 cặp:**

| Cặp khâu | Tại Home | Nhỏ nhất trên 300 cấu hình ngẫu nhiên |
|---|---|---|
| shoulder – forearm | 315,3 mm | −23,6 |
| shoulder – wrist_1 | 470,6 | −52,3 |
| shoulder – wrist_2 | 476,7 | −59,8 |
| shoulder – wrist_3 | 442,4 | −63,0 |
| upper_arm – wrist_1 | 287,3 | −50,7 |
| upper_arm – wrist_2 | 288,5 | −72,3 |
| upper_arm – wrist_3 | 378,3 | −85,0 |
| forearm – wrist_2 | **4,1** (cũng là giá trị lớn nhất trong 300 mẫu) | −5,0 |
| forearm – wrist_3 | 73,6 | −86,6 |

**Vì sao bỏ qua 6 cặp đầu?** Bằng chứng ở bảng thứ nhất: hai khâu liền kề nối với nhau bằng một khớp, và các quả cầu xấp xỉ của chúng **chồng lên nhau ở chính khớp đó, ở mọi cấu hình** (xuyên 43–107 mm, gần như không đổi theo cấu hình; tôi suy ra là do các cầu của hai khâu liền kề nằm gần trục khớp chung nên quay theo trục đó, chưa kiểm tra riêng). Nếu kiểm tra những cặp này thì **mọi** cấu hình, kể cả Home, đều bị báo "va chạm" và cuRobo không thể tìm được quỹ đạo nào. Theo thiết kế của cuRobo, đây là sai số của phép xấp xỉ bằng cầu chứ không phải va chạm thật, vì hai khâu liền kề được nối trực tiếp bằng khớp (tôi không kiểm tra điều này bằng lưới). wrist_1–wrist_3 không liền kề trực tiếp nhưng cầu của chúng sát nhau quanh cổ tay (đôi khi chồng nhau, nhỏ nhất −4,9 mm), nên cũng bị bỏ qua cùng các khâu cổ tay.

**9 cặp còn lại** là các khâu xa nhau trên chuỗi; cầu của chúng không chồng ở trạng thái bình thường (Home: 4,1 mm đến 477 mm) nhưng **có thể xuyên nhau khi cánh tay gập** (cột cuối: −5 đến −87 mm). Cặp **forearm–wrist_2 cách 4,1 mm ở Home và không lớn hơn 4,1 mm ở bất kỳ cấu hình nào trong 300 mẫu ngẫu nhiên (cũng như 1782 cấu hình của V4)**, nên nó là cặp sát nhất và 4,1 mm là giá trị cấu trúc của UR5.

Nhận xét quan trọng: cặp xuyên của V1 là **forearm–wrist_3**, một trong 9 cặp **được** kiểm tra. Nên không thể giải thích lỗi V1 bằng việc cuRobo "bỏ qua khớp liền kề".

### 1.3 Nguyên nhân
Vì cặp forearm–wrist_3 nằm trong nhóm được cuRobo kiểm tra, vấn đề không phải là cuRobo kiểm tra sai mà là **các đoạn bị xuyên không đi qua bộ kiểm tra của cuRobo**.

**Kiểm chứng 1 – cuRobo chấm kế hoạch V1 cũng báo va chạm.** Tôi cho chính bộ kiểm tra của cuRobo (`RobotCollisionChecker.get_scene_self_collision_distance_from_joints`, cùng `ur5.yml`) chấm các điểm của kế hoạch V1 (script `data/scripts/check_curobo_self_collision_V1.py`, chạy với `PYTHONPATH=~/.cache/cable_task/np2` vì cuRobo cần numpy 2):

| Đoạn V1 | Số điểm | Điểm cuRobo báo va chạm | Điểm hàm của tôi báo < 0 |
|---|---|---|---|
| approach | 121 | 0 | 0 |
| wave1 | 140 | 108 | 105 |
| wave2 | 80 | 80 | 80 |
| wave3 | 140 | 10 | 7 |

(Số "độ xuyên" của hai bên có thang đo khác nhau nên không so sánh trực tiếp; chỉ số điểm bị báo là so sánh được.) Hai bộ kiểm tra đồng ý nhau: **bộ kiểm tra không sai, nhưng kế hoạch cuối cùng vẫn chứa các điểm bị nó báo là va chạm**.

**Kiểm chứng 2 – các đoạn đó không do cuRobo tối ưu.** Trong `ur5_regrasp_curobo_V1.py`, mỗi chặng wave được tạo theo thứ tự: `pose_line` (nội suy đường thẳng TCP, giải IK nối tiếp từng điểm), `line_segment` (đoạn thẳng IK), hoặc đảo ngược một đoạn trước đó; chỉ khi các cách đó thất bại mới gọi `plan_cspace` của cuRobo. Các đoạn do cuRobo tối ưu có số điểm dạng 20k + 1 (101, 121, 141, 161 trong log V1new), còn wave1 = 140, wave2 = 80, wave3 = 140 điểm thì không. Đây là **suy luận từ số điểm và cấu trúc code**; tôi chưa chạy lại V1 để xác nhận từng đoạn.

`pose_line`, `line_segment`, `joint_line` và việc chọn nhánh IK (`ik_near`, `ik_candidates`, dùng IK số học `G.ik` của tôi) trong V1 chỉ kiểm tra sai số IK, bước nhảy khớp và khoảng hở tới cột/tường/sàn, **không kiểm tra tự va chạm**. Tóm lại:

| Thành phần | Kiểm tra tự va chạm? |
|---|---|
| `plan_pose`, `plan_cspace` (cuRobo trajopt và IK nội bộ) | **Có** (ràng buộc nhị phân, cầu `ur5.yml`) |
| `pose_line`, `line_segment`, `joint_line` (mã của tôi) | **Không** (V1, V4) |
| `ik_near`, `ik_candidates` (chọn nhánh IK, mã của tôi) | **Không** (V1, V4) |
| Mô phỏng Newton/MuJoCo | Không (`enable_self_collisions=False`) |

**V4 không va chạm là gì?** Tôi từng nói "V4 không va chạm có thể do may". Đã đo lại trên 1782 cấu hình lấy từ các kế hoạch V4: khoảng hở nhỏ nhất 4,1 mm (cặp forearm–wrist_2). V4 không va chạm thật, nhưng **không có cơ chế nào bảo đảm điều đó**; các nhánh IK của V4 vô tình không rơi vào vùng xuyên, nên đổi nhánh IK có thể gây xuyên như V1.

### 1.4 Giải pháp
Đặt đúng phép đo của cuRobo (cùng cầu, cùng bảng bỏ qua) vào **các chỗ cuRobo không phủ tới**, rồi loại mọi nghiệm/đoạn vi phạm.

**Phương trình.** Với cấu hình khớp q, cầu thứ i của khâu a có tâm c_i(q) (hệ thế giới, tính bằng động học thuận `link_frames(q)`) và bán kính r_i. Khoảng hở giữa hai khâu a, b và khoảng hở toàn cánh tay:

$$ d_{ab}(q) = \min_{i \in a,\ j \in b} \left( \|c_i(q) - c_j(q)\| - r_i - r_j \right) $$

$$ d_{\min}(q) = \min_{(a,b) \in P} d_{ab}(q) $$

Chú thích ký hiệu:
- `a, b`: hai khâu khác nhau; `i`, `j` chạy trên các cầu của a và b.
- `c_i(q)`, `c_j(q)`: tâm hai cầu; `r_i`, `r_j`: bán kính; `||.||`: khoảng cách Euclid. Biểu thức trong ngoặc là khoảng hở giữa **hai mặt cầu**: dương = tách nhau, âm = xuyên nhau.
- `P` = `SELF_PAIRS` = 9 cặp trong bảng "kiểm tra" ở mục 1.2 (6 cặp bỏ qua không có trong P, **sao chép từ `self_collision_ignore` của `ur5_build_assets_V4.py`** để hai bên dùng cùng quy tắc). Khi đang mang phích, P thêm 3 cặp: (shoulder, plug), (upper_arm, plug), (forearm, plug).

Quy tắc chấp nhận (δ = `SELF_MARGIN` = 2 mm):

$$ d_{\min}(q) \ge \delta \quad \text{(cấu hình)}, \qquad \min_k d_{\min}(q_k) \ge \delta \quad \text{(đường } q_1 \dots q_N \text{)} $$

Dạng văn bản thuần:

```
d_ab(q)  = min_{i thuộc a, j thuộc b} ( |c_i(q) - c_j(q)| - r_i - r_j )
d_min(q) = min_{(a,b) trong P} d_ab(q)
chấp nhận cấu hình q   <=>  d_min(q) >= delta = 2 mm
chấp nhận đường q_1..q_N <=>  min_k d_min(q_k) >= delta
```

Vì sao δ = 2 mm: giá trị cấu trúc nhỏ nhất của UR5 là 4,1 mm (forearm–wrist_2), nên δ = 2 mm loại được tư thế gập thật (−10 đến −68 mm) mà không chặn tư thế bình thường; δ lớn hơn 4,1 mm sẽ chặn cả tư thế Home.

**Code 1 – hàm đo (`ur5_regrasp_geom_V6.py`).** Cài đặt đúng hai công thức trên; `path_self_clearance` lấy nhỏ nhất trên cả đường.
```python
SELF_IGNORE_PAIRS = {frozenset(p) for p in [
    ("shoulder_link", "upper_arm_link"), ("upper_arm_link", "forearm_link"), ("forearm_link", "wrist_1_link"),
    ("wrist_1_link", "wrist_2_link"), ("wrist_1_link", "wrist_3_link"), ("wrist_2_link", "wrist_3_link")]}
# SELF_PAIRS: các cặp khâu được kiểm tra
SELF_PAIRS = [(a, b) for i, a in enumerate(LINK_NAMES) for b in LINK_NAMES[i + 1:] if frozenset((a, b)) not in SELF_IGNORE_PAIRS]
# SELF_MARGIN: khoảng hở tối thiểu yêu cầu giữa các cầu tự va chạm [m]. Trên các kế hoạch V4 đã chạy tốt, cặp forearm-wrist_2 chỉ
# còn ~4 mm (cấu trúc UR5), nên 2 mm đủ để loại các tư thế gập thật mà không chặn tư thế bình thường.
SELF_MARGIN = 0.002


# self_clearance: khoảng hở nhỏ nhất [m] giữa các cặp khâu không kề nhau (âm = các cầu xuyên vào nhau). carrying=True thêm cầu
# phích (gắn ở cổ tay) so với các khâu cánh tay không phải cổ tay (upper_arm/forearm/shoulder)
def self_clearance(q: np.ndarray, carrying: bool = False, detail: bool = False):
    """Minimum distance [m] between the collision spheres of non-adjacent arm links (negative = penetration). With
    ``carrying`` the plug spheres are also checked against the shoulder / upper arm / forearm. ``detail=True`` returns
    (clearance, (link_a, link_b))."""
    # world[n] = (N x 3 tâm cầu, N bán kính) của khâu n trong hệ thế giới
    world = {}
    frames = link_frames(q)
    for name, (r, p) in zip(LINK_NAMES, frames, strict=True):
        c = np.array([s[0] for s in LINK_SPHERES[name]], float)
        world[name] = (p + c @ r.T, np.array([s[1] for s in LINK_SPHERES[name]], float))
    if carrying:
        p_tcp, r_w = fk(q)
        ryaw = rot_z(hand_yaw(r_w))
        ps = plug_spheres()
        world["plug"] = (np.array([p_tcp - ryaw @ TCP_IN_PLUG + ryaw @ c for c, _ in ps]), np.array([rad for _, rad in ps]))
    pairs = list(SELF_PAIRS) + ([("shoulder_link", "plug"), ("upper_arm_link", "plug"), ("forearm_link", "plug")] if carrying else [])
    best, who = 1e9, None
    for a, b in pairs:
        ca, ra = world[a]
        cb, rb = world[b]
        d = np.linalg.norm(ca[:, None, :] - cb[None, :, :], axis=2) - ra[:, None] - rb[None, :]
        m = float(d.min())
        if m < best:
            best, who = m, (a, b)
    return (best, who) if detail else best


# path_self_clearance: khoảng hở tự va chạm nhỏ nhất trên cả một đường khớp (N x 6)
def path_self_clearance(qs, carrying: bool = False) -> float:
    """Minimum self_clearance over a joint path."""
    return min(self_clearance(q, carrying) for q in np.asarray(qs))
```

**Code 2 – lọc nghiệm IK (`ur5_regrasp_curobo_V6.py`, trong `ik_near`; `ik_candidates` có điều kiện tương tự).** Mỗi nghiệm ứng viên chỉ được giữ nếu d_min ≥ δ.
```python
        # V5: loại nghiệm làm cánh tay tự va chạm (self_clearance < SELF_MARGIN)
        if err < 1e-4 and d < best_d and np.all(q > G.UR_Q_LOWER + 0.05) and np.all(q < G.UR_Q_UPPER - 0.05) \
                and G.self_clearance(q, carrying) >= G.SELF_MARGIN:
            # Cập nhật nghiệm tốt nhất
            best, best_d = q, d
```

**Code 3 – loại các đường thẳng IK / đường khớp bị xuyên (`ur5_regrasp_curobo_V6.py`, cuối `pose_line`; `joint_line` làm tương tự).** Đây là chỗ V1 không có kiểm tra. Khi hàm trả về `None`, planner thử đoạn hoặc nhánh khác.
```python
    # V5: không được tự va chạm ở bất kỳ điểm nào
    if G.path_self_clearance(qs) < G.SELF_MARGIN:
        return None
```

**Code 4 – chốt chặn cuối cho mọi đoạn.** Sau khi lập xong kế hoạch, planner in `self-collision clearance` của từng đoạn và `raise RuntimeError` nếu < 0, nên kế hoạch không bao giờ im lặng cho qua một đoạn bị xuyên.

**Kiểm chứng ca âm tính** (hàm phải bắt được va chạm thật): tư thế gập `[0, −0,1, −3,1, −1,5, 0, 0]` cho −68 mm, `[0, −2,9, −3,0, 0, 0, 0]` cho −40 mm.

### 1.5 Kết quả
Khoảng hở nhỏ nhất mỗi đoạn (từ dòng `[check] ... self-collision clearance` trong log):

| Bản | Trước | Sau |
|---|---|---|
| V1 → V1new | wave1 −64,1 mm, wave2 −64,1 mm, wave3 −10,3 mm | cả ba **4,1 mm** (nhánh IK khác: wrist_1 quay 171°, wrist_3 tới −291°) |
| V2 → V2new | bước nhặt dây −19,4 mm (nhánh khớp đế 294°) | **4,1 mm** (cuRobo trả nhánh khớp đế ≈ 11°) |
| V4 → V5full / V6 | 4,1 mm (không có cơ chế bảo đảm) | 4,1 mm ở mọi đoạn, **có cơ chế** (kế hoạch bị dừng nếu < 0) |

Hệ quả cần lưu ý:
- Ở V2, nhánh không va chạm duy nhất là nhánh không quay; "hết va chạm" và "hết quay thừa" là một thay đổi. Nhưng với nhánh đó đoạn đặt dây `lay` làm cánh tay **chạm cột (−10,5 mm)** ở cả 10 nghiệm IK gần nhất, nên V2new chỉ chạy đến lúc nhặt dây xong.
- V1new chạy hết nhưng **dây tuột ở gap23_in** (khoảng cách dây–TCP 5,3 → 7,4 → 7,1 → 9,6 mm rồi mất). Chưa biết đây là lỗi có sẵn của V1 hay do nhánh IK mới (cần chạy V1 gốc để so).
- Khoảng hở 4,1 mm là giá trị cấu trúc, không phải dư địa an toàn; δ = 2 mm là ngưỡng chọn thủ công.

## 2. Ngày 11–12/11 – điều tra má kẹp lệch khi gắp dây

### 2.1 Hiện tượng (đã đo)
Khi gắp dây, **góc hai driver bằng nhau** (chênh 0,0000 rad) nhưng hai miếng đệm không đối xứng quanh dây. Chỉ số lệch:

$$ o = R_w^{T} \left( \frac{p_R + p_L}{2} - p_w \right) - t_{TCP}, \qquad \Delta_{jaw} = o_x $$

Dạng văn bản thuần:

```
o = R_w^T * ( (p_R + p_L)/2 - p_w ) - t_TCP        (vector 3 thành phần, hệ wrist_3)
lech_ma = o_x                                       (thành phần theo hướng khép của hai má)
```

($p_R$, $p_L$ = vị trí hai miếng đệm; $p_w$, $R_w$ = vị trí và ma trận quay của wrist_3; $t_{TCP}$ = offset TCP trong hệ wrist_3). Kẹp phích: o_x = 0,03 mm. Kẹp dây ở V4: o_x ≈ 4,7–5,2 mm (miếng đệm +36,7 / −27,4 mm); **bên lệch thay đổi giữa các lần chạy**.

### 2.2 Các giả thuyết đã bị đo đạc bác bỏ
| Giả thuyết | Thí nghiệm | Kết quả |
|---|---|---|
| Hai bộ PD của hai má mất đồng bộ | một PD cho cả hai má (ngón phải ×2, ngón trái ke = kd = 0) | lệch không đổi |
| Độ đóng / lực kẹp | đóng 0,79 thay 0,8; lực kẹp dây 0,8 thay 2,2 N·m | không đổi |
| Đầu ngón chạm sàn | đo tiếp xúc | ca đối xứng còn chạm sàn nhiều hơn |
| Dây biến dạng sau cắm nên TCP dây cao hơn (giả thuyết của người dùng) | đọc code: điểm kẹp tính từ vị trí dây **hiện tại**; dây chỉ cao hơn nghỉ 4 mm | sai |
| Vị trí dây trên miếng đệm quyết định đối xứng | T3 (nâng 7 mm) và T4 (độ cao cũ) có dây cùng vị trí so với TCP nhưng khác kết quả | sai (mục 2.3) |

### 2.3 Thí nghiệm tách biến (11–12/11, `--regrasp-only`, đặt dây tại vị trí cho trước)
| | Điều kiện | Miếng đệm phải / trái | Kết luận |
|---|---|---|---|
| T1 | độ cao cũ, không dây | +25,3 / −25,3 | đối xứng |
| T2 | dây đặt ở z = −16 mm | — | **không hợp lệ**: dây nằm dưới sàn nên bị đẩy văng |
| T3 | nâng 7 mm, dây y = −1, z = −5 | +11,7 / −11,7 | đối xứng, 3 điểm chạm mỗi bên |
| T4 | độ cao cũ, dây y = −1, z = −5 | **+36,4 / −26,6** | lệch, ngón phải 1 điểm chạm |
| T5 | độ cao cũ, dây y = +12 | **+26,2 / −36,5** | lệch ở ngón **trái** (1 điểm chạm) |
| T6 | độ cao cũ, dây z = +2 | — | **không hợp lệ**: dây bị đẩy ra ngoài |
| T7 | nâng 3 mm | **+36,2 / −23,5** | lệch, ngón phải 1 điểm chạm |

Quy luật đo được: **ngón lệch luôn là ngón chỉ chạm dây ở 1 điểm**, và bên nào lệch thì thay đổi. Góc khớp của ngón lệch: coupler ≈ −0,70, spring_link ≈ 0,155, **follower = 0,873**; ngón còn lại: −0,52, 0,38, 0,28.

### 2.4 Cách sửa tạm ở V5full
Nâng điểm kẹp dây 7 mm → lệch 0,03 mm, nhưng khác V4 và không giải thích được nguyên nhân (nâng 3 mm vẫn lệch 6 mm; 5 mm trở lên đối xứng).

## 3. Ngày 12–13/11 – nguyên nhân: lò xo `spring_link` quá yếu

### 3.1 Cấu trúc ngón (đọc từ `2f85.xml` của MuJoCo Menagerie)
Mỗi ngón là một cơ cấu bốn thanh khép kín:

```
base ─┬─ driver ── coupler                         (nhánh dẫn, có động cơ ở driver)
      └─ spring_link ── follower ── pad            (nhánh thụ động, pad chạm vật)
        (khớp có lò xo)         (follower nối vào coupler bằng ràng buộc `connect`)
```

Khớp spring_link: dải [−0,297; 0,8] rad, `stiffness` = 0,05 N·m/rad, `springref` = 2,62 rad (nằm ngoài dải nên lò xo luôn đẩy spring_link về phía giới hạn trên). Mô-men lò xo:

$$ \tau_s = k \, (q_{ref} - q_s), \qquad q_{ref} = 2.62 \ \mathrm{rad} $$

```
tau_s = k * (q_ref - q_s),   q_ref = 2.62 rad      (mo-men lo xo xoan cua spring_link, N*m)
```

Tại q_s = 0,4 rad: k = 0,05 cho tau_s = 0,05 × (2,62 − 0,4) ≈ 0,11 N·m; k = 0,2 cho tau_s ≈ 0,44 N·m (mô-men giới hạn driver 3,2 N·m). Follower có giới hạn ±0,872664 rad.

### 3.2 Giả thuyết và cách kiểm chứng
Giả thuyết: khi một ngón chỉ chạm dây ở một điểm, lực lệch tâm sinh mô-men lên cơ cấu thụ động; lò xo k = 0,05 quá yếu để giữ nên spring_link rơi về ≈ 0 và follower quay đến giới hạn: ngón **gập sang cấu hình thứ hai**, miếng đệm lùi ra → lệch má. Kiểm chứng bằng cách ghi đè `dof_passive_stiffness` của hai khớp spring_link và chạy lại ca T4.

### 3.3 Quét độ cứng, gắp dây (ca độ cao cũ, `--regrasp-only`)
| k [N·m/rad] | Miếng đệm phải / trái [mm] | Kết quả |
|---|---|---|
| 0,02 | +36,7 / −27,3 | lệch |
| 0,05 (gốc) | +36,4 / −26,6 | lệch |
| 0,1 | +36,5 / −26,1 | lệch |
| 0,12 | +37,0 / −26,7 | lệch |
| 0,125 | +34,1 / −30,1 | lệch nhẹ |
| **0,15** | **±30,05** | **đối xứng**, 1 điểm chạm mỗi bên, 42 N |
| **0,2** | **±28,85** | **đối xứng**, 1 điểm chạm mỗi bên, 39 N; giữ nguyên khi dây ở y = +12 (±29,1), y = −1 (±28,7) hoặc nâng 3 mm |
| 0,3 | ±38 | không giữ được dây (driver 0,255, má ép sàn 55 N) |
| 0,5 / 1 / 5 / 50 | ±36 / ±25 / ±5,7 / ±4,7 | không giữ được dây, má ép sàn 38–146 N |

→ **Giả thuyết đúng một phần**: k ≥ 0,15 chặn được lệch, nhưng k quá cao phá tư thế kẹp (cửa sổ k ≈ 0,15–0,2).

### 3.4 Cái giá của k = 0,2 ở khâu gắp phích
Chạy toàn bộ với k = 0,2 cố định → **cắm phích hỏng** (lệch trục 7,8 mm, nghiêng 33° trước nhả; phích văng, mô phỏng ra NaN). Quét độ cứng chỉ cho phần cắm phích (`--no-regrasp`):

| k | Lệch trục trước nhả / nghiêng | |
|---|---|---|
| 0,05 | 0,112 mm / 0,41° | tốt |
| 0,1 | 0,112 mm / 0,34° | tốt |
| 0,12 | 0,81 mm / 3,35° | kém hơn |
| 0,125 | 1,43 mm / 8,2° | hỏng |
| 0,15 | 5,15 mm / 23,8° | hỏng |
| 0,2 | 7,84 mm / 33,3° | hỏng |

Nguyên nhân (đã đo, ở k = 0,2 so với 0,05 khi kẹp phích): driver dừng ở 0,351 thay vì 0,544 rad; ngón chuyển động như hình bình hành cứng (coupler ≈ 0 thay vì −0,33); ở cuối bước kẹp **hai má chưa chạm phích** (chỉ ép sàn 13 và 129 N, so với 55,7 N lên phích ở k = 0,05); phích bị giữ lệch 9 mm theo y và 6 mm theo z so với bản gốc. Vì sao vị trí giữ lệch dẫn đến cú chèn hỏng thì **chưa đo** (có thể phích xoay trong tay kẹp khi chịu lực chèn).

→ **Không có một độ cứng duy nhất** thỏa cả hai: phích cần k ≤ 0,1, dây cần k ≥ 0,15.

### 3.5 k = 0 (Encompassing theo bản spec đầu tiên) cũng không dùng được
Với k = 0 khi gắp phích: ngón trái gập (spring_link 0,053; follower 0,873), phích bị giữ nghiêng 35°, `insertion IK failed`. Tức là gập ngón xảy ra **cả với phích** khi không có lò xo; k = 0,05 đủ để giữ ổn định ở phích (lực kẹp tâm lớn, nhiều điểm chạm). Spec sau đó được người dùng sửa thành k = 0,05.

### 3.6 Thí nghiệm nới giới hạn follower (13/11)
Nếu ngón chỉ "kẹt ở cữ chặn 0,873" thì nới giới hạn phải xóa lệch. Kết quả:

| Giới hạn follower | follower / spring_link của ngón lệch | Kết quả |
|---|---|---|
| ±0,873 (gốc) | 0,873 / 0,156 | lệch, vẫn giữ dây |
| ±1,2 | 1,19 / 0,068 | lệch nặng hơn, **không giữ dây** |
| ±1,57 | 1,34 / 0,032 | lệch nặng hơn, không giữ dây |

→ Giới hạn 0,873 không phải nguyên nhân mà chỉ **chặn đà gập**. Ngón bị gập không dừng khi bỏ cữ chặn: spring_link → 0, follower quay tiếp. Đây là mất ổn định của cơ cấu, không phải ma sát hay cữ chặn.

## 4. Ngày 14/11 – V6: chế độ kẹp theo spec.md mục 2.1.8

### 4.1 Công thức
Mỗi ngón tạo một tiếp xúc ma sát, nên để vật khối lượng m không trượt khi có gia tốc a thì phải có:

$$ 2 \mu F \ge m (g + a) \quad \Rightarrow \quad F_{sw} = \frac{m (g + a)}{2 \mu} \, S $$

```
2 * mu * F >= m * (g + a)        (hai tiep xuc ma sat, moi ngon mot tiep xuc)
F_sw = m * (g + a) / (2 * mu) * S
```

Với m = 2,5; g = 9,81; a = 0,8 m/s² (`--a-carry`); mu = 0,7 (`PAD_MU`, cao su/PVC); S = 2,5:

$$ F_{sw} = \frac{2.5 \times (9.81 + 0.8)}{2 \times 0.7} \times 2.5 = 47.37 \ \mathrm{N} $$

```
F_sw = 2.5 * (9.81 + 0.8) / (2 * 0.7) * 2.5 = 47.37 N
```

- F < F_sw: **Parallel Grasp**, lò xo spring_link k = 0,2 N·m/rad.
- F ≥ F_sw: **Encompassing Grasp**, k = 0,05 N·m/rad.

### 4.2 Lực kẹp F mỗi ngón
Không đo trực tiếp lúc quyết định; ước tính từ mô-men giới hạn driver, hiệu chuẩn bằng phép đo kẹp phích (56,4 N ở 3,2 N·m):

$$ F = 17.6 \cdot \tau_{max} \qquad \Rightarrow \qquad F_{plug} = 56.4 \ \mathrm{N}, \quad F_{cable} = 38.8 \ \mathrm{N} $$

```
F = 17.6 [N/(N*m)] * tau_max          tau_max = mo-men gioi han cua driver
phich: 17.6 * 3.2 = 56.4 N            day: 17.6 * 2.2 = 38.8 N
```

| Vật | F | F_sw | Chế độ | k |
|---|---|---|---|---|
| Phích | 56,4 N | 47,37 N | Encompassing | 0,05 |
| Dây | 38,8 N | 47,37 N | Parallel | 0,2 |

Lực tiếp xúc đo được khi gắp dây ở k = 0,2: 39–43 N mỗi bên, phù hợp ước tính 38,8 N.

### 4.3 Cách gắn vào V6
- `grasp_mode(effort, a_acc)` tính F_sw, F và trả về (chế độ, k, F, F_sw).
- `Example.__init__`: dựng mô hình với k của chế độ **phích** (ghi vào `mujoco:dof_passive_stiffness` của hai khớp spring_link).
- `regrasp_phase`: ngay trước khi tiếp cận dây, gọi `grasp_mode` cho **dây** rồi `set_spring_k(k)` (ghi lại bảng độ cứng + `notify_model_changed(JOINT_DOF_PROPERTIES)`).
- Điểm kẹp dây **giữ như V4** (không nâng 7 mm); giới hạn follower **giữ 0,873**.

### 4.4 Kết quả V6 (một lần chạy `ur5_regrasp_V6.py`, chạy lại sau khi làm sạch mã nguồn cho cùng số liệu)
| Chỉ tiêu | V4 / V5full | **V6** |
|---|---|---|
| Lệch má khi kẹp phích | 0,03 mm | 0,03 mm |
| Lệch má khi gắp dây | V4: ≈ 5 mm; V5full: 0,03 mm (nâng 7 mm) | **0,01–0,02 mm** (không nâng) |
| Góc driver khi gắp dây (R / L) | 0,776 / 0,776 | 0,652 / 0,652 |
| Cắm phích: lệch trục trước nhả / nghiêng | 0,112 mm / 0,41° | 0,111 mm / 0,414° |
| Phích dịch tối đa khi vẫy dây | 0,95 mm (V5full) | 1,058 mm |
| Phích dịch sau khi đặt dây | 0,51 mm (V5full) | 0,620 mm |
| Khoảng cách dây–TCP khi kẹp | 19,6–20,2 mm (V5full) | 21,1–21,5 mm |
| Dây rơi | không | không |
| Khoảng hở tự va chạm | 4,1 mm | 4,1 mm |

## 5. Sai sót của tôi (để bạn biết khi đọc lại)
- Nói V4 tránh tự va chạm "có thể do may" khi chưa đo → sai (mục 1.3).
- Nghi nhiều lần "đầu ngón chạm sàn" và "hai PD không đồng bộ" gây lệch → đều sai (mục 2.2).
- Giả thuyết "vị trí dây trên miếng đệm quyết định" → bị T3/T4 bác bỏ (mục 2.3). Thí nghiệm T2 và T6 không hợp lệ vì đặt dây dưới sàn / ngoài tay kẹp.
- Ngày 12/11 tôi để một tiến trình chờ nền dùng `pgrep -f` tự khớp với chính nó nên **không bao giờ kết thúc**, làm mất hơn một giờ chờ vô ích trước khi người dùng nhắc.
- Ban đầu tôi nói cấu hình của mình khác Menagerie ở `solref` của ràng buộc driver; đọc lại file thì giá trị 0,005 là **của Menagerie**. Khác biệt thật chỉ là hai actuator driver riêng (kp = 20, kd = 1) thay cho một actuator tendon `split`, và thí nghiệm "một PD" không thay đổi kết quả.
- Chạy V6 với k = 0,2 cố định cho toàn bộ nhiệm vụ làm hỏng phích (mục 3.4); chọn k theo giai đoạn mới đúng.
- Lần đầu tôi đọc công thức F_sw với m theo từng vật (phích 0,1 kg, dây ≈ 0,11 kg) nên F_sw ≈ 1–1,5 N và dây ra Encompassing; công thức đã được người dùng sửa thành m = 2,5, S = 2,5.

## 6. Hạn chế và việc tiếp theo
1. Chưa chạy lặp V6 để đo độ ổn định; các lần chạy trước đây không tất định (phích dịch 1–4 mm, đôi khi NaN).
2. Chưa có phân tích cơ học cho điều kiện ổn định của cơ cấu bốn thanh; hiện chỉ có số đo góc khớp và quét độ cứng. Cửa sổ k ≈ 0,15–0,2 cho gắp dây rất hẹp (k = 0,3 đã hỏng).
3. F ước tính từ mô-men giới hạn; nên đo F trực tiếp lúc kẹp và dùng cho quyết định chế độ. Với dây chỉ cách ngưỡng 8,6 N (38,8 so với 47,37 N).
4. Ma sát μ trong F_sw lấy `PAD_MU = 0,7` cho cả phích và dây, trong khi proxy dây trong mô phỏng có μ = 1,2.
5. Ngón gập còn xảy ra nếu đổi k hoặc lực kẹp; nên thử thêm giảm chấn cho spring_link như một cách khác để chặn gập mà ít ảnh hưởng đến kẹp phích (chưa thử).
6. V1new: dây tuột ở gap23_in (chưa biết có sẵn ở V1 không). V2new: đoạn đặt dây chạm cột (−10,5 mm), chỉ chạy đến lúc nhặt dây.
7. Nguồn gốc các đoạn va chạm của V1 (đoạn wave1–wave3 không do cuRobo tối ưu) suy ra từ số điểm của đoạn và cấu trúc code V1, chưa chạy lại V1 để xác nhận từng đoạn.
8. Mô phỏng vẫn tắt `enable_self_collisions`; an toàn tự va chạm dựa vào mô hình cầu, chưa so với lưới ở tư thế biên.

## 7. File và cách chạy lại
| File (`data/scripts/`) | Nội dung |
|---|---|
| `ur5_regrasp_V6.py` | mô phỏng chính (đã làm sạch, chú thích từng dòng) |
| `ur5_regrasp_geom_V6.py` | hình học, `self_clearance`, `path_self_clearance` |
| `ur5_regrasp_curobo_V6.py` | planner cuRobo (lọc nghiệm IK tự va chạm) |
| `ur5_regrasp_render_isaac_V6.py` | quay video Isaac |

Video: `data/results/video_ur5_regrasp_V6.mp4`. Log: `data/results/run_V6.log`.

```
cd data/scripts
python ur5_regrasp_V6.py --test                     # chuỗi đầy đủ V6 (~16 phút)
python ur5_regrasp_render_isaac_V6.py --stride 3    # video
```


## 8. Code sửa lỗi lệch má kẹp (trích nguyên văn từ V6; code tự va chạm nằm ở mục 1.4)

Các đoạn dưới đây lấy trực tiếp từ file nguồn V6 (đường dẫn ghi ở đầu mỗi đoạn); đọc kèm chú thích tiếng Việt trong file.

### 8.1 Lệch má – chọn chế độ kẹp theo spec 2.1.8 (`ur5_regrasp_V6.py`)
Hằng số và hàm `grasp_mode` cài đặt F_sw và quy tắc F < F_sw ở mục 4.1–4.2.
```python
K_PARALLEL = 0.2
K_ENCOMPASSING = 0.05
# GRASP_S: hệ số an toàn S của công thức F_sw (spec: 1 <= S <= 4)
GRASP_S = 2.5
# GRASP_MASS: khối lượng m trong công thức F_sw [kg] (spec: 2.5)
GRASP_MASS = 2.5
# F_PER_NM: lực pháp tuyến mỗi má / mô-men giới hạn của driver [N/(N·m)]; hiệu chuẩn từ phép đo kẹp phích: 56.4 N ở 3.2 N·m
F_PER_NM = 56.4 / 3.2


def grasp_mode(effort, a_acc):
    """Trả về (chế độ, k, F, F_sw). effort = mô-men giới hạn của driver [N·m]; a_acc = gia tốc tối đa khi mang vật [m/s^2]."""
    # Ngưỡng lực F_sw = m (g + a) / (2 mu) * S, với mu = hệ số ma sát miếng đệm PAD_MU
    f_sw = GRASP_MASS * (9.81 + a_acc) / (2.0 * PAD_MU) * GRASP_S
    # Lực kẹp mỗi ngón ước tính từ mô-men giới hạn (đã hiệu chuẩn bằng F_PER_NM)
    f = F_PER_NM * effort
    # F < F_sw -> Parallel, ngược lại Encompassing
    mode = "parallel" if f < f_sw else "encompassing"
    # Trả về chế độ, độ cứng tương ứng, F và F_sw
    return mode, (K_PARALLEL if mode == "parallel" else K_ENCOMPASSING), f, f_sw
```

### 8.2 Lệch má – đặt độ cứng ban đầu (chế độ phích) khi dựng mô hình (`ur5_regrasp_V6.py`, trong `Example.__init__`)
```python
        # ---- V6: chế độ kẹp ban đầu = chế độ kẹp PHÍCH (mục 2.1.8 spec.md) -> chọn độ cứng k của lò xo spring_link ----------------
        # grasp_mode() so sánh lực kẹp mỗi ngón F với ngưỡng F_sw và trả về (chế độ, k, F, F_sw)
        mode_plug = grasp_mode(float(self.args.grip_effort), self.args.a_carry)
        # In chế độ được chọn để đối chiếu với spec
        print(f"[Mode] plug: F={mode_plug[2]:.1f} N per finger, F_sw={mode_plug[3]:.2f} N -> {mode_plug[0]} (k={mode_plug[1]})", flush=True)
        # Thuộc tính tùy biến của MuJoCo chứa độ cứng lò xo thụ động của từng bậc tự do (đơn vị N·m/rad với khớp xoay)
        stiff_attr = b.custom_attributes["mujoco:dof_passive_stiffness"]
        # Nếu bảng giá trị chưa tồn tại thì tạo bảng rỗng
        if stiff_attr.values is None:
            stiff_attr.values = {}
        # spring_dofs = chỉ số bậc tự do của khớp spring_link bên phải và bên trái (để đổi độ cứng lúc chạy, xem set_spring_k)
        self.spring_dofs = []
        # Với từng bên:
        for side in ("right", "left"):
            # Tìm khớp spring_link của bên đó
            j_spring = _find_label_index(b.joint_label, f"/{side}_spring_link_joint")
            # Ghi nhớ chỉ số bậc tự do của khớp
            self.spring_dofs.append(b.joint_qd_start[j_spring])
            # Đặt độ cứng ban đầu theo chế độ kẹp phích
            stiff_attr.values[b.joint_qd_start[j_spring]] = float(mode_plug[1])
```

### 8.3 Lệch má – đổi độ cứng lúc đang chạy (`ur5_regrasp_V6.py`)
`dof_passive_stiffness` là bảng độ cứng lò xo thụ động của Newton/MuJoCo; sau khi sửa bảng phải gọi `notify_model_changed(JOINT_DOF_PROPERTIES)` để bộ giải nạp lại `jnt_stiffness`.
```python
    # V6: đổi độ cứng lò xo spring_link khi chuyển chế độ kẹp (Parallel k = 0.2 / Encompassing k = 0.05) ngay trong lúc chạy
    def set_spring_k(self, k):
        # Đọc bảng độ cứng lò xo thụ động của mọi bậc tự do (mảng warp -> numpy)
        arr = self.model_a.mujoco.dof_passive_stiffness.numpy()
        # Chỉ ghi đè hai bậc tự do spring_link (phải, trái)
        for d in self.spring_dofs:
            arr[d] = k
        # Ghi bảng mới về model
        self.model_a.mujoco.dof_passive_stiffness.assign(arr)
        # Báo bộ giải MuJoCo cập nhật thuộc tính khớp (jnt_stiffness)
        self.solver_a.notify_model_changed(newton.ModelFlags.JOINT_DOF_PROPERTIES)
        # In giá trị mới để kiểm tra trong log
        print(f"[Spring] spring_link stiffness -> {k} N*m/rad", flush=True)
```

### 8.4 Lệch má – gọi chuyển chế độ ngay trước khi khép má vào dây (`ur5_regrasp_V6.py`, trong `regrasp_phase`)
```python
        # V6: chọn chế độ kẹp cho DÂY theo mục 2.1.8 (lực kẹp dây = F_PER_NM * cable_grip_effort so với F_sw)
        mode_cable = grasp_mode(float(a.cable_grip_effort), a.a_carry)
        # In chế độ được chọn
        print(f"[Mode] cable: F={mode_cable[2]:.1f} N per finger, F_sw={mode_cable[3]:.2f} N -> {mode_cable[0]} (k={mode_cable[1]})", flush=True)
        # Đổi độ cứng lò xo spring_link sang giá trị của chế độ đó (Parallel: 0.2) TRƯỚC khi khép má vào dây
        self.set_spring_k(mode_cable[1])
```
