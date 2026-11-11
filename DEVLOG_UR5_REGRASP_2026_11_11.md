# DEVLOG 2026-11-11 – ur5_regrasp: chống tự va chạm (V5full, V1new, V2new) và lệch má kẹp khi kẹp dây

## 0. Overview (đọc phần này trước)
**Yêu cầu trong ngày**
1. Thêm tránh **tự va chạm của cánh tay** vào V4full (→ **V5full**), vào V1 (→ **V1new**) và vào V2 (→ **V2new**), vẫn giữ dây VBD, setup và logic của bản gốc.
2. Sửa hiện tượng **hai má kẹp lệch sang một bên** khi kẹp dây (V1, V4).
3. Xuất video V5full, V1new, V2new.

**Kết quả chính**
| | Kết quả | Mức chắc chắn |
|---|---|---|
| Tự va chạm | Có hàm `self_clearance` dùng đúng các cầu va chạm của `ur5.yml`; mọi đoạn quỹ đạo của V5full/V1new/V2new đều ≥ 4,1 mm (4,1 mm là giá trị cấu trúc của UR5 ở Home, không phải va chạm). V1 gốc đo được −64,1 mm, V2 gốc −19,4 mm | Đã đo |
| Nguyên nhân tự va chạm ở V2 | IK nội bộ của cuRobo luôn trả về nhánh quay (khớp đế 294°) và đoạn hạ thẳng xuống dây từ nhánh đó tự va chạm | Đã đo |
| Lệch má kẹp | Nâng điểm kẹp dây thêm **7 mm** → lệch 4,9 mm → **0,03 mm**, dây không tuột, phích còn trong ổ | Đã đo (1 lần chạy đầy đủ) |
| **Nguyên nhân** lệch má | **Chưa xác định.** Đã loại trừ 4 giả thuyết bằng đo đạc (mục 3.4); chỉ có tương quan và một phỏng đoán chưa kiểm chứng (mục 3.5) | Phỏng đoán |

**Điều cần biết trước khi dùng kết quả**
- Mọi video chỉ từ **một lần chạy**; các lần chạy trước đây không tất định (phích dịch 1–4 mm, đôi khi NaN).
- V1new: nhiệm vụ re-grasp **không đạt** (dây tuột ở gap23_in). V2new: video chỉ chạy **tới lúc nhặt dây xong** (mục 2.4).
- Thư mục V1, V2 gốc nằm trong thùng rác; tôi chỉ **sao chép** ra, không khôi phục hay xóa gì.

## 1. File và video
| Phiên bản | File (`data/scripts/`) | Ghi chú |
|---|---|---|
| V5full | `ur5_regrasp_{,geom_,curobo_,render_isaac_}V5full.py` | sao chép từ V4full; tên "V5full" để không đè lên các file `ur5_regrasp_V5*` cũ (V5 lá đồng, đã bị bỏ) |
| V1new | `ur5_regrasp_{,geom_,curobo_,render_isaac_}V1new.py` | sao chép từ V1 trong thùng rác |
| V2new | `ur5_regrasp_{,geom_,curobo_,render_isaac_}V2new.py` | sao chép từ V2 trong thùng rác |

| Video (`data/results/`) | Nội dung |
|---|---|
| `video_ur5_regrasp_V5full.mp4` | chuỗi đầy đủ, có dây VBD, 72 s mô phỏng |
| `video_ur5_regrasp_V1new.mp4` | chuỗi đầy đủ, 87 s; **dây tuột** sau gap23_in |
| `video_ur5_regrasp_V2new.mp4` | cắm phích rồi chỉ **nhặt dây và nhấc lên** (`--stop-after-pickup`), 48,6 s |

Bản V2new đầy đủ trước đó (có thêm `ik_candidates`, xem 2.4) đã bị ghi đè; bản sao chỉ nằm ở thư mục tạm của phiên làm việc nên không đáng tin để lưu lâu.

## 2. Tránh tự va chạm

### 2.1 Vấn đề
Ở V4, tự va chạm chỉ được xử lý ở **các đoạn do cuRobo lập kế hoạch**: cuRobo có sẵn thành phần chi phí tự va chạm, dùng các quả cầu trong `ur5.yml` (`self_collision_buffer = 0`, bỏ qua cặp khâu kề nhau theo `self_collision_ignore`). Các đoạn còn lại **không được kiểm tra**:
- đường thẳng IK nối tiếp: `line_segment` (hạ/nâng/cắm/lùi), `pose_line`, `joint_line`;
- chọn nghiệm IK: `ik_near`, `ik_candidates`, và IK nội bộ của cuRobo ở `plan_pose`;
- mô phỏng đặt `enable_self_collisions=False`, nên nếu cánh tay xuyên qua chính nó thì không có gì báo, chỉ thấy khi xem video.

Nhận định ban đầu của tôi trong ngày là "V4 không tự va chạm có thể do may". **Đã được đo lại và sửa**: trên 1782 cấu hình lấy từ các kế hoạch V4, khoảng hở nhỏ nhất là 4,1 mm (cặp forearm–wrist_2, bằng đúng giá trị ở Home) và các cặp khác ≥ 13,5 mm. Nghĩa là V4 an toàn theo mô hình cầu, không phải do may.

### 2.2 Cách làm (trong `ur5_regrasp_geom_*.py`)
- `SELF_IGNORE_PAIRS`: 6 cặp khâu kề nhau bỏ qua, **sao chép từ `self_collision_ignore` của `ur5_build_assets_V4.py`** nên hai bên (cuRobo và kiểm tra của tôi) dùng cùng một mô hình.
- `SELF_PAIRS`: 9 cặp còn lại được kiểm tra (shoulder–forearm, shoulder–wrist1/2/3, upper_arm–wrist1/2/3, forearm–wrist2/3).
- `self_clearance(q, carrying=False, detail=False)`: khoảng hở nhỏ nhất giữa các cặp quả cầu, âm = các cầu xuyên vào nhau. Dùng `LINK_SPHERES` và `link_frames` có sẵn. Khi `carrying=True`, thêm cầu của phích so với shoulder/upper_arm/forearm.
- `path_self_clearance(qs, carrying)`: nhỏ nhất trên một đường khớp.
- `SELF_MARGIN = 2 mm`: ngưỡng chấp nhận. Chọn 2 mm vì giá trị cấu trúc là 4,1 mm; 2 mm loại được tư thế gập thật mà không chặn tư thế bình thường.

**Kiểm chứng bằng ca âm tính**: các tư thế gập như `[0, −0,1, −3,1, −1,5, 0, 0]` cho −68 mm, `[0, −2,9, −3,0, 0, 0, 0]` cho −40 mm. Hàm bắt được va chạm thật.

### 2.3 Chỗ gắn vào planner (`ur5_regrasp_curobo_*.py`)
- `ik_near` / `ik_candidates`: loại nghiệm có `self_clearance < SELF_MARGIN`. Nhánh weave (đang mang phích) truyền `carrying=True`.
- `pose_line`, `joint_line`: trả về `None` nếu đường tự va chạm.
- Kiểm tra cuối cùng cho **mọi đoạn**: in `self-collision clearance`, và `raise RuntimeError` nếu < 0. Nhờ vậy kế hoạch không bao giờ im lặng cho qua một đoạn bị xuyên.

### 2.4 Kết quả theo từng phiên bản
**V5full** (từ V4full): mọi đoạn cắm phích và kẹp dây đều 4,1 mm.

**V1new** (từ V1):
- Đoạn tiếp cận dây **giống hệt V1**: quãng quay khớp 23/67/84/165/159/171°, lệch tối đa 0,0007 rad (nhiễu GPU). Tức là bộ lọc không đổi nhánh ở đoạn này; V1 vốn đã có `ik_candidates` ("no spin") nên không quay trọn vòng ở đây.
- Khác biệt thật nằm ở **wave1–wave3**. Tính trên kế hoạch V1 gốc: wave1 **−64,1 mm**, wave2 **−64,1 mm**, wave3 **−10,3 mm**; V1new: cả ba đều 4,1 mm. Bộ lọc đã loại nhánh IK tự va chạm của V1 và chọn nhánh khác (wrist_1 quay 171°, wrist_3 tới −291°).
- Video: mô phỏng chạy hết nhưng **dây tuột khỏi tay kẹp ở gap23_in** (khoảng cách dây–TCP 5,3 → 7,4 → 7,1 → 9,6 mm rồi mất), phích bị kéo lệch 1340 mm. **Chưa biết** tuột do V1 gốc sẵn có hay do nhánh IK mới (cần chạy V1 gốc để so).

**V2new** (từ V2) — làm hai lần:
- Lần 1: thêm `ik_candidates` để cứu kế hoạch bị bước kiểm tra chặn. Chạy đầy đủ, dây không rơi, mọi đoạn 4,1 mm. **Nhưng bản này không còn là "V2 chỉ thêm tránh tự va chạm"** (nó cũng bỏ luôn cú quay 286° của khớp đế), nên không dùng để kết luận về V2.
- Lần 2 (đúng yêu cầu: V2 giữ nguyên, chỉ thêm tránh tự va chạm). Đo ở bước nhặt dây:
  - IK nội bộ của cuRobo (lần hỏi đầu) cho nhánh khớp đế 294°; đoạn hạ thẳng xuống dây từ nhánh đó có khoảng hở **−19,4 mm**. Đây chính là hiện tượng "V2 cánh tay va chạm khi nhặt dây".
  - Khi thử lại bằng cách nhiễu trạng thái đầu 1e-3 rad, cuRobo trả về nhánh khớp đế ≈ 11° (hoặc −349°) với 4,1 mm. **Mọi nhánh không tự va chạm đều là nhánh không quay**; ở V2 "hết va chạm" và "hết quay thừa" là một thay đổi.
  - Nhưng với nhánh không quay, đoạn đặt dây cuối (`lay`) của V2 (hạ thẳng rồi lùi thẳng) làm cánh tay **chạm cột (−10,5 mm)** ở cả 10 nghiệm IK gần nhất, nên V2new không chạy trọn nhiệm vụ.
  - Theo lựa chọn của bạn, video V2new chỉ chạy **đến khi nhặt dây xong**: thêm `--stop-after-pickup` (mô phỏng) và `--stop-after-lift` (planner). Đoạn tiếp cận–hạ–nâng dây đều 4,1 mm.
  - **Chưa xác minh** dây có thật sự bị kẹp và nhấc lên (log V2 không có phép đo dây–TCP như V4).

## 3. Lệch má kẹp khi kẹp dây

### 3.1 Hiện tượng và cách đo
Thêm `jaw_state()` và dòng `[Jaw]` trong `ur5_regrasp_V5full.py`: góc hai khớp dẫn động, vị trí trung điểm hai miếng đệm so với TCP (hệ wrist_3), vị trí từng miếng đệm và dây (proxy), và các tiếp xúc của tay kẹp (`gripper_contacts`).

Để thử nhanh, thêm `--regrasp-only` (bỏ phần cắm phích, kẹp dây tại đúng vị trí của lần chạy thật: dây ở [0,294; 0,083; 0,009]) và `--stop-after-pickup`. Mỗi lần thử ≈ 3 phút thay vì ≈ 16 phút.

| Khi kẹp | Góc ngón R / L | Miếng đệm phải / trái | Lệch trung điểm |
|---|---|---|---|
| phích | 0,5441 / 0,5441 | +32,5 / −32,4 mm | **0,03 mm** (đối xứng) |
| dây, V4/V5full gốc | 0,776–0,779 / bằng nhau | +36,7 / −27,4 mm | **4,7–5,2 mm** |

Điểm quan trọng: **góc hai ngón bằng nhau** (chênh 0,0000 rad) mà hai miếng đệm vẫn không đối xứng quanh dây. Dấu của độ lệch đổi giữa các lần chạy (±5 mm), tức là "lệch sang một bên" nhưng bên nào thì thay đổi.

### 3.2 Giả thuyết ban đầu của tôi và cách nó bị bác
Tôi nghĩ hai ngón bị điều khiển bởi hai PD độc lập nên mất đồng bộ. Mô hình MuJoCo có đúng **8 actuator**: 6 khớp UR5 và 2 khớp dẫn động (kp = 20, kd = 1 mỗi bên); actuator tendon `split` gốc của Robotiq đã bị bỏ khi nhập. Vậy cấu trúc "hai PD" là thật. Nhưng góc hai ngón vẫn bằng nhau (nhờ ràng buộc equality `driver_R = driver_L` của Robotiq), nên hai PD không làm mất đồng bộ.

### 3.3 Thí nghiệm (mỗi thí nghiệm một lần chạy `--regrasp-only`)
| Thí nghiệm | Lệch trung điểm | Kết luận |
|---|---|---|
| Gốc | 4,9–5,1 mm | lệch |
| Đóng 0,79 thay 0,8 | 4,9–5,1 mm | không đổi |
| Lực kẹp dây 0,8 N·m thay 2,2 | 5,1–5,2 mm | không đổi |
| **Một PD cho cả hai má** (`GRIP_SINGLE_PD=1`: ngón phải ke/kd/mô-men gấp đôi, ngón trái ke = kd = 0, bị kéo theo bởi equality) | 4,9–5,1 mm | không đổi |
| Nâng điểm kẹp 3 mm | 6,4–6,6 mm | vẫn lệch |
| Nâng 5 mm | 0,1 mm | đối xứng |
| Nâng 7 mm | 0,03 mm | đối xứng |
| Nâng 10 mm | 0,00 mm | đối xứng |

Cách sửa chọn: `RG_Z_OFF` mặc định **0,007 m** trong `plan_regrasp` (nâng điểm kẹp dây; cũng nâng độ cao đặt dây lên 7 mm vì `z_place = p_g[2]`).

### 3.4 Các giả thuyết đã bị loại trừ bằng đo đạc
1. **Hai PD mất đồng bộ**: sai (3.2, 3.3).
2. **Độ đóng/lực kẹp**: sai (3.3).
3. **Đầu ngón chạm sàn**: tôi đã nghi nhiều lần và đã sai. Đo các tiếp xúc lúc kẹp dây:
   - độ cao cũ (lệch): chỉ miếng trái chạm sàn 3,8 N và chỉ ở pha giữ;
   - nâng 7 mm (đối xứng): **cả hai miếng chạm sàn** 9–12 N.
   Ca đối xứng còn chạm sàn nhiều hơn, nên chạm sàn không gây lệch.
4. **Dây biến dạng sau khi cắm nên TCP dây cao hơn TCP phích, nhưng code vẫn dùng TCP cũ** (giả thuyết của bạn): sai, vì (a) `plan_regrasp` tính `c` và `p_g = c + 12 mm` từ `cable_points()` là vị trí dây **hiện tại** sau khi cắm; (b) dây chỉ cao hơn lúc nghỉ 4 mm (z 0,009 so với 0,005), con số này đã nằm trong `p_g` (TCP kẹp dây 21 mm, kẹp phích 23,5 mm); (c) chế độ `--regrasp-only` không cắm phích, dây không hề biến dạng, vẫn lệch 5 mm.

### 3.5 Phỏng đoán của tôi (CHƯA kiểm chứng)
Chỉ có **tương quan** đã đo:
| | Độ cao cũ (lệch) | Nâng 7 mm (đối xứng) |
|---|---|---|
| Vị trí dây trong hệ wrist_3 | y ≈ −1 mm (giữa miếng đệm), z ≈ −5 mm | y ≈ +12 mm (gần đầu ngón), z ≈ −16 mm |
| Tiếp xúc miếng đệm–dây | trái **4 điểm**, phải **1 điểm**, 10,3 N | trái 2, phải 2, **28,6 N** |
| Góc ngón khi dừng | 0,778 rad, hai miếng đệm cách nhau ≈ 64 mm | 0,709 rad, cách nhau ≈ 23,5 mm |

Phỏng đoán: ở độ cao cũ, dây nằm ở vùng giữa miếng đệm, nơi tiếp xúc giữa capsule dây và hình học miếng đệm Robotiq không cân bằng (một bên tiếp xúc mặt, một bên chỉ tựa một cạnh). Khi đó cơ cấu bốn thanh có khớp lò xo thụ động (`spring_link`) và ràng buộc mềm (`solref 0,005`) bị đẩy lệch ngang dưới lực không đối xứng; góc ngón lái vẫn bằng nhau nhưng các khâu bị đẩy khác nhau. Ở vùng gần đầu ngón, tiếp xúc đều hai bên nên cặp má đối xứng. **Dấu hiệu ủng hộ**: miếng đệm cách nhau tới 64 mm dù ngón đã khép sâu hơn (0,778 so với 0,709 rad), tức cơ cấu đang biến dạng bất thường. **Chưa có**: chưa xem hình học va chạm của miếng đệm, chưa thử cứng hóa `spring_link`, chưa thử đổi vị trí dây dọc theo miếng đệm một cách độc lập với độ cao.

Cách kiểm chứng đề xuất: (a) in hình học và pháp tuyến tiếp xúc của hai miếng đệm lúc kẹp; (b) tăng độ cứng `spring_link` hoặc siết `solref` của equality và xem lệch có giảm không; (c) giữ nguyên độ cao nhưng đổi vị trí dây dọc theo trục công cụ.

### 3.6 Kết quả V5full (một lần chạy đầy đủ, `--test`, 972 s thực / 72 s mô phỏng)
| Chỉ số | Giá trị |
|---|---|
| Lệch trung điểm má khi kẹp dây | **0,03 mm** (miếng đệm +11,77 / −11,72 mm) |
| Khoảng hở tự va chạm | 4,1 mm ở mọi đoạn |
| Dây rơi | không (`cable dropped: False`) |
| Khoảng cách dây–TCP khi kẹp | 19,6–20,2 mm (V4: 5–10 mm) |
| Phích dịch tối đa trong lúc re-grasp | 0,95 mm |
| Phích dịch sau khi đặt dây | **0,51 mm** |
| Hở vỏ–mặt ổ | 0,87 mm (yêu cầu < 9 mm) |

## 4. Sai sót của tôi trong ngày (để bạn biết khi đọc lại)
- Nói V4 tránh tự va chạm "có thể do may" khi chưa đo → sai, đã đo lại (mục 2.1).
- Khẳng định "nhiều khả năng đầu ngón chạm sàn" gây lệch má → sai (mục 3.4). Comment trong code của `RG_Z_OFF` ban đầu ghi nguyên nhân chạm sàn và đã được sửa lại thành mô tả đúng (chỉ nêu tương quan).
- Một thử nghiệm 4 phút bị lãng phí vì bản vá `--regrasp-only` không áp dụng được (chuỗi tìm không khớp), phải chạy lại.

## 5. Hạn chế và việc tiếp theo
1. **Nguyên nhân lệch má chưa rõ** (3.5); cách sửa là thực nghiệm và có thể không giữ được nếu đổi tham số kẹp.
2. Dây nằm lệch ≈ 16 mm so với tâm miếng đệm theo bề rộng (wrist z ≈ −16 mm); hiện không tuột nhưng đây là điểm yếu nếu đổi lực kẹp/tốc độ.
3. V1new: cần chạy V1 gốc để biết dây tuột ở gap23_in có sẵn không.
4. V2: nhánh không quay làm đoạn đặt dây chạm cột (−10,5 mm); cần đổi cách đặt/lùi dây (như V1) mới chạy trọn được, nhưng khi đó V2new không còn là "chỉ thêm tránh tự va chạm".
5. Tất cả kết quả từ một lần chạy; cần chạy lặp để đánh giá độ ổn định.
6. Mô phỏng vẫn tắt `enable_self_collisions` (chủ ý, tránh nhiễu tiếp xúc ở khớp); an toàn tự va chạm hiện dựa vào mô hình cầu. Cần so với lưới (mesh) ở vài tư thế biên.
7. Giá trị `SELF_MARGIN = 2 mm` chọn thủ công; cặp forearm–wrist_2 luôn sát (4,1 mm) nên đây là ngưỡng nhạy.

## 6. Cách chạy lại
```
cd data/scripts
python ur5_regrasp_V5full.py --test                      # chuỗi đầy đủ V5full
python ur5_regrasp_render_isaac_V5full.py --stride 3     # video
python ur5_regrasp_V5full.py --regrasp-only --stop-after-pickup   # thử nhanh má kẹp (≈3 phút), đọc dòng [Jaw]
RG_Z_OFF=0.0 python ur5_regrasp_V5full.py --regrasp-only --stop-after-pickup   # tái tạo lỗi lệch
GRIP_SINGLE_PD=1 python ...                              # thử một PD cho cả hai má
python ur5_regrasp_V2new.py --stop-after-pickup          # V2new tới lúc nhặt dây
```
