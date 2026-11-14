# DEVLOG 2026-11-11 → 2026-11-14 – ur5_regrasp: chống tự va chạm, lệch má kẹp khi gắp dây, và V6

Tài liệu này gộp công việc của bốn ngày 11, 12, 13, 14/11/2026. Với mỗi lỗi ghi rõ: **hiện tượng → nguyên nhân (đã đo / chỉ là phỏng đoán) → giải pháp → kết quả**. Phần "đã đo" là số liệu đọc từ log mô phỏng; phần "phỏng đoán" chưa được chứng minh.

## 0. Tóm tắt

| Lỗi | Nguyên nhân | Giải pháp | Kết quả |
|---|---|---|---|
| **A. Cánh tay tự va chạm** (V1, V2; V4 chỉ "không va chạm" nhờ nghiệm phù hợp) | Chỉ cuRobo tránh tự va chạm ở các đoạn nó lập kế hoạch; các đoạn thẳng IK, chọn nhánh IK và mô phỏng (`enable_self_collisions=False`) không kiểm tra gì | Hàm `self_clearance` (cầu va chạm của `ur5.yml`) để lọc nghiệm IK và đường thẳng, ngưỡng 2 mm | Mọi đoạn của V5full/V1new/V2new/V6: khoảng hở ≥ 4,1 mm (giá trị cấu trúc) |
| **B. Má kẹp lệch khi gắp dây** (V1, V4) | Cơ cấu bốn thanh **của một ngón bị gập** khi lò xo `spring_link` quá yếu (k ≤ 0,125 N·m/rad) và má chỉ chạm dây ở 1 điểm | Chế độ kẹp theo spec 2.1.8: gắp dây dùng **Parallel (k = 0,2)**, gắp phích **Encompassing (k = 0,05)** | V6: lệch má **0,01–0,02 mm** (V4: ≈ 5 mm), điểm kẹp dây giữ như V4 (không nâng 7 mm như V5full) |

Kết quả V6 đầy đủ (một lần chạy, 972 s thực / 72,2 s mô phỏng): cắm phích lệch trục 0,111 mm (giới hạn 0,2 mm), nghiêng 0,414°; dây không rơi; phích sau khi đặt dây dịch 0,620 mm so với vị trí cắm.

**Điều cần biết trước khi dùng kết quả**
- Mọi kết luận về V6 đến từ **một lần chạy đầy đủ** (và một lần chạy lại sau khi làm sạch mã nguồn cho cùng số liệu). Chưa chạy lặp để đo độ ổn định.
- Cơ chế "ngón bị gập" được chứng minh bằng số đo góc khớp và thí nghiệm nới giới hạn (mục 3.6), **chưa có phân tích cơ học** cho điều kiện ổn định của cơ cấu bốn thanh.
- Lực kẹp mỗi ngón F dùng để chọn chế độ là **ước tính** từ mô-men giới hạn (mục 4.2), không phải đo trực tiếp.
- Giá trị m = 2,5, S = 2,5, k = 0,2 và k = 0,05 trong spec 2.1.8 do người dùng đặt; tôi chỉ kiểm chứng chúng hoạt động.

## 1. Ngày 11/11 – chống tự va chạm (V5full, V1new, V2new)

### 1.1 Hiện tượng
Ở V1 cánh tay đâm xuyên chính nó; ở V4 không thấy. Trong mô phỏng `enable_self_collisions=False` nên vật lý không báo gì, chỉ thấy khi xem video.

### 1.2 Nguyên nhân
Ở V4, tự va chạm chỉ được xử lý bởi **cuRobo** trên các đoạn nó lập kế hoạch (nó có chi phí tự va chạm dùng các cầu trong `ur5.yml`, `self_collision_buffer = 0`). Không có gì kiểm tra:
- các đoạn thẳng IK nối tiếp (`line_segment`, `pose_line`, `joint_line`);
- việc chọn nhánh IK (`ik_near`, `ik_candidates`) và IK nội bộ của cuRobo ở `plan_pose`;
- mô phỏng vật lý.

Tôi từng nói "V4 không va chạm có thể do may". **Đã đo lại**: trên 1782 cấu hình lấy từ các kế hoạch V4, khoảng hở nhỏ nhất là 4,1 mm (cặp forearm–wrist_2, bằng giá trị ở tư thế Home), nên V4 không va chạm thật nhưng không có cơ chế bảo đảm.

### 1.3 Giải pháp (trong `ur5_regrasp_geom_V6.py` và `ur5_regrasp_curobo_V6.py`)
Với mỗi cặp khâu (a, b) trong danh sách `SELF_PAIRS` (9 cặp; 6 cặp kề nhau trong `SELF_IGNORE_PAIRS` bị bỏ qua, sao chép từ `ur5_build_assets_V4.py` để cuRobo và bộ kiểm tra dùng cùng quy tắc), khoảng hở giữa các cầu va chạm:

$$ d_{ab}(q)=\min_{i\in a,\,j\in b}\Big(\lVert c_i(q)-c_j(q)\rVert-r_i-r_j\Big),\qquad d_{\min}(q)=\min_{(a,b)\in \text{SELF\_PAIRS}} d_{ab}(q) $$

Một cấu hình được chấp nhận khi $d_{\min}(q)\ge \delta$ với $\delta=$ `SELF_MARGIN` = 2 mm (giá trị cấu trúc 4,1 mm nên 2 mm loại được tư thế gập thật mà không chặn tư thế bình thường). Một đường khớp $\{q_k\}$ được chấp nhận khi $\min_k d_{\min}(q_k)\ge\delta$ (`path_self_clearance`).

Chỗ gắn vào planner: `ik_near`/`ik_candidates` loại nghiệm không đạt; `pose_line`/`joint_line` trả `None` nếu đường vi phạm; kiểm tra cuối cho mọi đoạn in `self-collision clearance` và `raise RuntimeError` nếu < 0.

**Kiểm chứng ca âm tính**: tư thế gập `[0, −0,1, −3,1, −1,5, 0, 0]` cho −68 mm, `[0, −2,9, −3,0, 0, 0, 0]` cho −40 mm: hàm bắt được va chạm thật.

### 1.4 Kết quả
| Bản | Kết quả |
|---|---|
| V5full (từ V4full) | mọi đoạn 4,1 mm |
| V1new (từ V1) | wave1, wave2, wave3 trước đây −64,1 / −64,1 / −10,3 mm → 4,1 mm; **dây tuột ở gap23_in** (chưa rõ do V1 gốc hay do nhánh IK mới) |
| V2new (từ V2) | cuRobo trả nhánh khớp đế 294° và đoạn hạ thẳng xuống dây có khoảng hở −19,4 mm (đúng hiện tượng "V2 va chạm khi nhặt dây"); mọi nhánh không va chạm đều là nhánh không quay; đoạn đặt dây `lay` chạm cột (−10,5 mm) nên V2new chỉ chạy **đến lúc nhặt dây** |
| V6 | 4,1 mm ở mọi đoạn |

## 2. Ngày 11–12/11 – điều tra má kẹp lệch khi gắp dây

### 2.1 Hiện tượng (đã đo)
Khi gắp dây, **góc hai driver bằng nhau** (chênh 0,0000 rad) nhưng hai miếng đệm không đối xứng quanh dây. Chỉ số lệch:

$$ \mathbf{o}=R_{w}^{\top}\Big(\tfrac12(p_R+p_L)-p_w\Big)-\mathbf t_{\text{TCP}},\qquad \text{lệch má}=o_x $$

($p_R, p_L$ = vị trí hai miếng đệm, $p_w$ và $R_w$ = vị trí và hướng wrist_3, $\mathbf t_{\text{TCP}}$ = offset TCP trong hệ wrist_3). Kẹp phích: $o_x=0{,}03$ mm. Kẹp dây ở V4: $o_x\approx 4{,}7$–$5{,}2$ mm (miếng đệm +36,7 / −27,4 mm); **bên lệch thay đổi giữa các lần chạy**.

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

$$ \tau_s = k\,(q_{\text{ref}}-q_s),\quad q_{\text{ref}}=2{,}62\ \text{rad} $$

Tại $q_s=0{,}4$ rad: $k=0{,}05\Rightarrow \tau_s\approx0{,}11$ N·m; $k=0{,}2\Rightarrow \tau_s\approx0{,}44$ N·m (mô-men giới hạn driver 3,2 N·m). Follower có giới hạn ±0,872664 rad.

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
Mỗi ngón tạo một tiếp xúc ma sát, nên để vật khối lượng $m$ không trượt khi gia tốc $a$:

$$ 2\mu F \ge m\,(g+a)\ \Rightarrow\ F_{sw}=\frac{m\,(g+a)}{2\mu}\,S $$

Với $m=2{,}5$, $g=9{,}81$, $a=0{,}8$ m/s² (`--a-carry`), $\mu=0{,}7$ (`PAD_MU`, cao su/PVC), $S=2{,}5$:

$$ F_{sw}=\frac{2{,}5\,(9{,}81+0{,}8)}{2\cdot0{,}7}\cdot2{,}5=47{,}37\ \text{N} $$

- $F<F_{sw}$: **Parallel Grasp**, lò xo spring_link k = 0,2 N·m/rad.
- $F\ge F_{sw}$: **Encompassing Grasp**, k = 0,05 N·m/rad.

### 4.2 Lực kẹp F mỗi ngón
Không đo trực tiếp lúc quyết định; ước tính từ mô-men giới hạn driver, hiệu chuẩn bằng phép đo kẹp phích (56,4 N ở 3,2 N·m):

$$ F = 17{,}6\ \text{N/(N·m)}\cdot\tau_{\max}\ \Rightarrow\ \text{phích: }56{,}4\ \text{N},\quad \text{dây (2,2 N·m): }38{,}8\ \text{N} $$

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
- Nói V4 tránh tự va chạm "có thể do may" khi chưa đo → sai (mục 1.2).
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
7. Mô phỏng vẫn tắt `enable_self_collisions`; an toàn tự va chạm dựa vào mô hình cầu, chưa so với lưới ở tư thế biên.

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
