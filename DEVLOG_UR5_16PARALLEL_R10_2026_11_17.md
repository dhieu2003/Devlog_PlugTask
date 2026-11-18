# DEVLOG ur5_16parallel — vòng 10 (r10), 2026-11-17

Kết quả: `data/results/ur5_16parallel_r10/` (summary.json, từng kịch bản trong thư mục riêng, `run.log` + `result.json`).
Video bảng 4x4 (cột: dáng dây | vị trí cột | vị trí ổ | cả ba): `data/results/video_ur5_16parallel_r10.mp4`.
Các vòng cũ (r1–r9) không bị ghi đè. Code mới nằm trong các file `ur5_16parallel*.py` (V6 không sửa).

## 1. Kết quả tổng

**9/16 thành công (56,25%)** — dáng dây 2/4, vị trí cột 2/4, vị trí ổ 3/4, cả ba 2/4.
(Tiêu chí thành công: hoàn thành chuỗi, không lỗi số (NaN/vận tốc dây > 5 m/s), dây không rơi, khe hở phích–ổ < 9 mm.)

| Kịch bản | Kết quả | Khe hở (mm) | Nguyên nhân thất bại |
|---|---|---|---|
| shape1 | FAIL | – | dây tuột khỏi tay kẹp sau cột 1 (proxy cách TCP 366 mm) |
| shape2 | OK | 0,89 | |
| shape3 | FAIL | – | dây tuột sau cột 1 (243 mm); ứng viên 1 trước đó cuRobo không lập được kế hoạch tiếp cận |
| shape4 | OK | 0,92 | |
| posts1 | OK | 0,91 | |
| posts2 | OK | 0,87 | |
| posts3 | FAIL | 1,32 (nhưng dây rơi) | dây tuột khi hạ tay kẹp xuống ('lower'), max khoảng cách dây–TCP 69 mm |
| posts4 | FAIL | – | không còn điểm kẹp dây ứng viên nào (5 ứng viên đều thất bại ở cuRobo, vd. kế hoạch về Home) |
| socket1 | OK | 0,73 | |
| socket2 | OK | 0,84 | |
| socket3 | FAIL | – | dây tuột khi hạ phích ('lower', 225 mm) |
| socket4 | OK | 0,82 | |
| all1 | OK | 0,87 | |
| all2 | FAIL | – | cuRobo không lập được kế hoạch ở đoạn sóng 6 (-> lay) với mọi nhánh IK; 9 ứng viên đều thất bại |
| all3 | OK | 3,61 | |
| all4 | FAIL | – | 7 ứng viên đều thất bại: cuRobo không lập được kế hoạch (tiếp cận / đoạn sóng 4 -> gap23_in) |

Tham số chạy chung: `--vbd-iterations 50 --cable-grip-effort 3.2 --ground-mu 0.3`; 16 tiến trình độc lập, khởi động cách nhau 60 s (GPU 10 GB không chịu nổi nhiều cuRobo cùng lúc); seed 2025–2040.

## 2. Các kiểu ngẫu nhiên và bảng tham số

Hệ toạ độ: gốc ở đế robot, x hướng ra phía cột, y sang trái; cột cao 0,30 m; tường cắm ổ ở x = cột 3 + 0,08.
Quy ước: mỗi nhóm chỉ đổi tham số của nhóm đó; các tham số còn lại giữ cơ sở (cột (0,35;0), (0,60;0), (0,85;0); ổ (0,20; 0,27); dây thẳng).

### 2.1 Dáng dây (nhóm shape, 4 ca)
Dây dài 1,6 m, 65 điểm, đường cong ngẫu nhiên (độ cong ≤ 14 rad/m) nằm trong hộp x −0,05…0,80, y 0,17…0,80 (trong khung camera); đoạn giao cắt được nâng 1,1 cm. Đầu dây (phích) luôn ở (0,20; 0,26).

| Ca | Số điểm giao cắt | x dây (m) | y dây (m) | Đầu tự do |
|---|---|---|---|---|
| shape1 | 6 | 0,05…0,34 | 0,26…0,74 | (0,08; 0,38) |
| shape2 | 13 | −0,03…0,20 | 0,24…0,75 | (−0,02; 0,37) |
| shape3 | 4 | −0,03…0,25 | 0,26…0,75 | (0,08; 0,69) |
| shape4 | 13 | −0,01…0,20 | 0,26…0,73 | (0,03; 0,43) |

### 2.2 Vị trí cột (nhóm posts, 4 ca) — theo `instructions/guide_16parallel.md`
Dây thẳng, ổ cơ sở. Toạ độ (x, y) cột 1 / 2 / 3:

| Ca | Cột 1 | Cột 2 | Cột 3 | Ghi chú |
|---|---|---|---|---|
| posts1 | (0,35; 0) | (0,60; 0) | (0,85; 0) | đúng guide |
| posts2 | (0,35; 0) | (0,60; **0,04**) | (0,85; 0) | guide: y2 = 0,15 → chỉnh xuống 0,04 |
| posts3 | (0,35; 0) | (0,60; −0,15) | (0,85; 0) | đúng guide |
| posts4 | (0,35; 0,15) | (0,60; **0,04**) | (0,85; 0) | guide: y2 = 0,15 → chỉnh xuống 0,04 |

Lý do chỉnh posts2/posts4 (theo yêu cầu của bạn: giữ nguyên ổ, chỉnh toạ độ cột): với y2 = 0,15, cuRobo không lập được kế hoạch `weave leg 6.0 (-> post3)` (kết quả cũ lưu ở `posts2_failed_plan_orig_coords`, `posts4_failed_plan_orig_coords`). Quét y2 ∈ {0,04; 0,06; 0,08; 0,10; 0,12; 0,15}: chỉ y2 ≤ 0,04 lập được. Số đo: khoảng hở giữa quả cầu bao tay kẹp (z = 0,36 m, cao hơn đỉnh cột 0,30 m) và cột 2 tại điểm ra khe 2–3 giảm từ 49,7 mm (y2 = 0) → 36,7 (0,04) → 31,0 (0,06) → 25,8 (0,08) → 13,3 mm (0,15). Ngưỡng ~31–37 mm là suy ra từ quét thực nghiệm, chưa đọc giá trị biên va chạm trong mã cuRobo; hàm khoảng hở là bản xấp xỉ numpy của mình, không phải mô hình va chạm của cuRobo.

### 2.3 Vị trí ổ (nhóm socket, 4 ca)
Dây thẳng, cột cơ sở. Ổ trên tường, ở **bên trái cột 3** (ổ bên phải bỏ qua theo yêu cầu). Toạ độ (y, z) của ổ trên tường (`socket_yz`):

| Ca | y (m) | z (m) |
|---|---|---|
| socket1 | 0,154 | 0,280 |
| socket2 | 0,149 | 0,284 |
| socket3 | 0,167 | 0,284 |
| socket4 | 0,164 | 0,279 |

(Cơ sở: 0,20; 0,27.)

### 2.4 Cả ba thay đổi đồng thời (nhóm all, 4 ca)
Cột theo đúng guide (không chỉnh), dáng dây ngẫu nhiên, ổ ngẫu nhiên:

| Ca | Cột 2 (x; y) | Cột 1 | Ổ (y; z) | Giao cắt dây |
|---|---|---|---|---|
| all1 | (0,60; 0) | (0,35; 0) | 0,197; 0,260 | 2 |
| all2 | (0,60; **0,15**) | (0,35; 0) | 0,207; 0,284 | 5 |
| all3 | (0,60; −0,15) | (0,35; 0) | 0,174; 0,263 | 5 |
| all4 | (0,60; **0,15**) và cột 1 (0,35; **0,15**) | | 0,194; 0,277 | 4 |

(Cột 3 luôn ở (0,85; 0).)

## 3. Phân tích

- **Ổ (3/4)** cao nhất: ổ chỉ dịch vài cm quanh vị trí cơ sở nên quỹ đạo gần như không đổi. Ca duy nhất hỏng (socket3, y = 0,167) hỏng vì dây tuột khi hạ phích, không phải do ổ không với tới. Trước đó (r9) ca này bị nổ dây; sau khi sửa (ngắt sớm khi dây tuột > 10 cm, VBD 50 vòng, kẹp dây 3,2 N·m, ma sát nền 0,3) không còn nổ nhưng vẫn tuột.
- **Cột (2/4)**: tỷ lệ phụ thuộc cấu hình. Cột 2 lệch +Y quá nhiều → tay kẹp sát đầu cột tại điểm ra khe 2–3 → cuRobo thất bại (đã xử lý bằng y2 = 0,04). Cột 2 lệch −Y (posts3) lập được kế hoạch nhưng dây tuột khi hạ.
- **Dáng dây (2/4)**: dây nhiều giao cắt (13) lại thành công, dây ít giao cắt (4–6) thất bại; chưa đủ dữ liệu (n = 4) để kết luận quan hệ. Điểm chung hai ca hỏng là dây tuột ngay sau cột 1, tức dây trượt khi đoạn đầu dây kéo qua cột.
- **Cả ba (2/4)**: all1, all3 thành công; all2, all4 dùng nguyên cột trường hợp 2/4 của guide (y2 = 0,15) và cuRobo thất bại — nhất quán với kết luận ở mục 2.2. all3 có khe hở lớn nhất (3,6 mm) nhưng vẫn < 9 mm.

## 4. Giải thích nguyên nhân thất bại

Có ba cơ chế, phân theo nơi xảy ra:

1. **Dây tuột khỏi tay kẹp (shape1, shape3, posts3, socket3 — 4/7 ca hỏng).** Tay kẹp song song (k = 0,2) giữ dây bằng ma sát nên khi dây bị cột cản hoặc tay kẹp hạ xuống mà dây còn bị kéo, dây trượt. Sim phát hiện khi điểm dây gần TCP cách TCP > 10 cm và dừng sớm (`cable slipped`). Nếu không dừng, dây tuột sẽ vướng cột, giãn (1,57 → 1,9–2,2 m) và hệ ghép Newton–VBD nổ NaN (đã quan sát ở các vòng trước). Chưa chứng minh được lực kẹp 3,2 N·m (56 N) là đủ; tăng lên 4,5 N·m không cải thiện ở thử nghiệm ổ phải.
2. **cuRobo không lập được kế hoạch (all2, all4, và cả posts4 một phần).** Chủ yếu do cột 2 lệch +Y (y2 = 0,15): tay kẹp ở trên đầu cột 2 (z 0,36 > 0,30) quá gần cột tại điểm ra khe 2–3 hoặc ở các đoạn đi/về tiếp theo. all2 hỏng ở đoạn 6 (-> lay), all4 ở tiếp cận và đoạn 4 (-> gap23_in).
3. **Không còn điểm kẹp dây (posts4).** Dây thẳng nên chỉ có vài điểm thoả điều kiện tầm với/khoảng cách tới cột (5 ứng viên); mỗi ứng viên lại hỏng ở cuRobo (kế hoạch về Home), nên hết ứng viên. Lỗi gốc ở đây cũng là cuRobo, không phải thiếu dây.

Không ca nào thất bại vì nổ số (`numerical_error` = false cho cả 16 ca) hay vì ổ ngoài tầm với 85 cm.

## 5. Việc còn mở
- Ổ bên phải cột 3: cuRobo lập được với yaw (60°, 60°) nhưng phích xoay 36–43° trong tay kẹp khi vẫy (khoảng nửa do lực dây, ~13,7° do trượt cơ khí) → lỗi IK khi cắm. Đang tạm hoãn theo yêu cầu.
- all2, all4 dùng cột y2 = 0,15 của guide; chưa chạy lại với y2 = 0,04 (chờ quyết định).
- Mỗi kịch bản chạy một lần; cuRobo và VBD có tính ngẫu nhiên nên chưa biết độ phân tán.
- Chưa có DEVLOG riêng cho r3–r9 (vòng 1–2 ở `DEVLOG_UR5_16PARALLEL_2026_11_14.md`).
