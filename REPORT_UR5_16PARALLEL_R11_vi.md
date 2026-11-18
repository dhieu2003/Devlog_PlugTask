# Nhiệm vụ regrasp UR5 + Robotiq 2F-85 trên 16 thế giới song song — báo cáo vòng 11 (r11)

Kết quả: `data/results/ur5_16parallel_r11/` (thư mục từng kịch bản, `summary.json`); video bảng 4x4: `data/results/video_ur5_16parallel_r11.mp4`
(cột: dáng dây | vị trí cột | vị trí ổ | cả ba thông số; mỗi hàng một lần thử).

## 1. Tóm tắt
**Thành công 9/16 ca (56,25 %).** Thành công = hoàn thành chuỗi, phích ở trong ổ (khe hở < 9 mm), dây không rơi và mô phỏng dây không bị nổ số (không NaN, vận tốc dây < 5 m/s).

| Nhóm | Thành công | Ca thành công | Ca thất bại (lý do) |
|---|---|---|---|
| Dáng dây | 2/4 | shape2, shape3 | shape1 (dây tuột khỏi tay kẹp sau khe 1-2), shape4 (tuột sau cột 2) |
| Vị trí cột | 2/4 | posts1, posts2 | posts3 (tuột khi hạ tay), posts4 (hết điểm kẹp dây: cuRobo không lập được kế hoạch cho mọi ứng viên) |
| Vị trí ổ | 2/4 | socket1, socket4 | socket2 (tuột sau cột 1), socket3 (ổ bên phải: phích đã cắm được nhưng cuRobo không lập được kế hoạch tiếp cận kẹp lại dây) |
| Cả ba | 3/4 | all1, all2, all3 | all4 (tuột sau cột 1) |

Khe hở phích–ổ của các ca thành công: 0,67–1,04 mm. Không ca nào kết thúc vì lỗi số.

## 2. Các kiểu thay đổi và bảng thông số
Bố trí chung: 3 cột (bán kính 5 cm, cao 30 cm), tường sau cột 3 là 8 cm, dây dài 1,6 m, phích nhặt ở (0,20; 0,26). Thông số không nêu của một nhóm giữ ở giá trị cơ sở (cột (0,35;0), (0,60;0), (0,85;0); ổ (y, z) = (0,20; 0,27); dây thẳng).

### 2.1 Dáng dây (nhóm shape; 4 seed ngẫu nhiên 2025–2028)
Đường cong trơn ngẫu nhiên trong hộp của camera (x −0,05…0,80; y 0,17…0,80), độ cong ≤ 14 rad/m, đoạn giao cắt nâng 1,1 cm.

| Ca | Số giao cắt | x [m] | y [m] | Đầu tự do (x, y) |
|---|---|---|---|---|
| shape1 | 6 | 0,05…0,34 | 0,26…0,74 | (0,08; 0,38) |
| shape2 | 13 | −0,03…0,20 | 0,24…0,75 | (−0,02; 0,37) |
| shape3 | 4 | −0,03…0,25 | 0,26…0,75 | (0,08; 0,69) |
| shape4 | 13 | −0,01…0,20 | 0,26…0,73 | (0,03; 0,43) |

### 2.2 Vị trí cột (nhóm posts; dây thẳng, ổ cơ sở) — theo `instructions/guide_16parallel.md`
| Ca | Cột 1 (x, y) | Cột 2 (x, y) | Cột 3 (x, y) | Ghi chú |
|---|---|---|---|---|
| posts1 | (0,35; 0) | (0,60; 0) | (0,85; 0) | đúng guide |
| posts2 | (0,35; 0) | (0,60; **0,04**) | (0,85; 0) | guide: y2 = 0,15; chỉnh xuống 0,04 |
| posts3 | (0,35; 0) | (0,60; −0,15) | (0,85; 0) | đúng guide |
| posts4 | (0,35; 0,15) | (0,60; **0,04**) | (0,85; 0) | guide: y2 = 0,15; chỉnh xuống 0,04 |

Lý do chỉnh y2 (người dùng cho phép ở ca 2 và 4): với y2 = 0,15 cuRobo không lập được đoạn sau khe 2–3, vì quả cầu va chạm của tay kẹp nằm trên đầu cột (z = 0,36 m so với cột cao 0,30 m) chỉ còn hở 13 mm so với cột 2. Khoảng hở theo y2: 49,7 mm (0), 36,7 (0,04), 31,0 (0,06), 25,8 (0,08), 13,3 (0,15). Lập được kế hoạch với y2 ≤ 0,04, thất bại với y2 ≥ 0,06; ngưỡng 31–37 mm là suy ra từ phép quét này, không đọc từ mã cuRobo.

### 2.3 Vị trí ổ (nhóm socket; dây thẳng, cột cơ sở)
Tâm ổ trên tường (y, z). r10 chỉ đổi z trong 0,279–0,284; r11 đổi z trong khoảng 0,15 m và có một ổ bên phải cột 3.

| Ca | y ổ [m] | z ổ [m] | Phía so với cột 3 |
|---|---|---|---|
| socket1 | 0,12 | 0,33 | trái |
| socket2 | 0,28 | 0,24 | trái |
| socket3 | **−0,15** | 0,21 | **phải** |
| socket4 | 0,20 | 0,36 | trái |

Độ cao luồn dây giờ tách khỏi độ cao ổ (`weave_z` = 0,28 m, thấp hơn đỉnh cột), nên phích chỉ nâng/hạ ở đoạn cuối.

### 2.4 Đổi đồng thời cả ba (nhóm all)
Dây từ seed 2029–2032 (ngẫu nhiên), cột theo guide (y2 = 0,04 ở ca 2 và 4 như mục 2.2 — thay đổi này làm cho nhóm all chưa được người dùng duyệt riêng), ổ chọn trong các vị trí cuRobo lập được kế hoạch.

| Ca | Cột 2 (x, y) | Cột 1 (x, y) | Ổ (y, z) | Giao cắt dây | Đầu tự do |
|---|---|---|---|---|---|
| all1 | (0,60; 0) | (0,35; 0) | (0,16; 0,30) | 2 | (0,18; 0,41) |
| all2 | (0,60; 0,04) | (0,35; 0) | (0,26; 0,26) | 5 | (0,03; 0,44) |
| all3 | (0,60; −0,15) | (0,35; 0) | (0,14; 0,34) | 5 | (0,12; 0,38) |
| all4 | (0,60; 0,04) | (0,35; 0,15) | (0,22; 0,30) | 4 | (0,01; 0,52) |

Cột 3 luôn ở (0,85; 0). Biên độ z của ổ trong nhóm này chỉ 0,08 m: ổ có z ≤ 0,25 không lập được kế hoạch khi cột 2 ở y = 0,04 (đã thử 0,22; 0,24; 0,25).

## 3. Vì sao ổ bên phải khó hơn, và cách sửa
Chạy chẩn đoán (ổ trái và phải, có và không dây, tới hết giai đoạn vận chuyển):
- Với ổ phải, phích phải tới cột 1 ở y = −0,2, xa tư thế nâng (y = +0,26). Hàm IK chọn nhánh có chuẩn khớp nhỏ nhất, là nhánh "vắt qua lưng" (vai 155°, khớp nâng −184°, cổ tay −251°).
- Đoạn kế tiếp phải quay vai 210° và cổ tay 205° trong 8 s (ổ trái: 2 s). Phích trượt xuống 2–4 cm trong má kẹp (độ lệch z phích–TCP 1,2 → 3,6–5 cm) và nghiêng 14° (không dây) / 35° (có dây), dẫn tới `insertion IK failed`.
- Cách sửa: `UR5P_NATURAL=1` loại các nhánh IK có khớp nâng vai ngoài (−180°, 0°). Vai chỉ quay 19°, cổ tay 3°, độ nghiêng phích tại ổ còn 1,5°. Ở r11 socket3 cắm được phích (lệch 1,7 mm sau khi nhả).
- Phần đóng góp của lực dây vào độ nghiêng (14 → 35°) suy ra từ so sánh có/không dây trên cùng quỹ đạo, không đo riêng.

## 4. Vì sao các ca thất bại
1. **Dây tuột khỏi tay kẹp (shape1, shape4, posts3, socket2, all4: 5 ca).** Má kẹp song song giữ dây bằng ma sát; khi dây bị cột cản hoặc bị kéo lúc hạ tay, dây trượt. Mô phỏng dừng khi điểm dây gần tay kẹp cách TCP > 10 cm. Nếu không dừng, dây vướng cột, giãn (1,57 → 1,9–2,2 m) và hệ Newton/VBD nổ. Lực kẹp đang dùng 3,2 N·m (56 N); chưa chứng minh được là đủ.
2. **cuRobo không lập được kế hoạch kẹp lại (posts4, socket3: 2 ca).** posts4: mọi ứng viên điểm kẹp đều thất bại (dây thẳng ít điểm kẹp gần cột, và kế hoạch về Home hỏng); socket3: tiếp cận cả 3 ứng viên đều hỏng. Chưa điều tra nguyên nhân của socket3 (ổ thấp, bên phải chỉ là giả thuyết).

## 5. Việc còn mở
- Dây tuột vẫn là lỗi chính (5/7 ca hỏng).
- Kẹp lại dây sau khi cắm ở ổ phải, thấp.
- Ổ có z ≤ 0,25 kết hợp cột 2 lệch.
- Mỗi ca chỉ chạy một lần; cuRobo và VBD có tính ngẫu nhiên nên chưa biết độ phân tán.
- 16 "thế giới" là 16 tiến trình độc lập, không phải một mô hình Newton nhiều world.

File: `data/scripts/ur5_16parallel_scenarios_r11.py` (mới), `ur5_16parallel_geom.py` (`weave_z`), `ur5_16parallel_curobo.py` (`UR5P_NATURAL`). Devlog: `data/devlog/DEVLOG_UR5_16PARALLEL_R11_2026_11_17.md`.
