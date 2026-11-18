# DEVLOG ur5_16parallel — vòng 11 (r11), 2026-11-17
Kết quả: `data/results/ur5_16parallel_r11/`, video `data/results/video_ur5_16parallel_r11.mp4`. **9/16 (56,25%)**: shape 2/4, posts 2/4, socket 2/4, all 3/4.

## Thay đổi so với r10
- `ur5_16parallel_scenarios_r11.py` (mới): ổ socket (y;z) = (0,12;0,33), (0,28;0,24), **(−0,15;0,21) bên phải cột 3**, (0,20;0,36) → biên độ z 0,15 m. Nhóm all: (0,16;0,30), (0,26;0,26), (0,14;0,34), (0,22;0,30) (biên độ z 0,08 m), cột 2 đổi y 0,15→0,04 như nhóm posts. Dây/cột giữ như r10.
- `ur5_16parallel_geom.py`: khóa `weave_z` trong kịch bản tách độ cao luồn dây (0,28 m) khỏi độ cao ổ.
- `ur5_16parallel_curobo.py`: `UR5P_NATURAL=1` loại nghiệm IK có vai q2 ngoài (−180°, 0°) trong `ik_near`.

## Vì sao ổ phải khó hơn ổ trái (đã đo)
Chạy chẩn đoán `r11d` (L/R, có/không dây, `--until transport`):
- Ổ phải: điểm cột 1 ở y = −0,2, xa chỗ nâng phích (y = +0,26). `ik_near` chọn nghiệm có chuẩn khớp nhỏ nhất, là nhánh "vắt qua lưng" (q1 = 155°, q2 = −184°, q5 = −251°).
- Đoạn tới khe 1–2 sau đó quay vai 210° và cổ tay 205° trong 8 s (ổ trái: 2 s). Phích trượt xuống trong má kẹp (độ lệch z phích–TCP 1,2 → 3,6–5 cm) và nghiêng 14° (không dây) / 35° (có dây) → `insertion IK failed`.
- Sau `UR5P_NATURAL=1`: vai quay 19°, cổ tay 3°, độ nghiêng phích tại pre_insert 1,5°. socket3 ở r11 cắm được phích (lệch 1,7 mm sau khi nhả).
- Phần nguyên nhân "lực dây làm tăng thêm" là so sánh có/không dây ở cùng quỹ đạo, chưa tách riêng từng thành phần.

## Kết quả từng ca
| Ca | OK | Lý do thất bại |
|---|---|---|
| shape1 | không | dây tuột sau 'gap12_out' |
| shape2, shape3 | có | |
| shape4 | không | dây tuột sau 'post2' |
| posts1, posts2 | có | |
| posts3 | không | dây tuột khi hạ ('lower', 262 mm) |
| posts4 | không | hết điểm kẹp dây (cuRobo) |
| socket1 | có | |
| socket2 (0,28;0,24) | không | dây tuột sau 'post1' |
| socket3 (phải) | không | cắm phích được, nhưng cuRobo không lập được kế hoạch tiếp cận kẹp lại dây (3/3 ứng viên) |
| socket4 (0,20;0,36) | có | |
| all1, all2, all3 | có | |
| all4 | không | dây tuột sau 'post1' |

## Còn mở
- Kẹp lại dây sau khi cắm ở ổ phải thấp (z = 0,21): `regrasp approach` thất bại (chưa tìm nguyên nhân).
- Dây tuột khỏi tay kẹp vẫn là nguyên nhân chính (6/7 ca hỏng còn lại).
- Ổ thấp (z ≤ 0,25) với cột 2 y = 0,04 không lập được kế hoạch → nhóm all chỉ biên độ z 0,08 m.
