# DEVLOG 2026-10-26 (V2) – UR5 + Robotiq 2F-85 luồn dây qua 3 cột và cắm vào ổ tường (ur5_screen_V2)

## 0. Overview
Bản V0 hiểu sai môi trường (3 cột xếp ngang, dây không luồn qua cột). Sau khi plans/guide/spec được sửa, V2 làm lại theo đúng mô tả:
- Robot, 3 cột nhôm, tường **thẳng hàng theo trục X**; cột 1 cách chân robot **0,50 m** (guide.md III.1); cột D = 5 cm, **H = 30 cm** (spec.md mục 4), cách nhau 0,20 m; tường (mặt ổ) ở x = 1,04 m, ổ ở y = +0,11 m, z = 0,27 m.
- Phích đi **so le**: qua cột 1 phía +Y, cột 2 phía −Y (ngược chiều), cột 3 phía +Y, rồi cắm thẳng vào ổ. Giữa hai cột, tay xoay quanh trục Z (90° ở khe 1–2, 60° ở khe 2–3) để lách qua khe.
- Video: toàn cảnh (robot, dây, 3 cột, tường), vệt quỹ đạo phích (cyan), gizmo OXYZ góc trái dưới, vector trọng lực P giữa cánh tay.

Kết quả (`python ur5_screen_V2.py --test`, 41,5 s mô phỏng, 586 s thực): chuỗi Home → Approach → Grasp → Lift → luồn cột 1-2-3 → Plug → Release → Return chạy đủ; phích cắm hết cỡ và tự giữ sau khi nhả (mặt vỏ cách mặt ổ 0,45 mm). Lực kẹp ≈ 27 N/má, tốc độ TCP ≤ 0,25 m/s. **Chưa đạt**: sai số chân 0,19/0,12 mm (yêu cầu < 0,07 mm), nghiêng ~1°.

File: `data/scripts/ur5_screen_V2.py` (+ `ur5_screen_geom_V2.py`, `ur5_screen_curobo_V2.py`, `ur5_screen_render_isaac_V2.py`); video `data/results/video_ur5_screen_V2.mp4`. (Các file `_V1` là bước trung gian của cùng ngày, giữ nguyên; V0 không đổi.)

## 1. Thay đổi so với V0
| Mục | V0 | V2 |
|---|---|---|
| Cột | 3 cột r=2 cm, H=40 cm, xếp ngang (khe cổng) | 3 cột D=5 cm, H=30 cm, thẳng hàng với robot và tường, cách nhau 0,2 m |
| Quỹ đạo | qua khe giữa cột 2–3 | so le: cột 1 (+Y) → khe → cột 2 (−Y) → khe → cột 3 (+Y) → ổ, mỗi đoạn do cuRobo lập (joint-space, nghiệm IK liên tục) |
| Dây | 0,9 m, đuôi kéo dài ra xa | 1,2 m, đuôi xếp gọn cạnh robot (hai đoạn song song nối chữ U) để bị kéo theo đường luồn cột |
| Tường | 1,6 × 0,9 m | 1,0 × 0,6 m |
| Tay kẹp | nghiêng 70° | nghiêng 40° (với tới tường xa 1,04 m và cổ tay không chạm đỉnh cột) |
| Video | cố định | camera xa, thêm vệt quỹ đạo phích |

## 2. Lỗi, nguyên nhân, giải pháp
| # | Lỗi | Nguyên nhân | Giải pháp |
|---|---|---|---|
| 1 | **Dây "đứt" khi chạm cột 2** | độ cứng chống giãn của dây `--stretch-stiffness` = 10 quá mềm: khi dây căng (kéo tới cột, ma sát sàn) các đoạn bị giãn rời nhau, nhìn như đứt; thêm đuôi dây dài làm dây căng thẳng hơn | tăng lên 1e6 (như các ví dụ cable của Newton) và xếp đuôi dây gọn; video V2 dây liên tục |
| 2 | cuRobo không qua khe giữa hai cột | phích dài 11 cm + bàn kẹp rộng 9 cm lớn hơn khe | xoay tay quanh Z khi qua khe (pins dọc hướng đi), cột cách 0,2 m |
| 3 | cuRobo thất bại dù kiểm tra hình cầu tay kẹp ổn | cầu cổ tay (z ≈ 0,33–0,35 m) chạm đỉnh cột (0,30 m); hàm `clearance()` chỉ xét tay kẹp | nâng độ cao phích khi luồn lên 0,27 m, tilt 40°; `clearance()` dùng hình trụ hữu hạn; thăm dò bằng cầu của cuRobo |
| 4 | IK không có nghiệm tại khe 2–3 với xoay +90° | ngoài vùng với tới (nhánh trước) | khe 2–3 xoay 60° |
| 5 | goal IK ra nhánh khác / vượt giới hạn khớp | IK cục bộ | `ik_near`: nhiều hạt giống, chọn nghiệm gần cấu hình trước nhất, trong giới hạn khớp; lập kế hoạch bằng `plan_cspace` |

## 3. Hạn chế
- Sai số chân 0,19/0,12 mm và nghiêng ~1° (> 0,07 mm): chỉ cắm được nhờ độ mềm contact của MuJoCo; cần servo lực/thị giác và vát dẫn hướng 1,5 mm.
- Dây chỉ tác động một chiều lên phích (không kéo ngược); chưa có số đo định lượng "không quấn cột" (chỉ quan sát video).
- Dây 1,2 m (spec ghi "10–20 m" – không mô phỏng được); UR5e thay UR5; D cột = 5 cm (nhỏ nhất theo spec) để khe đủ rộng.
- Đoạn thẳng (Descend, Lift, Insert, Retreat) không được cuRobo kiểm va chạm cánh tay; các đoạn luồn cột dừng ở từng điểm giữa.

## 4. Hướng phát triển
Servo lực khi cắm, vát dẫn hướng, đo khoảng cách dây–cột, quỹ đạo liên tục không dừng giữa các điểm, đo lực căng dây.

## 5. Chạy lại
```
cd data/scripts
python ur5_screen_V2.py --test            # cuRobo + Newton (MuJoCo + VBD)
python ur5_screen_render_isaac_V2.py      # data/results/video_ur5_screen_V2.mp4
```
