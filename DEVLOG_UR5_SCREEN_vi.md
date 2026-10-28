# DEVLOG UR5_SCREEN (V0 → V3) – UR5e + Robotiq 2F-85 cắm phích NEMA 5-15P qua 3 cột

Tổng hợp từ `DEVLOG_UR5_SCREEN_2026_10_26.md` (V0), `_V2.md`, `_V3.md`. Mỗi phiên bản đi theo mạch: **hiện tượng → giả thuyết → thay đổi → kết quả**. V1 chỉ là bước trung gian trong ngày (không có devlog riêng, file `_V1` giữ nguyên).

**Nhiệm vụ** (`plans/UR5_Screen.md`): UR5 + Robotiq 2F-85 gắp phích ở đầu dây cáp VBD trên sàn, nhấc lên, đưa qua hàng 3 cột chướng ngại, cắm vào ổ tường (vuông góc sàn), nhả, về Home. Video Isaac Sim có gizmo OXYZ (góc trái dưới) và vector trọng lực P ở giữa cánh tay.

**Công nghệ**: Newton + `SolverMuJoCo` (robot, phích, contact; 16 substep/khung 60 Hz), `SolverVBD` cho dây (một chiều: dây không tác động ngược lên phích), cuRobo 0.8 lập kế hoạch tránh va chạm, Isaac Sim headless dựng video.

## Tổng quan kết quả

| Phiên bản | Cấu hình chính | Phích cắm & tự giữ | Sai số chân / nghiêng | Dây |
|---|---|---|---|---|
| V0 | 3 cột r=2 cm, H=40 cm xếp ngang, đi qua khe cột 2–3 | Có, mặt vỏ cách ổ 0,49 mm | 0,19 / 0,11 mm, ~1,0° | Theo phích, không quấn cột (chỉ xem video) |
| V2 | 3 cột D=5 cm, H=30 cm thẳng hàng robot–tường, luồn so le | Có, 0,45 mm | 0,19 / 0,12 mm, ~1° | Liên tục, nhưng xuyên cột 2 tối đa 17,8 mm |
| V3 | Như V2, đổi độ cứng tiếp xúc dây | Có, 0,45 mm | 0,19 / 0,12 mm, ~1° | Không xuyên cột, giãn lớn nhất 1,005 |

Yêu cầu sai số chân < 0,07 mm **chưa đạt** ở cả ba phiên bản.

---

## V0 – `ur5_screen.py`: dựng chuỗi cắm phích cơ bản

### Thiết kế
| Thành phần | Cách làm |
|---|---|
| Robot | UR5e (Menagerie, không có UR5 thuần) + Robotiq 2F-85 |
| Phích | Vật rắn tự do 0,1 kg: vỏ 50×22×30 mm, 2 lưỡi dẹt 18×6,35×1,524 mm (cách nhau 12,7 mm), chân tròn Ø4,76×21 mm lệch dưới 7,9 mm |
| Ổ cắm | Khe 6,5×1,8 mm sâu 17,5 mm; lỗ tròn Ø4,9 mm sâu 21,5 mm; khe hở 0,075 / 0,07 mm; toàn bằng hộp (không dùng mesh lõm) |
| Cột | 3 cột trụ r = 20 mm, cao 0,40 m, thẳng hàng theo Y tại x = 0,58 m; khe sáng 0,14 m |
| Dây | VBD, 45 đoạn, dài 0,9 m; 4 đoạn đầu gắn cứng theo phích |
| Lập kế hoạch | cuRobo `plan_pose` / `plan_cspace` (tránh sàn, tường, cột, tự va chạm, cầu bao phích); đoạn thẳng theo trục tay kẹp dùng IK giảm chấn |
| Tri giác trong tay | Đo pose phích so với TCP rồi tính lại đường cắm thẳng |

Giả định tự quyết (chờ xác nhận): dây 0,9 m (spec "10–20 m" quá lớn cho cảnh); phích đi qua khe giữa cột 2–3; phích đặt đứng trên cạnh nhỏ, kẹp nghiêng 70°; Pose A = (0,30; 0) m, ổ tại (0,78; 0,09; 0,28) m. Hệ tọa độ bàn tay phải (spec ghi Y "từ trong ra ngoài" mâu thuẫn với X phải, Z lên) – cần xác nhận.

### Hiện tượng → giả thuyết → thay đổi → kết quả

| # | Hiện tượng | Giả thuyết (nguyên nhân) | Thay đổi | Kết quả |
|---|---|---|---|---|
| 1 | cuRobo báo `Numpy is not available` | torch 2.11 cần numpy 2, môi trường `cable` dùng numpy 1.26 (Isaac Sim/Newton) | Cài numpy 2.2.6, scipy 1.14.1, cuda-core vào `~/.cache/cable_task/np2`; chạy cuRobo ở tiến trình con có `PYTHONPATH` + `LD_LIBRARY_PATH` tới `nvrtc` 13 | cuRobo chạy, không ảnh hưởng conda |
| 2 | cuRobo không có cấu hình UR5e | Chỉ có UR10e | Tự viết URDF từ Menagerie + cầu va chạm tay kẹp | FK khớp Newton (TCP home = (0,492; 0,134; 0,3465)) |
| 3 | Phích kẹt ở mặt ổ | Lỗi hình học: thanh tường lấp kín cửa sổ ổ | Tường dựng từ 4 hộp quanh cửa sổ + khối ổ khoét khe bằng lưới hộp | Phích vào được ổ |
| 4 | Phích tuột rồi rơi khi vận chuyển | Đường cuRobo xuyên cột: cột trụ (`cylinder`) bị bỏ qua trong scene cuRobo; Home cũng nằm trong cột | Dùng hộp bao cho cột, dời cột/tường ra x = 0,58/0,78, thêm `clearance()` kiểm độc lập, thêm điểm trung gian ở cổng | Vận chuyển qua khe không rơi |
| 5 | Phích dừng cách ổ 46 mm | Khớp `wrist_1_link` (z≈0,48 m) va cột giữa lúc đẩy; đoạn thẳng không được kiểm va chạm cánh tay | Hạ cột xuống 0,40 m, lập đoạn "qua cổng" bằng cuRobo (có kiểm cánh tay), thêm `debug_contacts` để in contact | Phích đẩy tới ổ |
| 6 | Tường hiển thị sai chỗ trong video | USD áp scale trước transform làm nhân tọa độ | Áp transform trước rồi mới scale | Tường đúng vị trí |
| 7 | Đổi `ke` của shape không đổi kết quả | Độ cứng contact MuJoCo do `solref/solimp`, không do `ke` | Thử `solref`=0,004 và `solimp` chặt | **Phích kẹt** (nghiêng 3,4°, cách ổ 21 mm): tri giác trong tay chỉ chính xác ~0,5° (≈0,2 mm ở đầu chân) > khe hở 0,07 mm. Tắt mặc định (`SCREEN_STIFF=1` để bật) |

### Kết quả V0 (`python ur5_screen.py --test`)
- Chuỗi Home → Approach → Grasp → Lift → Transport (khe cột 2–3) → Plug → Release → Retreat → Return chạy đủ; phích cắm hết cỡ, tự giữ sau khi nhả, mặt vỏ cách ổ **0,49 mm** (thiết kế 0,5 mm).
- Lực kẹp 26–28 N mỗi má (khuyến nghị 12–30 N); tốc độ TCP khi mang phích ≤ 0,25 m/s, gia tốc ≤ 0,8 m/s².
- **Chưa đạt**: lệch trục chân 0,19 mm ở mặt ổ, 0,11 mm ở đầu lưỡi, nghiêng ~1,0° (yêu cầu < 0,07 mm). Cắm được là nhờ độ mềm contact mặc định của MuJoCo (xuyên ~0,1 mm).
- Hạn chế: lực cắm chỉ gián tiếp (~7 N), chưa điều khiển lực kẹp; cuRobo dùng hộp bao cho cột; dây một chiều; "không quấn cột" chỉ kiểm bằng video; chưa kiểm tính lặp lại.

### Vấn đề dẫn sang V2
Sau khi `plans/guide/spec` được sửa, rõ ra **V0 hiểu sai môi trường**: 3 cột phải thẳng hàng theo trục X (robot → cột → tường) và dây phải được luồn qua cột, không chỉ đi qua một khe.

---

## V2 – `ur5_screen_V2.py`: làm lại theo đúng mô tả (luồn so le qua 3 cột)

### Thay đổi so với V0
| Mục | V0 | V2 |
|---|---|---|
| Cột | 3 cột r=2 cm, H=40 cm, xếp ngang | 3 cột D=5 cm, H=30 cm, thẳng hàng robot–tường, cột 1 cách chân robot 0,50 m, cách nhau 0,20 m |
| Quỹ đạo | Qua khe giữa cột 2–3 | So le: cột 1 (+Y) → khe → cột 2 (−Y) → khe → cột 3 (+Y) → ổ; mỗi đoạn do cuRobo lập (joint-space, IK liên tục); tay xoay quanh Z 90° ở khe 1–2, 60° ở khe 2–3 |
| Dây | 0,9 m, đuôi kéo dài ra xa | 1,2 m, đuôi xếp gọn thành hai đoạn song song nối chữ U để bị kéo theo đường luồn cột |
| Tường / ổ | 1,6×0,9 m | 1,0×0,6 m, mặt ổ tại x = 1,04 m, ổ ở y = +0,11 m, z = 0,27 m |
| Tay kẹp | Nghiêng 70° | Nghiêng 40° (với tới tường xa, cổ tay không chạm đỉnh cột) |
| Video | Cố định | Camera xa, thêm vệt quỹ đạo phích (cyan) |

### Hiện tượng → giả thuyết → thay đổi → kết quả
| # | Hiện tượng | Giả thuyết | Thay đổi | Kết quả |
|---|---|---|---|---|
| 1 | Dây "đứt" khi chạm cột 2 | `--stretch-stiffness` = 10 quá mềm: dây căng (kéo tới cột, ma sát sàn) thì các đoạn giãn rời nhau, nhìn như đứt; đuôi dài làm dây căng hơn | Tăng lên 1e6 (như ví dụ cable của Newton), xếp đuôi gọn | Dây liên tục trong video V2 *(nhưng gây lỗi ở V3)* |
| 2 | cuRobo không qua khe giữa hai cột | Phích dài 11 cm + bàn kẹp rộng 9 cm lớn hơn khe | Xoay tay quanh Z khi qua khe (phích dọc hướng đi), cột cách 0,2 m | Qua được khe |
| 3 | cuRobo thất bại dù cầu tay kẹp ổn | Cầu cổ tay (z ≈ 0,33–0,35 m) chạm đỉnh cột 0,30 m; `clearance()` chỉ xét tay kẹp | Nâng độ cao phích khi luồn lên 0,27 m, tilt 40°; `clearance()` dùng trụ hữu hạn; thăm dò bằng cầu cuRobo | Lập kế hoạch thành công |
| 4 | IK vô nghiệm ở khe 2–3 với xoay +90° | Ngoài vùng với tới (nhánh trước) | Khe 2–3 xoay 60° | Có nghiệm |
| 5 | Goal IK ra nhánh khác / vượt giới hạn khớp | IK cục bộ | `ik_near`: nhiều hạt giống, chọn nghiệm gần cấu hình trước nhất, trong giới hạn khớp; lập kế hoạch bằng `plan_cspace` | Nghiệm liên tục |

### Kết quả V2 (`python ur5_screen_V2.py --test`, 41,5 s mô phỏng, 586 s thực)
- Chuỗi Home → Approach → Grasp → Lift → luồn cột 1-2-3 → Plug → Release → Return chạy đủ; phích cắm hết cỡ, tự giữ, mặt vỏ cách ổ 0,45 mm. Lực kẹp ≈ 27 N/má, TCP ≤ 0,25 m/s.
- **Chưa đạt**: sai số chân 0,19/0,12 mm, nghiêng ~1°.
- **Lỗi mới phát hiện khi xem video**: lúc cắm phích, đoạn dây đang ôm cột 2 đột ngột đi xuyên qua cột → V3.

---

## V3 – `ur5_screen_V3.py`: sửa dây xuyên cột 2

### Hiện tượng
Đo trên quỹ đạo V2: dây xuyên vào cột 2 tối đa **17,8 mm** (bán kính cột 25 mm) trong t = 24,4–25,5 s; cột 1 và 3 không bị.

### Giả thuyết
Va chạm dây–cột trong Newton/VBD là tiếp xúc penalty với `ke = 1e3` N/m. Ở V2, để hết "đứt dây", độ cứng chống giãn tăng từ 10 lên 1e6 ⇒ dây gần như không giãn; khi phích kéo căng dây quanh cột 2, lực căng lớn hơn nhiều so với lực đẩy tối đa của tiếp xúc mềm ⇒ dây bị ép xuyên qua. Hai thông số lệch nhau: **dây cứng, tiếp xúc mềm**.

### Kiểm chứng
Phát lại đúng quỹ đạo phích V2 chỉ qua mô hình dây (script tạm, ngoài dự án) với các `ke` khác nhau:

| ke | Độ xuyên cột 2 |
|---|---|
| 1e3 (V2) | 22,8 mm (tái hiện lỗi) |
| 1e5 | Không xuyên (khe hở nhỏ nhất −0,2 mm) |
| 1e7 | Không xuyên (+0,1 mm) |

Giả thuyết được xác nhận.

### Thay đổi
- Đưa độ cứng/giảm chấn tiếp xúc của dây thành tham số: `--cable-contact-ke` (mặc định **1e5**, trước 1e3) và `--cable-contact-kd` (mặc định **10**). Chọn 1e5 thay vì 1e7 để tránh bất ổn số khi tiếp xúc quá cứng.
- Quỹ đạo robot, cuRobo, hình học giữ nguyên V2.

### Kết quả V3 (`python ur5_screen_V3.py --test`, 41,5 s mô phỏng)
| Cột | Khe hở dây–cột nhỏ nhất | Khung xuyên > 2 mm |
|---|---|---|
| 1 | 41,4 mm | 0 |
| 2 | −0,2 mm (chạm) | 0 |
| 3 | 68,1 mm | 0 |

Dây không xuyên cột nào; tỉ lệ giãn đoạn lớn nhất 1,005. Phích cắm hết cỡ, tự giữ (0,45 mm). Sai số chân 0,19/0,12 mm và nghiêng ~1° vẫn **chưa đạt**.

---

## Hạn chế còn lại (đến V3)
- Sai số chân > 0,07 mm; cắm được nhờ độ mềm contact của MuJoCo.
- Dây tác động một chiều lên phích (không kéo ngược); chưa đo lực căng dây; tiếp xúc 1e5 N/m vẫn cho xuyên ~0,2 mm khi căng mạnh (không nhìn thấy).
- Dây 1,2 m (spec ghi "10–20 m" không mô phỏng được); UR5e thay UR5; D cột = 5 cm (nhỏ nhất theo spec).
- Đoạn thẳng (Descend, Lift, Insert, Retreat) không được cuRobo kiểm va chạm cánh tay; các đoạn luồn cột dừng ở từng điểm giữa.
- Chưa kiểm tính lặp lại nhiều lần.

## Hướng phát triển
1. Servo lực/thị giác khi cắm (tìm lỗ bằng lực), giảm sai số < 0,07 mm; contact cứng + căn chỉnh theo phản hồi.
2. Vát dẫn hướng 1,5 mm × 45° ở miệng khe/lỗ (spec) để cho phép lệch ±5°.
3. Điều khiển lực kẹp theo phản hồi (15–30 N), tránh trượt 12 mm lúc nhấc đầu tiên.
4. cuRobo kiểm cả đoạn thẳng; dây–phích hai chiều; đo khoảng cách dây–cột và lực căng dây; quỹ đạo liên tục không dừng giữa các điểm.

## Cách chạy lại
```
cd data/scripts
python ur5_screen.py --test            # V0 (video: ur5_screen_render_isaac.py)
python ur5_screen_V2.py --test         # V2
python ur5_screen_V3.py --test          # V3
python ur5_screen_render_isaac_V3.py   # data/results/video_ur5_screen_V3.mp4
```
Tùy chọn: `--no-cable`, `--grip-effort`, `--v-carry`, `--replan`, `--cable-contact-ke`, `--cable-contact-kd` (V3).
