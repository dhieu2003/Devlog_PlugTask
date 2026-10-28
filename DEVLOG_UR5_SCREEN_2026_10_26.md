# DEVLOG 2026-10-26 – UR5 + Robotiq 2F-85 cắm phích NEMA 5-15P vào ổ tường, qua 3 cột (ur5_screen)

## 0. Overview công việc hôm nay

**Mục tiêu** (`plans/UR5_Screen.md`): UR5 + Robotiq 2F-85 gắp phích ở đầu dây cáp VBD đang nằm trên sàn, nhấc lên, đưa qua hàng 3 cột chướng ngại, cắm vào ổ trên tường (vuông góc sàn), nhả, về Home. Video Isaac Sim có gizmo OXYZ (góc trái dưới) và vector trọng lực P ở giữa cánh tay.

**Kết quả của phiên bản V0 (`ur5_screen.py`)** – chạy lại bằng `python ur5_screen.py --test`:
- Chuỗi Home → Approach → Grasp → Lift → Transport (qua khe giữa cột 2 và cột 3) → Plug → Release → Retreat → Return chạy đủ, phích **cắm hết cỡ và tự giữ trong ổ sau khi nhả**: mặt trước vỏ phích cách mặt ổ **0,49 mm** (thiết kế 0,5 mm: lưỡi dẹt chạm đáy hốc sâu 17,5 mm).
- Lực kẹp 26–28 N mỗi má (spec khuyến nghị 12–30 N); tốc độ TCP khi mang phích ≤ 0,25 m/s, gia tốc ≤ 0,8 m/s².
- **Chưa đạt**: sai số vị trí chân < 0,07 mm. Đo được: lệch trục chân so với trục ổ **0,19 mm** tại mặt ổ và **0,11 mm** ở đầu lưỡi; phích nghiêng ~1,0° (xem mục 4).
- Dây cáp VBD đi theo phích, không quấn vào cột (xác nhận bằng video, chưa có phép đo định lượng).

**File** (trong `data/scripts/`): `ur5_screen.py` (mô phỏng), `ur5_screen_geom.py` (hình học, FK/IK), `ur5_screen_curobo.py` (cuRobo), `ur5_screen_render_isaac.py` (Isaac Sim). Dữ liệu: `data/assets/ur5e_curobo/` (URDF + cấu hình cầu va chạm), `data/results/ur5_screen_{plan,traj}.npz`, `ur5_screen_shapes.pkl`, `video_ur5_screen.mp4`.

---

## 1. Thiết kế

| Thành phần | Cách làm |
|---|---|
| Robot + phích + sàn/cột/tường/ổ | Newton, `SolverMuJoCo` (contact gốc MuJoCo, 16 substep/khung 60 Hz). UR5e (Menagerie, không có UR5 thuần) + Robotiq 2F-85. |
| Phích | Vật rắn tự do 0,1 kg: vỏ 50×22×30 mm, 2 lưỡi dẹt 18×6,35×1,524 mm (cách nhau 12,7 mm), chân tròn Ø4,76×21 mm lệch dưới 7,9 mm (spec). |
| Ổ cắm | Khe 6,5×1,8 mm sâu 17,5 mm; lỗ tròn Ø4,9 mm sâu 21,5 mm (đa giác 16 cạnh bằng hộp); khe hở 0,075 / 0,07 mm. Toàn bộ bằng hộp (không dùng mesh lõm). |
| Cột | 3 cột trụ r = 20 mm, cao 0,40 m, thẳng hàng theo Y tại x = 0,58 m; khe sáng 0,14 m. |
| Dây cáp | Newton `SolverVBD` (mô hình riêng, 45 đoạn, dài 0,9 m): 4 đoạn đầu gắn cứng theo phích (đọc pose phích từ MuJoCo mỗi substep, nội suy), va chạm sàn/cột/tường. Một chiều: dây không tác động ngược lên phích (khối lượng dây nhỏ). |
| Lập kế hoạch | cuRobo 0.8: `plan_pose` / `plan_cspace` tránh sàn, tường, 3 cột, tự va chạm, và cầu bao phích khi đang mang. Đoạn thẳng theo trục tay kẹp (Descend, Lift, Insert, Retreat) bằng IK giảm chấn trên cùng động học. |
| Tri giác trong tay | Sau khi đưa phích tới cổng, đo pose phích so với TCP (từ trạng thái mô phỏng) và tính lại đường cắm thẳng để chân thẳng hàng với ổ. |
| Video | Isaac Sim headless, phát lại quỹ đạo; gizmo OXYZ + mũi tên P. |

Quy ước hệ tọa độ: gốc ở chân đế, Z lên, X sang phải, Y theo quy tắc bàn tay phải. **Lưu ý**: `spec.md` mô tả Y hướng "từ trong ra ngoài"; với X sang phải và Z lên thì bàn tay phải cho Y đi vào màn hình, nên tôi giữ bàn tay phải (như bản V3 trước) – cần bạn xác nhận.

## 2. Các giả định tôi tự quyết định (xin bạn xác nhận)
1. Dây "10–20 m" tôi hiểu là quá lớn cho cảnh này → dùng 0,9 m (kế hoạch cũ ghi 10–50 cm).
2. "Luồn qua 3 cột" → phích đi qua khe giữa cột 2 và 3 (hàng cột vuông góc hướng đi), dây theo sau qua khe, không vòng quanh cột.
3. Phích đặt đứng trên cạnh nhỏ (mặt lớn thẳng đứng) để tay kẹp kẹp ngang mặt lớn, kẹp nghiêng 70° so với phương ngang (để má kẹp không chạm sàn).
4. Pose A = (0,30; 0) m, ổ tại (x = 0,78 m, y = 0,09 m, z = 0,28 m), Pose B = vị trí ổ.

## 3. Lỗi gặp phải, nguyên nhân, giải pháp

| # | Lỗi | Nguyên nhân | Giải pháp / kết quả |
|---|---|---|---|
| 1 | cuRobo: `Numpy is not available` | torch 2.11 cần numpy 2, môi trường `cable` dùng numpy 1.26 (Isaac Sim/Newton) | Cài numpy 2.2.6, scipy 1.14.1, cuda-core vào `~/.cache/cable_task/np2` (không đụng conda); chạy cuRobo ở tiến trình con có `PYTHONPATH` đó + `LD_LIBRARY_PATH` tới `nvrtc` 13. |
| 2 | cuRobo không có cấu hình UR5e | chỉ có UR10e | Tự viết URDF từ Menagerie (FK khớp Newton: TCP home = (0,492; 0,134; 0,3465) ở cả hai) + cầu va chạm tay kẹp. |
| 3 | Ổ cắm: phích kẹt ở mặt ổ | tôi để thanh tường lấp kín cửa sổ ổ (lỗi hình học) | Tường tạo từ 4 hộp quanh cửa sổ + khối ổ khoét khe bằng lưới hộp. |
| 4 | Phích tuột rồi rơi khi vận chuyển | Đường cuRobo xuyên cột: cột trụ (`cylinder`) bị bỏ qua trong scene cuRobo; Home cũng nằm trong cột | Dùng hộp bao cho cột, dời cột/tường ra x = 0,58/0,78, thêm hàm kiểm khoảng cách `clearance()` độc lập, thêm điểm trung gian ở cổng. |
| 5 | Phích dừng cách ổ 46 mm | khớp `wrist_1_link` (z≈0,48 m) va cột giữa lúc đẩy; không thấy trong kế hoạch vì đoạn thẳng không kiểm cánh tay | Hạ cột xuống 0,40 m, lập đoạn "qua cổng" bằng cuRobo (có kiểm cánh tay); in danh sách contact (`debug_contacts`) để chẩn đoán. |
| 6 | Hiển thị tường sai chỗ trong video | USD áp scale trước transform làm nhân tọa độ | Thêm transform trước rồi mới scale. |
| 7 | `ke` của shape không đổi kết quả | độ cứng contact MuJoCo do `solref/solimp`, không phải `ke` | Thử `solref`=0,004 và `solimp` chặt: **phích kẹt** (nghiêng 3,4°, dừng cách ổ 21 mm) vì tri giác trong tay chỉ chính xác ~0,5° (≈0,2 mm ở đầu chân) lớn hơn khe hở 0,07 mm. Tắt mặc định (`SCREEN_STIFF=1` để bật). |

## 4. Hạn chế còn tồn tại
- **Sai số chân 0,11–0,19 mm, nghiêng ~1°** (yêu cầu < 0,07 mm): việc cắm thành công nhờ độ mềm contact mặc định của MuJoCo (cho phép xuyên ~0,1 mm); không đạt dung sai 0,07 mm ở nghĩa chặt. Mô phỏng contact cứng cần căn thẳng chính xác hơn 0,05°.
- Lực cắm chỉ gián tiếp (đẩy vượt 2 mm; ~7 N tại phích/tường); chưa điều khiển lực kẹp/cắm theo khuyến nghị 10→30 N.
- Cột: cuRobo dùng hộp bao (bảo thủ); các đoạn thẳng không được cuRobo kiểm va chạm cánh tay (chỉ kiểm khoảng cách tay kẹp/phích bằng `clearance()` và contact lúc chạy).
- Dây chỉ một chiều (không kéo ngược phích); tay kẹp xuyên qua dây ở sau phích (dây không va tay kẹp).
- Chưa có phép đo định lượng "dây không quấn cột".
- UR5e thay UR5; mô phỏng chưa kiểm tính lặp lại nhiều lần.

## 5. Hướng phát triển
1. Servo thị giác/lực khi cắm (tìm lỗ bằng lực), giảm sai số xuống < 0,07 mm; contact cứng + căn chỉnh theo phản hồi.
2. Vát dẫn hướng 1,5 mm × 45° ở miệng khe/lỗ (spec) để cho phép lệch ±5°.
3. Mô-men/lực kẹp điều khiển theo phản hồi (15–30 N), tránh trượt 12 mm lúc nhấc đầu tiên.
4. Cho cuRobo kiểm cả đoạn thẳng; hai chiều giữa dây và phích; đo khoảng cách dây–cột.

## 6. Cách chạy lại
```
cd data/scripts
python ur5_screen.py --test            # cuRobo (tự chạy), MuJoCo + VBD, lưu quỹ đạo
python ur5_screen_render_isaac.py      # video data/results/video_ur5_screen.mp4
```
Tùy chọn: `--no-cable` (nhanh, debug), `--grip-effort`, `--v-carry`, `--replan`.
