# DEVLOG ur5_16parallel — 2026-11-14

## Tổng quan
Thực hiện plans/UR5_16parallel.md: chạy lại nhiệm vụ regrasp (V6) trên 16 thế giới, 4 nhóm x 4 thử nghiệm
(hình dáng dây / vị trí cột / vị trí lỗ / đổi cả ba). Không sửa file V6; toàn bộ code mới có tiền tố `ur5_16parallel`.
(`instructions/sapc.md` không tồn tại; dùng `instructions/spec.md`.)

**Kết quả: 7/16 thành công (43,75%)** — shape 2/4, posts 2/4, socket 2/4, all 1/4. Chi tiết: `data/results/ur5_16parallel/summary.json`.
Video bảng 4x4 (cột = nhóm): `data/results/video_ur5_16parallel.mp4`.

## File mới (data/scripts)
- `ur5_16parallel_geom.py`: như geom V6 nhưng đọc kịch bản JSON từ biến môi trường `UR5P_SCEN` (cột, ổ, dáng dây); khoảng ngang tới từng cột `WEAVE_OFFS`.
- `ur5_16parallel_curobo.py`: planner; file robot urdf/yml riêng theo `UR5P_ID` để tiến trình song song không ghi đè nhau.
- `ur5_16parallel.py`: mô phỏng; dáng dây ban đầu theo `CABLE_SHAPE` (straight/s/arc/zigzag); thư mục kết quả theo kịch bản; ghi `result.json` kể cả khi lỗi; thử tối đa 5 điểm kẹp dây ứng viên; nới tiêu chí điểm kẹp khi không có điểm nào.
- `ur5_16parallel_render_isaac.py`, `ur5_16parallel_scenarios.py`, `ur5_16parallel_run.sh`, `ur5_16parallel_grid.py`.

## Lưu ý về "16 thế giới song song"
16 tiến trình độc lập (mỗi cái có Newton MuJoCo + VBD + cuRobo riêng), không phải 16 world trong một model Newton.

## Lỗi và xử lý
1. Thiếu `return pts` trong `_cable_points` -> thêm lại.
2. Chạy 14 tiến trình cùng lúc: cuRobo hết bộ nhớ GPU (9,6 GB) ở 4 kịch bản -> chạy lại, khởi động cách nhau 60 s.
3. all1 (ổ z = 0,25 m) không lập được kế hoạch tới cột 3 -> đổi ổ về z >= 0,27 m (all1, all3, all4); all1 vẫn thất bại ở đoạn về Home.
4. "no graspable cable point" -> nới tiêu chí (tầm 0,28–0,70 m, x > 0,10, cách cột > 8 cm); một số kịch bản vẫn thất bại.

## Hạn chế / hướng tiếp
- Nguyên nhân thất bại: dây tuột khỏi tay kẹp khi vẫy (shape2, socket4, all2) và cuRobo không lập được kế hoạch kẹp lại / về Home (shape3, posts2, posts4, socket2, all1, all4).
- Chỉ mỗi kịch bản chạy một lần (cuRobo/VBD có tính ngẫu nhiên), chưa lặp lại để đo độ phân tán.
- Hướng: chọn điểm kẹp dây theo khả năng tới của UR5 thay vì tiêu chí cố định; tăng lực kẹp dây ở khe 2-3; thêm planner thử lại nhiều seed.

---
# Vòng 2 (2026-11-15): kịch bản ngẫu nhiên có seed — thư mục `data/results/ur5_16parallel_r2/`
Yêu cầu mới: dây cong/chồng lên nhau trong khung hình; cột không thẳng hàng; bỏ đường quỹ đạo xanh. (Ổ dưới đất: để giai đoạn 2.)
- `ur5_16parallel_scenarios.py`: lấy mẫu ngẫu nhiên (seed 2025-2040). Dây: đường cong tùy ý, độ cong <= 14 rad/m, trong hộp x -0.05..0.80, y 0.17..0.80,
  đoạn giao cắt được nâng 1.1 cm; cột: x1 0.33-0.42, khe 0.25-0.28, y lệch +-0.08; ổ trên tường y 0.15-0.28, z 0.26-0.29.
- `ur5_16parallel.py`: `CABLE_SHAPE type=poly` đọc đường dây từ JSON. Render: tắt đường quỹ đạo xanh. Biến môi trường `UR5P_ROOT` chọn thư mục kết quả.
- Kết quả: **5/16 (31,25%)**: shape 2/4 (shape3, shape4), posts 0/4, socket 1/4 (socket1), all 2/4 (all1, all2).
- Nguyên nhân thất bại: dây tuột khỏi tay kẹp khi vẫy (shape1, shape2, posts2, socket2, socket3, all3), không có điểm kẹp dây khả thi (posts1, posts3, socket4),
  cuRobo không lập được kế hoạch (posts4, all4).
- Hạn chế: nhóm posts/socket giữ dây thẳng nên các ô này trông giống nhau về dây; chênh lệch vị trí ổ chỉ vài cm nên khó thấy.
