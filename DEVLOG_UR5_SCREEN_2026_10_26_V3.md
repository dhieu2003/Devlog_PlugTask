# DEVLOG 2026-10-26 (V3) – Sửa dây xuyên qua cột 2 (ur5_screen_V3)

## 0. Overview
Video V2 có lỗi vật lý: khi robot cắm phích vào ổ, đoạn dây đang ôm cột 2 đột ngột đi xuyên qua cột. V3 sửa lỗi đó; quỹ đạo robot, kế hoạch cuRobo và kết quả cắm giữ nguyên như V2.

Kết quả V3 (`python ur5_screen_V3.py --test`, 41,5 s mô phỏng): dây không còn xuyên cột nào (đo trên toàn bộ quỹ đạo), dây không bị giãn (tỉ lệ giãn đoạn lớn nhất 1,005). Phích vẫn cắm hết cỡ và tự giữ sau khi nhả (mặt vỏ cách mặt ổ 0,45 mm); sai số chân 0,19/0,12 mm và nghiêng ~1° vẫn **chưa đạt** yêu cầu < 0,07 mm (xem V2).
File: `data/scripts/ur5_screen_V3.py` (+ `_geom_V3`, `_curobo_V3`, `_render_isaac_V3`), video `data/results/video_ur5_screen_V3.mp4`.

## 1. Lỗi và nguyên nhân
- **Hiện tượng**: ở V2, dây xuyên vào cột 2 tối đa **17,8 mm** (bán kính cột 25 mm) trong t = 24,4–25,5 s; cột 1 và 3 không bị.
- **Nguyên nhân**: va chạm dây–cột trong Newton/VBD là tiếp xúc penalty, độ cứng `ke = 1e3` N/m. Ở bước sửa lỗi "dây đứt" trước đó, độ cứng chống giãn của dây được tăng từ 10 lên 1e6 → dây gần như không giãn, nên khi phích kéo căng dây quanh cột 2, lực căng lớn hơn nhiều so với lực đẩy tối đa của tiếp xúc mềm ⇒ dây bị ép xuyên qua cột. (Hai thông số lệch nhau: dây cứng, tiếp xúc mềm.)
- **Cách kiểm chứng**: phát lại đúng quỹ đạo phích của V2 chỉ qua mô hình dây (script tạm ở thư mục scratchpad, không thuộc dự án) với các `ke` khác nhau:

| ke | độ xuyên cột 2 |
|---|---|
| 1e3 (V2) | 22,8 mm (tái hiện lỗi) |
| 1e5 | không xuyên (khe hở nhỏ nhất −0,2 mm) |
| 1e7 | không xuyên (+0,1 mm) |

## 2. Thay đổi trong V3
- Độ cứng/giảm chấn tiếp xúc của dây thành tham số: `--cable-contact-ke` (mặc định **1e5**, trước là 1e3) và `--cable-contact-kd` (mặc định **10**). Chọn 1e5 thay vì 1e7 để tránh bất ổn số khi tiếp xúc quá cứng.
- Không đổi quỹ đạo robot, cuRobo, hình học.

## 3. Kết quả đo (trên toàn bộ video V3)
| Cột | khe hở dây–cột nhỏ nhất | khung xuyên > 2 mm |
|---|---|---|
| 1 | 41,4 mm | 0 |
| 2 | −0,2 mm (chạm) | 0 |
| 3 | 68,1 mm | 0 |

[https://github.com/user-attachments/assets/3bd77422-2d84-4741-8d16-f87c252aaa50](https://github.com/user-attachments/assets/3bd77422-2d84-4741-8d16-f87c252aaa50)

## 4. Hạn chế còn lại
Như V2: sai số chân > 0,07 mm; dây tác động một chiều lên phích; chưa có đo lực căng dây; dây 1,2 m. Tiếp xúc dây–cột 1e5 N/m vẫn cho phép xuyên ~0,2 mm khi căng mạnh (không nhìn thấy).

## 5. Chạy lại
```
cd data/scripts
python ur5_screen_V3.py --test
python ur5_screen_render_isaac_V3.py     # data/results/video_ur5_screen_V3.mp4
```
