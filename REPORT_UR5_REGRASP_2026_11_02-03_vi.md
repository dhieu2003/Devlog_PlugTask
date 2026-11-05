# Báo cáo 2026-11-02 và 2026-11-03: UR5 cắm phích và re-grasp dây

Bài toán: UR5 thật (không phải UR5e) gắn tay kẹp Robotiq 2F-85 nhặt phích NEMA 5-15P ở cuối một sợi dây, luồn qua ba cột, cắm vào ổ trên tường, thả phích, rồi quay lại gắp dây, đưa dây đi theo đường sóng qua các cột và đặt xuống sàn. Cuối cùng phích phải còn trong ổ. Phích và dây được mô phỏng hai chiều (phích kéo dây, dây kéo ngược phích), độ cứng dây tính theo mô-đun Young.

Bộ mô phỏng: Newton/MuJoCo (robot, tay kẹp, phích, cột, tường, ổ cứng), Newton/VBD (dây), cuRobo (quỹ đạo không va chạm, chạy tiến trình riêng), Isaac Sim (dựng video).

Mục lục: 1. Tóm tắt, 2. Diễn biến, 3. Lỗi (lỗi, nguyên nhân, giải pháp, kết quả), 4. Kết quả cuối, 5. Việc còn mở, 6. Ghi chú về quy trình, 7. Cách chạy lại.

---

## 1. Tóm tắt

| | Ngày 2/11 | Ngày 3/11 |
|---|---|---|
| Điểm xuất phát | Các bản trước cắm được phích; chưa re-grasp dây | Chuỗi đầy đủ chạy được nhưng sóng dây kéo phích dịch 3 mm ở khe 2-3 |
| Kết quả chính | Chuỗi đầy đủ chạy: cắm, thả, gắp dây ở cung 0,78 m, sóng qua cột 1, khe 1-2, cột 2, khe 2-3, đặt dây dọc theo đường y của phích, về Home | Bỏ xoay cổ tay khi đi sóng: phích dịch ở khe 2-3 giảm từ 3,05 mm xuống 0,79 mm. Bản được chấp nhận: V4full |
| Phích còn trong ổ lúc cuối | Có (dịch 1,2 đến 3,6 mm, thay đổi theo lần chạy) | Có (dịch 0,47 đến 0,48 mm, tối đa 1,07 đến 1,09 mm) |
| Dây tuột khỏi tay kẹp | Đã sửa ở V1 | Không |
| Chưa đạt | Sai số ngang chân phích sau khi thả 0,43 đến 0,68 mm (giới hạn 0,2 mm) | Như cũ; plan mới đã bỏ kiểm tra 0,2 mm khỏi bài test |

Lưu ý tên gọi: "V5" trong log ngày 2/11 là mô hình tiếp điểm đồng thau trong ổ, đã bị bỏ theo yêu cầu. "V5" trong log ngày 3/11 là một thí nghiệm khác (độ cao đường đi 0,20 m). Hai thứ này không liên quan nhau.

---

## 2. Diễn biến

**Ngày 2/11**
1. Bản gốc `ur5_regrasp`: bố cục mới (cột R = 5 cm cao 30 cm, robot cách cột 1 là 0,40 m, ổ lồi), ổ cứng không có lá lò xo, dây E = 5e6 Pa, tăng lực kẹp, thêm `regrasp_phase`, thêm thân proxy cứng cho đoạn dây bị kẹp.
2. Cập nhật buổi chiều: băng khe bằng đường thẳng Descartes, quét 9 bố cục cột/tường, giãn cột ra 0,26 m.
3. Chạy cuối của bản gốc: sóng qua cả hai khe, dây 1,6 m, đặt dây dọc theo đường của phích.
4. V1: dây không còn tuột ở khe 2-3, cánh tay bớt xoay thừa.

**Ngày 3/11**
1. V3 (bản sao của V1) chạy toàn chuỗi: phích còn cắm nhưng assert 0,2 mm cũ không đạt.
2. V4 / V4full: yaw cổ tay khi đi sóng đặt bằng 0, bài test rút gọn còn "phích còn trong ổ và dây không tuột". Chạy xong và được chấp nhận.
3. Thí nghiệm V5 (độ cao đường đi 0,20 m): thất bại ở bước lập kế hoạch.
4. Render V6: thêm tùy chọn ẩn đường quỹ đạo màu xanh.

---

## 3. Lỗi: lỗi, nguyên nhân, giải pháp, kết quả

### Ngày 2/11

| # | Lỗi | Nguyên nhân | Giải pháp | Kết quả |
|---|---|---|---|---|
| 1 | Phích kẹt ở độ sâu khoảng 9 mm và xoay 13 độ trong má kẹp lúc cắm | Cắm cần lực khoảng 36 N trở lên, và ma sát của má ở lực kẹp cũ (1,8 N·m) không đủ giữ phích (log gốc ghi khả năng ma sát khoảng 45 N; các số liệu được chép lại từ log, chưa tính lại) | Tăng lực kẹp lên 3,2 N·m (khoảng 55 N mỗi má). Khi kẹp dây dùng giá trị riêng (`--cable-grip-effort`, 0,9 rồi 2,2) | Phích cắm hết cỡ |
| 2 | cuRobo không lập được kế hoạch tới điểm kẹp ở cung 0,6 m | Điểm đó nằm sát cột 1 | Điểm kẹp phải nằm trên sàn và cách trục mọi cột ít nhất 13 cm (sau đó 10 cm) | Lập được kế hoạch |
| 3 | Tay đập vào phích và kéo lệch 7 mm lúc đi sóng | Đường sóng có "post3", chính là chỗ phích đang cắm | Bỏ post3 khỏi đường sóng | Hết va chạm |
| 4 | Sóng qua khe 2-3 với điểm kẹp ở cung 0,72 m: dây gần căng, phích bị rút từ 0,2 mm lên 14 mm rồi NaN | Giới hạn hình học: đường sóng đầy đủ dài hơn sợi dây 1,2 m (một đầu cố định ở phích) | Tăng dây lên 1,6 m (64 đoạn); điểm kẹp là điểm hợp lệ xa nhất trên dây (cung 0,78 m, đoạn thứ 32); sóng: cột 1, khe 1-2, cột 2, khe 2-3, rồi thẳng theo đường y của phích và đặt dây thẳng hàng với hình chiếu của phích xuống sàn | Sóng qua cả hai khe chạy xong |
| 5 | IK của planner thất bại ngẫu nhiên, kết quả khác nhẹ giữa các lần chạy | Hạt giống IK ngẫu nhiên | Tăng số lần thử lên 200 | Lập kế hoạch ổn định hơn |
| 6 | Phích đi vòng qua đỉnh cột ở các khe (z lên tới 0,32 m, đỉnh cột 0,30 m; x lên tới 0,557 m, tâm khe 0,51 m) | cuRobo nội suy trong không gian khớp nên đường TCP bị cong | Mọi lượt băng khe (gap_in sang gap_out) là đoạn thẳng Descartes bằng IK, cùng yaw và cùng độ cao (`is_cross`); cột được mô tả cho cuRobo bằng các hộp mảnh nằm trong hình trụ | Khe 1-2: x = 0,530 m (tâm khe), z = 0,264 đến 0,267 m. Khe 2-3: x = 0,797 m, z = 0,265 đến 0,268 m. Tất cả thấp hơn đỉnh cột |
| 7 | Khoảng hở âm khi cột cách nhau 0,22 m (tay chạm cột 2,0 mm ở khe 1-2 và 12,8 mm ở khe 2-3) | Khe quá hẹp so với tay kẹp rộng 89 mm | Quét 9 bố cục (bảng dưới). Chọn cột cách nhau 0,26 m, khe tường 0,08 m | Vận chuyển qua 3 cột và cắm thành công |
| 8 | Re-grasp không lập được đường tiếp cận tới điểm (0,234; 0,105) | Điểm quá gần chân robot, cuRobo không tìm ra đường | Thêm bộ lọc điểm kẹp có bán kính r > 0,30 m (điểm (0,294; 0,095) lập kế hoạch riêng lẻ thành công rồi dùng) | Lập được kế hoạch re-grasp |
| 9 | Phát lại ngược đường đi để về làm phích bị rút ra 11,9 mm rồi NaN trong một lần chạy | Dây kéo phích khi chạy ngược | Bỏ đường về phát lại ngược; về bằng bước tránh cột rồi đường thẳng trong không gian khớp | Lần chạy cuối không NaN |
| 10 | Dây tuột khỏi tay kẹp trước khi qua khe 2-3 (z từ 0,26 xuống 0,02 m, khe kẹp đóng về 12,5 mm) | Lực kẹp dây 0,9 N·m và ma sát proxy 0,7 quá thấp | Lực kẹp dây 0,9 lên 2,2 N·m, ma sát proxy 0,7 lên 1,2, thêm `hold_check` giám sát khoảng cách proxy tới TCP (cảnh báo trên 3 cm, `--test` thất bại nếu dây tuột) | Dây không còn tuột (lệch tối đa khỏi tâm kẹp 21 mm) |
| 11 | Khớp đế quay 286 độ khi tới gắp dây; tổng góc đế quay trong pha re-grasp 687 độ | cuRobo tự chọn nghiệm IK cho `plan_pose`, có thể lệch một vòng đầy | `ik_candidates` sinh nhiều nghiệm IK, sắp theo độ dịch khớp lớn nhất, cuRobo chỉ lập kế hoạch tới các nghiệm gần nhất; về Home dùng đích đã "unwrap" và thử đường thẳng trong không gian khớp trước | Đế quay khi tiếp cận 23 độ, tổng góc đế 240 độ, đường về tối đa 77 độ ở một khớp |
| 12 | Mô phỏng nổ (NaN): một nhánh cuRobo vung đầu kẹp 3,04 m lên z = 1,14 m, giật dây và rút phích ra | Nhánh được chọn mà không xét độ dài đường đi | Trong các phương án của cuRobo chọn đường đi đầu kẹp ngắn nhất | Nhánh được chọn dài 1,0 m, z tối đa 0,38 m |
| 13 | cuRobo báo "Start or End state in collision" ở bước tiếp cận và đường về | Điểm xuất phát của đường về bị coi là va chạm | Chọn nhánh IK gần, thêm bước lùi sang bên 15 cm khỏi cột (`rg_clear`) trước khi lập đường về | Lập được kế hoạch |
| 14 | Lỗi cú pháp: dùng `or` với mảng numpy | Lỗi lập trình | Đã sửa | |

Bảng quét bố cục (khoảng hở nhỏ nhất của tay kẹp và cánh tay tới cột, mm):

| Cột cách (m) | Khe tường (m) | Khe 1-2 / khe 2-3 | Cắm (tay tới cột 3) |
|---|---|---|---|
| 0,22 (cũ) | 0,08 | -2,0 / -12,8 (chạm cột) | +22 |
| **0,26** | **0,08** | **+18 / +2,1** | **+1,2** |
| 0,27 | 0,08 | +23 / +6,3 | -27,6 |
| 0,26 | 0,09 đến 0,12 | +18 / +2,1 | -12 đến -40 (tay đâm vào cột 3) |
| 0,26 | 0,14 | UR5 không với tới | |

Lưu ý: giãn cột làm đường dây dài hơn nên không tạo thêm độ chùng.

### Ngày 3/11

| # | Lỗi | Nguyên nhân | Giải pháp | Kết quả |
|---|---|---|---|---|
| 15 | Phích dịch 3,05 mm ở khe 2-3 và 3,88 mm sau khi đặt dây | Cổ tay xoay yaw -90 / +60 độ ở các khe (lấy từ đường cắm) làm dây bị kéo, vặn, trong khi phích đã cố định trong ổ và tay kẹp dây ở cung 0,78 m trên tổng 1,6 m. Cổ tay 6 quay khoảng 106 độ ở chặng vào khe 2-3 | `--wave-yaw-deg 0` (mặc định ở V4): không xoay cổ tay trong lúc đi sóng. cuRobo vẫn lập được kế hoạch không va chạm | Phích dịch ở khe 2-3: 3,05 mm xuống 0,79 mm. Sau khi đặt dây: 3,88 mm xuống 0,48 mm. Dây lệch TCP: 17,9 mm xuống 6,5 mm. Hở vỏ-ổ cuối: 3,74 mm xuống 0,84 mm |
| 16 | `--test` thất bại ở dung sai chân 0,2 mm dù phích vẫn còn cắm | Plan mới (`plans/UR5_regrasp.md`) không còn quy định dung sai chân | `--tol` tắt mặc định; bài test chỉ kiểm tra phích còn trong ổ (hở vỏ nhỏ hơn 9 mm) và dây không tuột | Bài test đạt |
| 17 | V5 (độ cao đường đi WEAVE_Z từ 0,27 xuống 0,20 m, để dây rõ ràng đi giữa các cột) thất bại: `cuRobo failed to plan 'weave leg 3.0 (-> post2)'` | Chưa kiểm chứng. Giả thuyết: ở độ cao thấp, thân tay kẹp và cẳng tay sát cột 2 hơn (log ngày 2/11 từng đo khoảng hở chỉ 30 mm ở z = 0,17 m lúc cắm) | Chưa sửa. Không dùng bản này | Còn mở |
| 18 | Đường quỹ đạo màu xanh vẽ đè lên dây ở giây 59 đến 62 (pha `rg_lower`), dây trông mờ dần | Bản render vẽ quỹ đạo của phích bằng màu xanh | Render V6 với `--no-trajectory` | Dữ liệu dây ở giai đoạn này liên tục (29 đến 35 đoạn trong khe 1-2, không NaN, không nhảy); video đủ khung hình |
| 19 | Render V6 không chạy được | Nó import `ur5_regrasp_geom_V4`, file này đã bị xóa khi dọn thư mục | Chưa sửa | Còn mở |

---

## 4. Kết quả cuối

**Ngày 2/11, bản gốc, lần chạy cuối (94,5 s mô phỏng, 1292 s chạy thực)**

| Hạng mục | Giá trị |
|---|---|
| Cắm, trước khi thả | sai số chân 0,112 / 0,086 mm (giới hạn 0,2 mm, đạt) |
| Cắm, sau khi thả | 0,441 / 0,418 mm (không đạt) |
| Re-grasp | phích dịch tối đa 1,21 mm, cuối 1,27 mm, hở vỏ-ổ 1,29 mm, còn cắm |
| Sai số chân cuối bài test | 0,638 mm, nghiêng 4,2 độ |
| Dao động giữa các lần chạy (cùng cấu hình, GPU không tất định) | phích dịch 1,2 đến 3,6 mm; một lần 11,9 mm rồi NaN (khi dùng đường về phát lại ngược) |

**Ngày 2/11, V1 (95,1 s mô phỏng, chạy tiếp từ trạng thái lưu trước khi cắm, nên video chỉ gồm đoạn cắm và re-grasp)**

| | Bản gốc | V1 |
|---|---|---|
| Dây trong tay kẹp qua khe 2-3 | tuột, có lần NaN | không rơi, lệch tối đa 21 mm |
| Khớp đế quay khi tiếp cận | 286 độ | 23 độ |
| Tổng góc đế quay trong re-grasp | 687 độ | 240 độ |
| Phích dịch tối đa | 1,2 đến 3,6 mm | 4,0 mm (còn cắm, hở 3,76 mm) |
| Sai số chân cuối | 0,64 mm | 0,68 mm (vẫn trên 0,2 mm) |

**Ngày 3/11, V3 (xoay cổ tay) và V4full (không xoay cổ tay)**

| | V3 | V4full |
|---|---|---|
| Phích dịch khi vào khe 2-3 | 3,05 mm | 0,79 mm |
| Phích dịch sau khi đặt dây | 3,88 mm | 0,48 mm (tối đa 1,09 mm lúc re-grasp) |
| Dây lệch TCP tối đa | 17,9 mm | 6,5 mm |
| Hở vỏ-ổ cuối | 3,74 mm | 0,84 mm |
| Sai số chân cuối | 0,66 mm | 0,37 mm (không còn là tiêu chí) |
| Dây rơi | không | không |
| Kiểm tra cuối (phích còn trong ổ) | đạt (assert 0,2 mm cũ thất bại) | đạt |

V4full: 959 s chạy thực, 72,2 s mô phỏng. Lần chạy lặp (`_V4full2`) cho kết quả gần như trùng (dịch 0,474 mm, hở 0,86 mm, lệch tối đa 1,07 mm).

Video được chấp nhận: `video_ur5_regrasp_V4full.mp4`.

---

## 5. Việc còn mở

1. Sai số ngang chân phích sau khi thả là 0,37 đến 0,68 mm, vượt 0,2 mm của plan cũ. Nguyên nhân: tiếp xúc penalty mềm (ke = 1e5) và phích nghiêng 2,5 đến 4 độ. Hướng thử: tăng ke, vát miệng ổ, má kẹp có rãnh V.
2. Đoạn dây trong tay kẹp là kinematic (một thân proxy), nên không có lực dây lên má kẹp. Ghép hai chiều đầy đủ chỉ có ở phích.
3. Mỗi cấu hình chỉ chạy một lần; GPU không tất định nên kết quả lệch 0,01 đến 0,1 mm (và vài mm ở các bản trước).
4. Cột có R = 5 cm theo `plans/UR5_regrasp.md`, trong khi bài screen trước dùng 2,5 cm.
5. Chưa kiểm tra bằng mắt va chạm giữa tay kẹp và cột khi tay không xoay.
6. Việc tiếp theo: chọn hướng cho độ cao đường đi (z = 0,22 đến 0,24 m, hoặc chỉ hạ z lúc băng khe), sửa render V6 để import `ur5_regrasp_geom_V4full`, kẹp dây gần đuôi hoặc dùng dây 2 m để có thêm độ chùng. Bài học: muốn có thêm độ chùng thì kẹp xa phích, không phải gần phích.

---

## 6. Ghi chú về quy trình (sai sót và cách xử lý)

- Đề xuất "kẹp dây gần phích để chùng thêm" là sai và đã được đảo lại (kẹp xa mới chùng thêm).
- `ur5_regrasp_V3.py` từng bị sửa tại chỗ, trái quy tắc "mỗi lần chỉnh là một phiên bản mới"; đã hoàn tác (khôi phục dung sai 0,2 mm, bỏ tùy chọn yaw) và thay đổi được chuyển sang V4.
- Một vòng lặp chờ nền khớp chính dòng lệnh của nó nên không bao giờ kết thúc; đã thay bằng chờ theo mã tiến trình. Không file nào bị ảnh hưởng.
- `ur5_regrasp_V4full_traj.npz` (ghi lúc 22:03) mới hơn video được chấp nhận (20:02), nên đã bị một lần chạy sau ghi đè. So 1444 khung của video với bản dựng lại từ quỹ đạo hiện có cho sai khác trung bình 0,16/255 (tối đa 0,67, không khung nào trên 2), nên rất có thể là cùng quỹ đạo, nhưng không có bằng chứng chắc chắn. Mỗi lần chạy nên dùng một `REGRASP_TAG` mới để khỏi ghi đè.
- Khi dọn thư mục, `ur5_regrasp_V4full.py` và video V4full từng bị chuyển nhầm vào Thùng rác và đã được khôi phục.

---

## 7. Cách chạy lại

```
cd data/scripts
python ur5_build_assets_V4.py                                  # một lần
REGRASP_TAG=_V4full3 python ur5_regrasp_V4full.py --test       # thẻ mới để không ghi đè kết quả
python ur5_regrasp_render_isaac_V4full.py --stride 3 \
    --traj ../results/ur5_regrasp_V4full3_traj.npz \
    --shapes ../results/ur5_regrasp_V4full3_shapes.pkl \
    --out ../results/video_ur5_regrasp_V4full3.mp4
```

Một số tùy chọn hữu ích: `--save-state` / `--resume` (chỉ chạy đoạn cắm và re-grasp từ trạng thái lưu trước khi cắm), `--regrasp-near-start`, `--wave-yaw-deg`, `--cable-grip-effort`, `--regrasp-arc`.
