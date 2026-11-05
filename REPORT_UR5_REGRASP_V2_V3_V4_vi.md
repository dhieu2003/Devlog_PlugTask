# So sánh V2, V3, V4: UR5 cắm phích rồi re-grasp dây

Bài toán: UR5 thật (không phải UR5e) với tay kẹp Robotiq 2F-85 nhặt phích ở cuối dây, luồn qua 3 cột, cắm vào ổ tường, thả, rồi quay lại gắp dây, đi sóng qua cột 1, khe 1-2, cột 2, khe 2-3 và đặt dây xuống sàn. Cuối cùng phích phải còn trong ổ.

Ba phiên bản được so sánh:

| Phiên bản | Tình trạng |
|---|---|
| **V2** | Lỗi khi gắp dây: cánh tay xoay thừa cả vòng, dây tuột khỏi tay kẹp khi qua khe 2-3 |
| **V3** | Hết hai lỗi trên, nhưng cổ tay xoay ở khe 2-3 làm phích dịch |
| **V4** (`V4full`) | Hoàn thiện, được chấp nhận |

Về nguồn số liệu: log gốc không đặt nhãn "V2" cho một bản cụ thể. Các lỗi của V2 ở đây (xoay thừa 286 độ, dây tuột ở khe 2-3) được log ghi cho bản gốc trước khi sửa ở V1, nên số liệu V2 lấy từ cột "bản gốc" trong log. V3 là bản sao của V1 (đã mang các sửa chữa đó) và V4 là V3 cộng thay đổi yaw cổ tay. Các script V2 và V3 cũ đã bị xóa khi dọn thư mục, nên đoạn code trong báo cáo lấy từ `V4full`, nơi các sửa chữa đó vẫn còn nguyên.

---

## 1. Bảng so sánh tổng quát

| Hạng mục | V2 | V3 | V4 |
|---|---|---|---|
| Dây trong tay kẹp qua khe 2-3 | tuột (khe kẹp đóng về 12,5 mm), có lần NaN | không rơi, lệch tối đa 17,9 mm | không rơi, lệch tối đa 6,5 mm |
| Khớp đế quay khi tiếp cận dây | 286 độ | 23 độ | 23 độ |
| Tổng góc đế quay trong re-grasp | 687 độ | 240 độ | 240 độ |
| Lực kẹp dây | 0,9 N·m, ma sát proxy 0,7 | 2,2 N·m, ma sát proxy 1,2 | 2,2 N·m, ma sát proxy 1,2 |
| Chọn nghiệm IK | cuRobo tự chọn | `ik_candidates`, chọn nghiệm ít quay nhất | như V3 |
| Yaw cổ tay khi đi sóng qua khe | có (-90 / +60 độ) | có (-90 / +60 độ), cổ tay 6 quay khoảng 106 độ vào khe 2-3 | **0 độ** (`--wave-yaw-deg 0`) |
| Phích dịch khi vào khe 2-3 | 1,2 đến 3,6 mm (không ổn định) | 3,05 mm | **0,79 mm** |
| Phích dịch sau khi đặt dây | | 3,88 mm | **0,48 mm** (tối đa 1,09 mm) |
| Hở vỏ-ổ cuối | | 3,74 mm | **0,84 mm** |
| Sai số chân cuối | 0,64 mm | 0,66 mm | 0,37 mm |
| Tiêu chí bài test | còn đòi 0,2 mm | còn đòi 0,2 mm (thất bại) | phích còn trong ổ và dây không tuột (đạt) |
| Dây rơi khỏi tay kẹp | có | không | không |

Số liệu V2 ở hàng "phích dịch" lấy từ các lần chạy của bản gốc (1,2 đến 3,6 mm giữa các lần, có lần 11,9 mm rồi NaN). Các ô để trống là log không ghi.

---

## 2. V2: hai lỗi khi gắp và đi sóng

### 2.1 Lỗi A: cánh tay xoay thừa một vòng

**Hiện tượng.** Khi tiếp cận dây để re-grasp, khớp đế quay 286 độ, cổ tay 6 quay 229 độ. Tổng góc đế quay trong cả pha re-grasp là 687 độ. Cánh tay vung lớn không cần thiết. Khi sửa lỗi này, log còn ghi một nhánh cuRobo vung đầu kẹp 3,04 m lên z = 1,14 m, giật dây và rút phích ra, làm mô phỏng nổ (NaN).

**Nguyên nhân.** Cùng một tư thế TCP có nhiều bộ góc khớp. Hai nguồn đa nghiệm:

- UR5 có nhiều nhánh IK (vai, khuỷu, cổ tay lật) khác nhau.
- Mỗi khớp có giới hạn $\pm 2\pi$, nên $q_j$ và $q_j \pm 2\pi$ cùng cho một tư thế. Một nghiệm có thể lệch đúng một vòng so với tư thế hiện tại.

Ở V2, bước `plan_pose` giao cho cuRobo tự giải IK và tự chọn nghiệm, không xét nghiệm đó cách tư thế hiện tại bao xa. Vì vậy có thể chọn nghiệm xa một vòng.

**Giải pháp.** Ba thay đổi:

1. Sinh nhiều nghiệm IK rồi sắp theo độ dịch khớp lớn nhất so với tư thế hiện tại $q_{ref}$:

$$
J(q) = \max_{i=1..6} \left| q_i - q_{ref,i} \right|
$$

Nghiệm có $J$ nhỏ nhất đứng đầu là nhánh "không quay vòng". cuRobo chỉ lập kế hoạch tới các nghiệm đầu danh sách, và bỏ qua nghiệm nào có $J > 200$ độ.

2. Mỗi nghiệm IK được tìm bằng bình phương tối thiểu có giảm chấn (DLS), khởi động từ nhiều điểm gần $q_{ref}$:

$$
\Delta q = J_a^{\top}\left(J_a J_a^{\top} + \lambda^2 I\right)^{-1} e,\qquad \lambda^2 = 10^{-3}
$$

   Ở đây $J_a$ là Jacobian 6x6 tính bằng sai phân hữu hạn, $e = [\,p_{tgt}-p\;;\; e_R\,]$ là sai số 6 chiều, và sai số hướng lấy từ phần phản đối xứng:

$$
e_R = \tfrac12\,\mathrm{vee}\!\left(R_{t}R_{c}^{\top} - \left(R_{t}R_{c}^{\top}\right)^{\top}\right)
$$

3. Khi về Home, đưa đích về dạng tương đương modulo $2\pi$ gần tư thế hiện tại nhất, để cổ tay không phải tháo xoắn:

$$
q_j' = \arg\min_{k\in\{-1,0,1\}} \left| q_j + 2\pi k - q_{cur,j} \right|,\qquad \left|q_j'\right| < 2\pi - 0.1
$$

Code (`ur5_regrasp_curobo_V4full.py`):

```python
def ik_candidates(p, r, q_ref, tries=120, seed=1):
    rng = np.random.default_rng(seed)
    sols = []
    def add(q0):
        q, err = G.ik(p, r, q0, iters=300)
        if err < 1e-4 and np.all(np.abs(q) < 2*np.pi - 0.05) \
           and not any(np.linalg.norm(q - s) < 1e-3 for s in sols):
            sols.append(q)
    for k in range(tries):                       # giai đoạn 1: nhiễu quanh q_ref
        add(q_ref if k == 0 else q_ref + rng.normal(0.0, 0.3 if k < tries//2 else 0.9, 6))
    if len(sols) < 4:                            # giai đoạn 2: khởi động đều trong [-pi, pi]
        rng2 = np.random.default_rng(seed + 1)
        for _ in range(2*tries):
            add(rng2.uniform(-np.pi, np.pi, 6))
    sols.sort(key=lambda q: float(np.max(np.abs(q - q_ref))))   # J(q) nhỏ nhất đứng đầu
    return sols

def unwrap_goal(q_goal, q_cur):
    out = np.array(q_goal, float)
    for j in range(len(out)):
        cands = [q_goal[j] + 2*np.pi*k for k in (-1, 0, 1)]
        cands = [c for c in cands if abs(c) < 2*np.pi - 0.1]
        out[j] = min(cands, key=lambda c: abs(c - q_cur[j]))
    return out
```

Ngoài ra, khi một chặng không đi được bằng đường thẳng thì chọn phương án cuRobo có đường đi đầu kẹp **ngắn nhất** (nhánh bị loại vung 3,04 m, nhánh được chọn chỉ 1,0 m, z tối đa 0,38 m), và thêm bước lùi sang bên 15 cm (`rg_clear`) trước khi lập đường về Home.

**Kết quả.**

| | V2 | V3 và V4 |
|---|---|---|
| Khớp đế quay khi tiếp cận | 286 độ | **23 độ** |
| Khớp cổ tay 6 khi tiếp cận | 229 độ | 171 độ (cần để định hướng tay kẹp) |
| Tổng góc đế trong re-grasp | 687 độ | **240 độ** |
| Đường về Home | cổ tay có thể tháo xoắn khoảng 300 độ | đường thẳng trong không gian khớp, tối đa 77 độ ở một khớp |

### 2.2 Lỗi B: dây tuột khỏi tay kẹp khi qua khe 2-3

**Hiện tượng.** Ở chặng từ cột 2 sang khe 2-3, dây tụt dần ra khỏi má kẹp: độ cao dây giảm từ z = 0,26 m xuống 0,02 m và khe kẹp đóng về 12,5 mm. Có lần mô phỏng NaN.

**Nguyên nhân (theo log).** Lực kẹp dây 0,9 N·m và hệ số ma sát của thân proxy 0,7 đều quá thấp, nên dây không giữ được khi bị kéo qua khe. Điều kiện không tuột là lực tiếp tuyến lên dây không vượt ma sát của hai má:

$$
F_{t} \le 2\,\mu\,F_n
$$

với $F_n$ là lực pháp tuyến mỗi má (do lực kẹp quyết định) và $\mu$ là ma sát giữa má và dây (ở đây là ma sát của thân proxy). Hai thay đổi nâng cả hai vế $\mu$ và $F_n$ cùng lúc.

**Giải pháp.**

- Lực kẹp dây 0,9 lên 2,2 N·m (`--cable-grip-effort`), ma sát proxy 0,7 lên 1,2.
- Thêm giám sát: đo khoảng cách giữa thân proxy và TCP, nếu lớn hơn 3 cm thì coi là dây đã tuột, và `--test` thất bại.

```python
# ma sát cao của đoạn dây bị kẹp (proxy)
pxc = newton.ModelBuilder.ShapeConfig(ke=CONTACT_KE, kd=0.0, mu=1.2, mu_torsional=MU_TORSION,
                                      margin=0.0, gap=0.0, density=0.0, is_visible=False)

# giám sát tuột dây
def hold_check(self, tag):
    pq  = self.s0.body_q.numpy()[self.proxy_body, :3].astype(np.float64)
    tcp, _ = G.fk(self.robot_q())
    d = float(np.linalg.norm(pq - tcp))          # d = || p_proxy - p_TCP ||
    self.hold_dist_max = max(self.hold_dist_max, d)
    if d > 0.03:                                  # > 3 cm: đã tuột
        self.cable_dropped = True
    return d
```

**Kết quả.** Dây không còn tuột trong V3 và V4. Độ lệch tối đa giữa dây và tâm kẹp: V3 17,9 mm, V4 6,5 mm (đều nhỏ hơn ngưỡng 3 cm của `hold_check`).

**Hạn chế của phân tích này.** Hai thay đổi (lực kẹp và ma sát) được đưa vào cùng lúc, nên chưa biết mỗi thay đổi đóng góp bao nhiêu. Lực dây lên phích đo được chỉ khoảng 0,4 đến 3,5 N, thấp hơn nhiều so với khả năng ma sát ước tính của hai má, nên phép cân bằng lực tĩnh ở trên chưa giải thích đầy đủ vì sao dây tuột ở V2. Muốn tách bạch cần chạy riêng từng thay đổi.

---

## 3. V3: cổ tay xoay ở khe 2-3 làm phích dịch

**Hiện tượng.** V3 hết hai lỗi của V2 và chạy trọn chuỗi, phích còn cắm. Nhưng phích bị dịch 3,05 mm khi vào khe 2-3 và 3,88 mm sau khi đặt dây, hở vỏ-ổ cuối 3,74 mm. Bài test cũ (đòi sai số chân 0,2 mm) thất bại.

**Nguyên nhân.** Các điểm đi sóng lấy từ đường vận chuyển lúc cắm, nơi tay có yaw $\psi = -90^\circ$ ở khe 1-2 và $+60^\circ$ ở khe 2-3. Hàm hướng tay là:

$$
R_{tool}(\psi,\theta) = R_z(\psi)\,R_y(\theta)\,R_0 ,\qquad \theta = 40^\circ\ (\text{tilt})
$$

Lúc cắm, phích do tay giữ và dây còn chùng, nên xoay yaw không sao. Lúc re-grasp thì khác: phích đã cố định trong ổ, tay kẹp dây ở cung 0,78 m trên tổng 1,6 m. Xoay yaw làm cổ tay 6 quay khoảng 106 độ ở chặng vào khe 2-3, vặn và kéo dây, và lực đó truyền về phích. Dây có độ cứng (E = 5·10⁶ Pa, bán kính 5 mm, 64 đoạn dài $\ell = 25$ mm):

$$
k_{stretch} = \frac{EA}{\ell} \approx 1.57\cdot 10^{4}\ \mathrm{N/m},\qquad
k_{bend} = \frac{EI}{\ell} = \frac{E\,\pi r^4/4}{\ell} \approx 0.098\ \mathrm{N\,m/rad}\ (\text{mỗi khớp})
$$

Bằng chứng nhân quả khá chắc: giữa V3 và V4 chỉ đổi yaw cổ tay (và tiêu chí test), nên mức phích dịch giảm là do bỏ yaw. Cơ chế cụ thể (vặn và kéo dây) là giải thích theo log, chưa được tách riêng bằng thí nghiệm khác.

**Giải pháp.** Bỏ yaw cổ tay trong lúc đi sóng khi re-grasp. Điểm đi sóng dùng yaw 0 thay cho yaw của đường vận chuyển (đường vận chuyển lúc cắm vẫn giữ yaw):

```python
# --wave-yaw-deg: mặc định 0 (V4); None thì dùng yaw gốc của đường vận chuyển
p.add_argument("--wave-yaw-deg", type=float, default=0.0)

fwd = G.weave_waypoints()[:6]    # post1, gap12 vào/ra, post2, gap23 vào/ra
wave = [(n, [float(v) for v in w],
         float(y if a.wave_yaw_deg is None else np.deg2rad(a.wave_yaw_deg)))
        for n, w, y in fwd]
```

Các lượt băng khe vẫn là đoạn thẳng Descartes ở cùng độ cao và hướng:

$$
p(s) = p_0 + s\,(p_1 - p_0),\qquad R(s) = \mathrm{Slerp}(R_0, R_1; s),\qquad s\in[0,1]
$$

Kết quả đo: khe 1-2 phích đi ở x = 0,530 m (tâm khe), z = 0,264 đến 0,267 m, khe 2-3 ở x = 0,797 m, z = 0,265 đến 0,268 m, đều thấp hơn đỉnh cột 0,30 m. cuRobo vẫn lập được kế hoạch không va chạm khi tay không xoay.

**Kết quả (V3 so với V4).**

| | V3 | V4 |
|---|---|---|
| Phích dịch khi vào khe 2-3 | 3,05 mm | **0,79 mm** |
| Phích dịch sau khi đặt dây | 3,88 mm | **0,48 mm** (tối đa 1,09 mm lúc re-grasp) |
| Dây lệch TCP tối đa | 17,9 mm | **6,5 mm** |
| Hở vỏ-ổ cuối | 3,74 mm | **0,84 mm** |

---

## 4. V4: bản hoàn thiện

V4 là V3 cộng hai thay đổi: yaw cổ tay khi đi sóng đặt bằng 0, và bài test bỏ dung sai chân 0,2 mm (plan mới `plans/UR5_regrasp.md` không còn quy định dung sai đó). Bài test chỉ kiểm tra hai điều: dây không tuột khỏi tay kẹp và phích còn trong ổ (hở vỏ-ổ nhỏ hơn nửa chiều dài lưỡi):

```python
assert not ex.cable_dropped, "the cable slipped out of the gripper during the wave"
assert gap < 0.5 * G.BLADE_LEN * 1000, "plug was pulled out of the socket"
assert args.tol is None or lateral < args.tol      # chỉ kiểm khi truyền --tol
```

Kết quả V4full: 959 s chạy thực, 72,2 s mô phỏng. Lần chạy lặp (`_V4full2`) cho kết quả gần như trùng: phích dịch 0,474 mm, hở 0,86 mm, lệch tối đa 1,07 mm. Video được chấp nhận: `video_ur5_regrasp_V4full.mp4`.

Trình tự V4: Home, tiếp cận, kẹp phích, nâng, vận chuyển qua 3 cột, cắm, thả, lùi, re-grasp dây (kẹp ở cung 0,78 m), sóng qua cột 1, khe 1-2, cột 2, khe 2-3, đặt dây thẳng hàng với hình chiếu của phích, về Home.

---

## 5. Vẫn còn tồn tại ở V4

1. Sai số ngang chân phích sau khi thả vẫn 0,37 mm, vượt 0,2 mm của plan cũ (nghiêng khoảng 2,5 độ, tiếp xúc penalty mềm ke = 1e5). Chỉ không còn là tiêu chí của bài test.
2. Đoạn dây trong tay kẹp là kinematic (một thân proxy), nên không có lực dây lên má kẹp. Ghép hai chiều đầy đủ chỉ ở phích.
3. Mỗi cấu hình chỉ chạy một lần; GPU không tất định nên kết quả lệch 0,01 đến 0,1 mm (bản cũ lệch vài mm).
4. Chưa kiểm tra bằng mắt va chạm giữa tay kẹp và cột khi tay không xoay.
5. Thử hạ độ cao đường đi xuống z = 0,20 m (để dây trông đi giữa các cột) thất bại ở bước lập kế hoạch (`cuRobo failed to plan 'weave leg 3.0 (-> post2)'`); nguyên nhân chưa kiểm chứng.

---

## 6. Cách chạy lại V4

```
cd data/scripts
python ur5_build_assets_V4.py                                  # một lần
REGRASP_TAG=_V4full3 python ur5_regrasp_V4full.py --test       # thẻ mới để không ghi đè kết quả
python ur5_regrasp_render_isaac_V4full.py --stride 3 \
    --traj ../results/ur5_regrasp_V4full3_traj.npz \
    --shapes ../results/ur5_regrasp_V4full3_shapes.pkl \
    --out ../results/video_ur5_regrasp_V4full3.mp4
```
