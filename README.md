Robot Arm 4DOF – Smooth Letter Drawing
======================================

Mô phỏng và điều khiển cánh tay robot 4 bậc tự do để vẽ chữ với quỹ đạo được làm mượt, có thể chạy thuần mô phỏng trên MATLAB hoặc gửi lệnh thật đến ESP32-C3 điều khiển 4 servo (3x MG996 + 1x SG90).
======================================
Nội dung chính
--------------
- Tạo quỹ đạo chữ từ bộ font Hershey, nội suy S-curve và lọc góc để giảm giật.
- Giải IK toàn bộ quỹ đạo bằng RigidBodyTree + ETS3 (Robotics Toolbox).
- Chuyển đổi góc khớp sang góc servo, kẹp theo giới hạn 0–180°.
- Kết nối ESP32-C3 qua serial, truyền lệnh SERVO tuyệt đối, có chế độ fallback mô phỏng nếu không kết nối được.
- Trực quan hóa 3D và bảng thông tin tiến trình.

Thư mục chính
-------------
- `main_draw_letter.m` – luồng chạy chính thực thi
- `config/RobotConfig.m` – tham số phần cứng, quỹ đạo, làm mượt, tốc độ, chữ cần vẽ
- `trajectory/` – tạo quỹ đạo chữ, nội suy S-curve, chuyển quỹ đạo sang pose
- `robot/` – dựng mô hình ETS3 và IK
- `motion/` – làm mượt góc khớp, ánh xạ sang góc servo
- `hardware/` – kết nối/giải phóng ESP32
- `visualization/` – tạo figure, cập nhật UI, in thống kê
- `ESP32-C3/remote_servo/` – firmware Arduino điều khiển 4 servo qua lệnh serial
- `compare_alg.m` – benchmark 2 solver IK (dùng `inverseKinematics` built-in) trên bộ 36 ký tự (A–Z, 0–9)
- `compare_alg_handle.m` – cùng benchmark nhưng dùng solver IK tự code

Yêu cầu môi trường
------------------
- MATLAB R2023+ khuyến nghị.
- Toolbox: Cần Robotics Toolbox for MATLAB (ETS3, mstraj, hershey) của Peter Corke.
- Arduino IDE 2.x hoặc PlatformIO (nạp code ESP32-C3).

Cách chạy
------------------------------------------
1) Mở MATLAB, đặt working dir tới gốc repo.
2) Điều chỉnh cổng COM trong `config/RobotConfig.m` (`esp32_com_port`) và các tham số mong muốn (`letter`, `scale`, `penUp`, `INTERP_FACTOR`, `USE_SCURVE`).
3) Chạy:
   ```
   main_draw_letter
   ```
4) Nếu MATLAB không kết nối được ESP32, script tự động chuyển sang chế độ mô phỏng và vẫn hiển thị quỹ đạo.

Quy trình nội bộ
----------------
- Tạo quỹ đạo chữ: `generateTrajectory` lấy stroke Hershey, thêm điểm pen-up, chạy `mstraj` với tốc độ/acc trong config.
- Làm mượt quỹ đạo: `applySmoothMotion` nội suy S-curve với `INTERP_FACTOR`, cập nhật `dt`.
- Chuyển quỹ đạo sang pose: `trajectoryToPoses` áp dụng offset và xoay đầu bút.
- IK toàn bộ đường đi: `solveInverseKinematics` dùng `inverseKinematics("RigidBodyTree")`, warm-start liên tiếp.
- Làm mượt góc khớp: `smoothJointAngles` dùng `movmean` giảm jerk.
- Ánh xạ góc servo: `convertToServoAngles` đổi rad→deg, đảo hướng khớp 2, lấy trị tuyệt đối khớp 4, kẹp 0–180°.
- Thực thi & hiển thị: `executeMotion` gửi lệnh SERVO (nếu có ESP32), vẽ trail, cập nhật UI, in thống kê (`printSummary`).

Cấu hình (RobotConfig)
---------------------------------
- Phần cứng: `esp32_com_port`, `esp32_baudrate`, `servo_min_angle/max_angle`, `joint_limits`.
- Làm mượt: `INTERP_FACTOR` (mặc định 3), `USE_SCURVE` (true).
- Quỹ đạo chữ: `letter` (mặc định 'B'), `scale`, `penUp`, `vmax`, `accTime`.
- Thời gian đồng bộ: `SERVO_MOVE_TIME` + `SAFETY_MARGIN` → `SYNC_DELAY`.

ESP32-C3 firmware (remote_servo.ino)
-------------------------------------
- Chân: Servo1=GPIO2 (MG996), Servo2=GPIO3 (MG996), Servo3=GPIO4 (MG996), Servo4=GPIO5 (SG90).
- PWM: MG996 1000–2000 µs, SG90 500–2400 µs.
- Lệnh serial (115200, kết thúc LF):
  - `TEST` – kiểm tra kết nối.
  - `START ABSOLUTE` – reset thống kê, bắt đầu nhận lệnh.
  - `SERVO,a1,a2,a3,a4` – đặt góc tuyệt đối 0–180°.
  - `STOP` – về vị trí an toàn, in thống kê.
  - `STATUS` / `STATS` / `CALIBRATE` / `TEST_JOINT,joint,angle` – tiện ích debug.
- Có deadband 0.5° để bỏ qua thay đổi nhỏ; cảnh báo khi Δ lớn (>20°).

Ghi chú an toàn
---------------
- Điều chỉnh `joint_limits` và dải PWM servo thực tế trước khi chạy robot thật.
- Kiểm tra dây nguồn servo riêng (MG996 cần dòng lớn), nối mass chung với ESP32.
- Bắt đầu với chữ nhỏ (`scale` thấp) và tăng dần.

Benchmark IK
------------
- Chạy `compare_alg.m` (solver MATLAB built-in) hoặc `compare_alg_handle.m` (solver tự code) để so sánh BFGSGradientProjection vs LevenbergMarquardt trên 36 ký tự; in bảng kết quả trung bình.