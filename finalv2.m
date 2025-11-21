function draw_letter_4dof()
% Vẽ chữ bằng robot 4DOF tự tạo (ETS3 + numerical IK)
% - Robot: 4 DOF, model từ ETS3 -> rigidBodyTree
% - Quỹ đạo: dùng font Hershey (RVC Toolbox)
% - IK: inverseKinematics, ràng buộc vị trí + pitch = 90 độ (underactuated)

close all; clear; clc;

%% ===== 1. Tạo robot 4DOF từ ETS3 =====
% Thông số link (m) – chỉnh theo robot của bạn
a1 = 0.12;    % link 1
a2 = 0.118;   % link 2
a3 = 0.083;   % cổ tay + bút
a4 = 0.045;   % link cố định vuông góc, 4.5 cm (m)

% ETS3: Rz q1 -> Ry q2 -> Tz a1 -> Ry q3 -> Tz a2 -> Ry q4 -> Tz a3 -> Tx a4
e = ETS3.Rz('q1') * ...
    ETS3.Ry('q2') * ...
    ETS3.Tz(a1)   * ...
    ETS3.Ry('q3') * ...
    ETS3.Tz(a2)   * ...
    ETS3.Ry('q4') * ...
    ETS3.Tz(a3)   * ...
    ETS3.Tx(a4);

arm = ets2rbt(e);          % chuyển sang rigidBodyTree
arm.DataFormat = "row";    % để cấu hình là row vector

figure;
show(arm, zeros(1,4));
title('Robot 4DOF từ ETS3');
axis equal; grid on;

eeName = arm.BodyNames{end};   % tên end-effector (link cuối)

%% ===== 2. Tạo quỹ đạo chữ bằng font Hershey =====
% Font Hershey có sẵn trong RVC Toolbox
load hershey          % biến hershey: cell array các ký tự

ch = '4';             % ký tự muốn vẽ: 'A','B','C', ...
B = hershey{ch};

% stroke: 2 hàng (x,y) trong range ~[0,1]  => scale xuống cho robot nhỏ
scale = 0.12;         % kích thước chữ (m), chỉnh tuỳ robot
p = [scale * B.stroke; zeros(1, size(B.stroke,2))];  % thêm z = 0

% Xử lý NaN: NaN phân tách các nét, ta giữ nguyên (x,y) nhưng nâng z
k = find(isnan(p(1,:)));
p(:,k) = p(:,k-1);    % copy điểm trước đó
penUp = 0.07;         % nâng bút lên 3cm giữa các nét
p(3,k) = penUp;

% Tham số trajectory
dt = 0.02;                 % bước thời gian (s)
vmax = [0.05 0.05 0.05];   % vận tốc tối đa các trục (m/s)
accTime = 0.2;             % thời gian tăng tốc

traj = mstraj(p(:,2:end)', vmax, [], p(:,1)', dt, accTime);
% traj: Nx3, mỗi hàng là [x y z] trong "frame viết"

figure;
plot3(traj(:,1), traj(:,2), traj(:,3), '.-');
xlabel('x'); ylabel('y'); zlabel('z');
grid on; axis equal;
title(['Quỹ đạo đầu bút cho chữ "', ch, '" (frame viết)']);

%% ===== 3. Chuyển quỹ đạo sang pose 4x4xN trong base robot =====
N = size(traj,1);
Tp = zeros(4,4,N);

% Offset đặt chữ trong không gian robot
% Bàn vẽ thấp hơn base robot 5.5 cm, dịch ra trước 15 cm
% Mặt phẳng vẽ ≈ mặt phẳng XY tại z = -0.055 (m)
T_offset = trvec2tform([0.15 0 -0.075]);   % [x y z]

% Orientation của đầu bút:
%  - Muốn pitch (quanh trục y) = +90 deg (vuông góc mặt phẳng vẽ)
%  - Cho phép xoay tự do quanh pháp tuyến (yaw, trục z)
% Dùng hàm chuẩn MATLAB: quay quanh trục y một góc +pi/2
% axang2tform([ux uy uz angle])
T_orient = axang2tform([0 1 0 pi/2]);
% Nếu thấy hướng ngược ý bạn (bút quay sang bên kia), đổi thành:
% T_orient = axang2tform([0 1 0 -pi/2]);

for i = 1:N
    % Từ điểm traj(i,:) (trong "frame chữ") sang transform
    T_traj = trvec2tform(traj(i,:));  % [x y z]

    % Pose end-effector trong base robot:
    %   base --T_offset--> gốc chữ --T_traj--> điểm trên chữ --T_orient--> khung tool
    Tp(:,:,i) = T_offset * T_traj * T_orient;
end

%% ===== 4. Numerical IK cho robot 4DOF (underactuated) =====
ik = inverseKinematics("RigidBodyTree", arm);

% Weights: [rx ry rz tx ty tz]
%   -> [0 1 0 1 1 1]:
%      + ry    : ưu tiên bám pitch = 90°
%      + rx,rz : cho phép linh hoạt hơn (dễ tìm nghiệm với 4DOF)
%      + tx,ty,tz: bám vị trí quỹ đạo
weights = [0 1 0 1 1 1];

% Cấu hình ban đầu: home (tất cả 0)
cfg0 = arm.homeConfiguration;
nJoints = numel(cfg0);
qj = zeros(N, nJoints);    % quỹ đạo joint space

for i = 1:N
    [cfgSol, solInfo] = ik(eeName, Tp(:,:,i), weights, cfg0);

    % Lưu nghiệm
    qj(i,:) = cfgSol;

    % Dùng nghiệm hiện tại làm guess cho bước sau để quỹ đạo mượt
    cfg0 = cfgSol;

    % Nếu muốn debug thêm:
    % fprintf("Step %d: %s, err=%.2e\n", i, solInfo.Status, solInfo.PoseErrorNorm);
end

% %% ===== 5. Mô phỏng robot vẽ chữ (vẽ bằng ETS3 cho đẹp) =====
% figure;
% 
% % Vẽ robot ETS3 ở cấu hình đầu tiên
% e.plot(qj(1,:));       % không có output
% ax = gca;
% view(ax, 135, 25);
% grid(ax, 'on');
% axis(ax, 'equal');
% title(ax, ['Robot 4DOF vẽ chữ "', ch, '" (IK = rigidBodyTree, vẽ = ETS3)']);
% hold(ax, 'on');
% 
% % Vẽ quỹ đạo mong muốn (đường tím)
% X = squeeze(Tp(1,4,:));
% Y = squeeze(Tp(2,4,:));
% Z = squeeze(Tp(3,4,:));
% plot3(ax, X, Y, Z, 'm-', 'LineWidth', 1.5);
% 
% % Vệt bút thực tế
% trail = animatedline('Parent', ax, 'LineWidth', 2);
% 
% % Tốc độ animation
% % r = rateControl(1/dt);
% r = rateControl(30);
% 
% for i = 1:N
%     % Animate robot ETS3 theo nghiệm IK
%     e.animate(qj(i,:));       % robot chuyển động như teach
% 
%     % Lấy vị trí tool từ rigidBodyTree
%     Tee = getTransform(arm, qj(i,:), eeName);
%     addpoints(trail, Tee(1,4), Tee(2,4), Tee(3,4));
% 
%     drawnow limitrate;
%     r.waitfor;
% end
% 
% disp('Hoàn thành mô phỏng vẽ chữ.');

%% ===== 5. Mô phỏng + LOG GÓC SERVO =====
figure;

e.plot(qj(1,:));
ax = gca;
view(ax, 135, 25);
grid(ax, 'on');
axis(ax, 'equal');
title(ax, ['Robot 4DOF vẽ chữ "', ch, '"']);
hold(ax, 'on');

% Vẽ quỹ đạo mong muốn
X = squeeze(Tp(1,4,:));
Y = squeeze(Tp(2,4,:));
Z = squeeze(Tp(3,4,:));
plot3(ax, X, Y, Z, 'm--', 'LineWidth', 1.5);

trail = animatedline('Parent', ax, 'Color','b','LineWidth', 2);

r = rateControl(30);  % 30 Hz

fprintf('Bắt đầu vẽ chữ "%s" - Tổng %d điểm\n', ch, N);
fprintf('%5s %8s %8s %8s %8s\n', 'Step', 'q1(°)', 'q2(°)', 'q3(°)', 'q4(°)');
fprintf('%s\n', repmat('-',1,50));

for i = 1:N
    e.animate(qj(i,:));

    Tee = getTransform(arm, qj(i,:), eeName);
    addpoints(trail, Tee(1,4), Tee(2,4), Tee(3,4));

    % ========== LOG GÓC SERVO ==========
    q_deg = rad2deg(qj(i,:));   % đổi ra độ
    fprintf('%5d %8.2f %8.2f %8.2f %8.2f\n', i, q_deg(1), q_deg(2), q_deg(3), q_deg(4));
    % ===================================

    drawnow limitrate;
    r.waitfor;
end

disp('Hoàn thành vẽ chữ và đã log toàn bộ góc servo!');
end
