function draw_letter_4dof_final()
% Vẽ chữ bằng robot 4DOF với chuyển động mượt mà
% PHIÊN BẢN HOÀN THIỆN - SỬA LỖI HANDLE HOÀN TOÀN

close all; clear; clc;

%% ===== CẤU HÌNH =====
esp32_com_port = '/dev/ttyUSB0';
esp32_baudrate = 115200;
servo_min_angle = 0;
servo_max_angle = 180;
joint_limits = [-90 90; -90 90; -90 90; -90 90];

% THÔNG SỐ CHUYỂN ĐỘNG
SERVO_MOVE_TIME = 0.15;
SAFETY_MARGIN = 0.02;
SYNC_DELAY = SERVO_MOVE_TIME + SAFETY_MARGIN;

% THÔNG SỐ SMOOTH
INTERP_FACTOR = 3;
USE_SCURVE = true;

esp32_serial = [];

%% ===== 1. TẠO ROBOT =====
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('🤖 ROBOT 4DOF - SMOOTH MOTION SYSTEM\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n');

fprintf('=== KHỞI TẠO ROBOT ===\n');

a1 = 0.12;    % link 1
a2 = 0.118;   % link 2
a3 = 0.04;   % cổ tay + bút
a4 = 0.045;
e = ETS3.Rz('q1') * ETS3.Ry('q2') * ETS3.Tz(a1) * ...
    ETS3.Ry('q3') * ETS3.Tz(a2) * ETS3.Ry('q4') * ...
    ETS3.Tz(a3) * ETS3.Tx(a4);

arm = ets2rbt(e);
arm.DataFormat = "row";
eeName = arm.BodyNames{end};

fprintf('✓ Links: %.3f, %.3f, %.3f, %.3f m\n', a1, a2, a3, a4);

%% ===== 2. TẠO QUỸ ĐẠO =====
fprintf('\n=== TẠO QUỸ ĐẠO ===\n');
load hershey
ch = 'Z';
B = hershey{ch};

scale = 0.15;
p = [scale * B.stroke; zeros(1, size(B.stroke,2))];

k = find(isnan(p(1,:)));
p(:,k) = p(:,k-1);
penUp = 0.03;
p(3,k) = penUp;

dt = SYNC_DELAY;
vmax = [0.05 0.05 0.05];
accTime = 0.2;
traj = mstraj(p(:,2:end)', vmax, [], p(:,1)', dt, accTime);

fprintf('✓ Chữ: "%c", waypoints: %d\n', ch, size(traj,1));

%% ===== 3. S-CURVE INTERPOLATION =====
fprintf('\n=== S-CURVE SMOOTHING ===\n');

if USE_SCURVE && size(traj,1) > 2
    traj_smooth = [];
    for i = 1:size(traj,1)-1
        p1 = traj(i,:);
        p2 = traj(i+1,:);
        for j = 0:INTERP_FACTOR
            t = j / INTERP_FACTOR;
            s = 3*t^2 - 2*t^3;
            p_interp = p1 + s * (p2 - p1);
            traj_smooth = [traj_smooth; p_interp];
        end
    end
    traj_smooth = [traj_smooth; traj(end,:)];
    
    fprintf('✓ %d -> %d points (x%d smooth)\n', ...
            size(traj,1), size(traj_smooth,1), INTERP_FACTOR);
    traj = traj_smooth;
    dt = dt / (INTERP_FACTOR + 1);
    fprintf('✓ dt = %.4fs\n', dt);
end

%% ===== 4. POSE TRANSFORMATION =====
fprintf('\n=== POSE TRANSFORMATION ===\n');
N = size(traj,1);
Tp = zeros(4,4,N);

T_offset = trvec2tform([0.15 0 -0.055]);
T_orient = axang2tform([0 1 0 pi/2]);

for i = 1:N
    T_traj = trvec2tform(traj(i,:));
    Tp(:,:,i) = T_offset * T_traj * T_orient;
end

fprintf('✓ Generated %d poses\n', N);

%% ===== 5. INVERSE KINEMATICS =====
fprintf('\n=== INVERSE KINEMATICS ===\n');
ik = inverseKinematics("RigidBodyTree", arm);
weights = [0 1 0 1 1 1];

cfg0 = arm.homeConfiguration;
nJoints = numel(cfg0);
qj = zeros(N, nJoints);

fprintf('Computing IK');
success_count = 0;
for i = 1:N
    [cfgSol, solInfo] = ik(eeName, Tp(:,:,i), weights, cfg0);
    if strcmp(solInfo.Status, 'success')
        success_count = success_count + 1;
    end
    qj(i,:) = cfgSol;
    cfg0 = cfgSol;
    if mod(i, 50) == 0
        fprintf('.');
    end
end
fprintf(' Done!\n✓ Success: %d/%d (%.1f%%)\n', success_count, N, success_count/N*100);

%% ===== 6. JOINT SMOOTHING =====
fprintf('\n=== JOINT SMOOTHING ===\n');

if N > 5
    qj_smooth = qj;
    window_size = 3;
    
    for j = 1:nJoints
        qj_smooth(:,j) = movmean(qj(:,j), window_size);
    end
    
    jerk_before = sum(abs(diff(diff(diff(qj)))));
    jerk_after = sum(abs(diff(diff(diff(qj_smooth)))));
    
    fprintf('✓ Jerk reduction: %.1f%%\n', (1-jerk_after/jerk_before)*100);
    qj = qj_smooth;
end

%% ===== 7. SERVO ANGLES =====
fprintf('\n=== SERVO CONVERSION (NO MAPPING) ===\n');

qj_degrees = rad2deg(qj);      % Góc mô phỏng → độ
qj_servo = zeros(size(qj_degrees));

for i = 1:nJoints
    
    % ===== Điều kiện từng servo =====
    switch i
        case 1
            % Servo 1: giữ nguyên
            qj_servo(:,i) = qj_degrees(:,i);

        case 2
            % Servo 2: đảo hướng = 180 - góc
            qj_servo(:,i) = 180 - qj_degrees(:,i);

        case 3
            % Servo 3: giữ nguyên
            qj_servo(:,i) = qj_degrees(:,i);

        case 4
            % Servo 4: lấy trị tuyệt đối
            qj_servo(:,i) = abs(qj_degrees(:,i));
    end

    % Clamp vào giới hạn servo
    qj_servo(:,i) = max(servo_min_angle, min(servo_max_angle, qj_servo(:,i)));
end

fprintf('✓ Converted to servo angles (simple mode)\n');

%% ===== 8. ESP32 CONNECTION =====
fprintf('\n=== ESP32 CONNECTION ===\n');

try
    if ~isempty(instrfind)
        fclose(instrfind);
        delete(instrfind);
    end
    
    fprintf('Connecting...');
    esp32_serial = serialport(esp32_com_port, esp32_baudrate);
    configureTerminator(esp32_serial, "LF");
    pause(2);
    
    writeline(esp32_serial, "TEST");
    pause(0.5);
    
    if esp32_serial.NumBytesAvailable > 0
        response = readline(esp32_serial);
        fprintf(' OK\n✓ %s\n', strtrim(response));
    else
        fprintf(' OK\n');
    end
    
catch ME
    fprintf(' FAILED\n⚠️  %s\n➜ Simulation mode\n', ME.message);
    esp32_serial = [];
end

%% ===== 9. VISUALIZATION SETUP =====

fprintf('\n=== VISUALIZATION SETUP ===\n');

fig = figure('Position', [100 100 1400 700], 'Name', 'Robot Smooth Motion');

% Robot subplot
e.plot(qj(1,:));       % không có output
ax = gca;
hold(ax_robot, 'on');
view(ax_robot, 135, 25);
grid(ax_robot, 'on');
axis(ax_robot, 'equal');
title(ax_robot, sprintf('Robot 4DOF - "%c" (SMOOTH)', ch));

% Desired path
X = squeeze(Tp(1,4,:));
Y = squeeze(Tp(2,4,:));
Z = squeeze(Tp(3,4,:));
plot3(ax_robot, X, Y, Z, 'm--', 'LineWidth', 1, 'DisplayName', 'Target');

% Trail
trail = animatedline('Parent', ax_robot, 'LineWidth', 2.5, ...
                     'Color', 'red', 'DisplayName', 'Actual');
legend(ax_robot, 'Location', 'best');

% Info subplot
ax_info = subplot(1,2,2);
axis(ax_info, 'off');
xlim(ax_info, [0 1]);
ylim(ax_info, [0 1]);

text(0.5, 0.95, '🤖 MONITOR', 'FontSize', 16, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
txt_step = text(0.05, 0.85, '', 'FontSize', 13, 'FontWeight', 'bold');
txt_time = text(0.05, 0.77, '', 'FontSize', 11);
txt_progress = text(0.05, 0.70, '', 'FontSize', 10, 'Color', [0 0.5 0]);

text(0.05, 0.62, '📊 ANGLES:', 'FontSize', 11, 'FontWeight', 'bold');
txt_servo1 = text(0.08, 0.56, '', 'FontSize', 10);
txt_servo2 = text(0.08, 0.50, '', 'FontSize', 10);
txt_servo3 = text(0.08, 0.44, '', 'FontSize', 10);
txt_servo4 = text(0.08, 0.38, '', 'FontSize', 10);

text(0.05, 0.30, '🔄 DELTA:', 'FontSize', 11, 'FontWeight', 'bold');
txt_delta = text(0.08, 0.24, '', 'FontSize', 10, 'Color', 'blue');
txt_max_delta = text(0.08, 0.18, '', 'FontSize', 9, 'Color', [0.5 0.5 0.5]);

txt_status = text(0.5, 0.08, '', 'FontSize', 13, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');

fprintf('✓ Ready\n');

%% ===== 10. EXECUTION =====
fprintf('\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('=== EXECUTION ===\n');
fprintf('⏱️  Estimated time: %.1fs\n', N*dt);
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n');

tic;
sent_commands = 0;
max_delta_seen = 0;

if ~isempty(esp32_serial)
    fprintf('🤖 REAL ROBOT MODE\n\n');
    writeline(esp32_serial, "START ABSOLUTE");
    pause(1);
    mode_text = '🤖 REAL';
else
    fprintf('🎬 SIMULATION MODE\n\n');
    mode_text = '🎬 SIM';
end

% Update rate control
update_interval = max(1, round(N/20)); % Update display every 5%

for i = 1:N
    step_start = tic;
    
    try
        % 1. Send command
        if ~isempty(esp32_serial)
            cmd = sprintf("SERVO,%.1f,%.1f,%.1f,%.1f", ...
                         qj_servo(i,1), qj_servo(i,2), ...
                         qj_servo(i,3), qj_servo(i,4));
            writeline(esp32_serial, cmd);
            sent_commands = sent_commands + 1;
        end
        
        % 2. Update visualization (KHÔNG XÓA, CHỈ VẼ THÊM)
        if mod(i-1, update_interval) == 0 || i == 1 || i == N
            % Chỉ vẽ lại robot khi cần thiết để tăng tốc độ
            delete(findobj(ax_robot, 'Type', 'Line', '-and', 'Tag', 'robot'));
            delete(findobj(ax_robot, 'Type', 'Patch'));
            
            % Vẽ robot
            show(arm, qj(i,:), 'Parent', ax_robot, 'PreservePlot', false);
            
            % Vẽ lại desired path
            plot3(ax_robot, X, Y, Z, 'm--', 'LineWidth', 1);
            
            view(ax_robot, 135, 25);
        end
        
        % Update trail
        Tee = getTransform(arm, qj(i,:), eeName);
        addpoints(trail, Tee(1,4), Tee(2,4), Tee(3,4));
        
        % 3. Update info
        if mod(i-1, update_interval) == 0 || i == 1 || i == N
            elapsed = toc;
            remaining = (N - i) * dt;
            progress = i / N * 100;
            
            set(txt_step, 'String', sprintf('Step %d/%d', i, N));
            set(txt_time, 'String', sprintf('⏱️  %.1fs / %.1fs', elapsed, N*dt));
            set(txt_progress, 'String', sprintf('▓%.0f%%', progress));
            
            set(txt_servo1, 'String', sprintf('J1: %.1f°', qj_servo(i,1)));
            set(txt_servo2, 'String', sprintf('J2: %.1f°', qj_servo(i,2)));
            set(txt_servo3, 'String', sprintf('J3: %.1f°', qj_servo(i,3)));
            set(txt_servo4, 'String', sprintf('J4: %.1f°', qj_servo(i,4)));
            
            if i > 1
                delta = qj_servo(i,:) - qj_servo(i-1,:);
                max_delta = max(abs(delta));
                max_delta_seen = max(max_delta_seen, max_delta);
                
                set(txt_delta, 'String', sprintf('[%.2f,%.2f,%.2f,%.2f]°', ...
                    delta(1), delta(2), delta(3), delta(4)));
                set(txt_max_delta, 'String', sprintf('Max: %.2f°', max_delta_seen));
            end
            
            set(txt_status, 'String', sprintf('%s RUNNING', mode_text), 'Color', [0 0.7 0]);
            
            drawnow limitrate;
            
            fprintf('  [%3.0f%%] %d/%d | %.1fs\n', progress, i, N, elapsed);
        end
        
        % 4. Sync wait
        step_elapsed = toc(step_start);
        wait_time = dt - step_elapsed;
        if wait_time > 0
            pause(wait_time);
        end
        
    catch ME
        if mod(i-1, update_interval) == 0
            fprintf('  ❌ Step %d: %s\n', i, ME.message);
        end
    end
end

total_time = toc;

% Cleanup
if ~isempty(esp32_serial)
    writeline(esp32_serial, "STOP");
    clear esp32_serial;
end

fprintf('\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('✅ COMPLETED!\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('📊 Points: %d\n', N);
if sent_commands > 0
    fprintf('📤 Sent: %d/%d\n', sent_commands, N);
end
fprintf('⏱️  Time: %.2fs (%.3fs/point)\n', total_time, total_time/N);
fprintf('🎯 Max Δ: %.2f°\n', max_delta_seen);
fprintf('✨ Smooth: x%d + S-curve + Filter\n', INTERP_FACTOR);
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

set(txt_status, 'String', '✅ DONE!', 'Color', [0 0.5 0]);

end