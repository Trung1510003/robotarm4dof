%% ======================= FINAL IK BENCHMARK (CUSTOM SOLVERS) =========================
% So sánh 2 IK solver tự code trên bộ 36 ký tự (A–Z, 0–9)
% Metrics: mean_RMSE, mean_MaxPoseErr, mean_Manip, mean_Time, overall_SuccessRate
% Solver:
%   - "BFGSGradientProjection": bước Jacobian transpose có giới hạn (mô phỏng GP)
%   - "LevenbergMarquardt": damped least squares (LM)
% Author: ChatGPT

clc; clear; close all; rng default;

%% ========================== Robot (ETS3 4DOF) ============================
a1 = 0.12; a2 = 0.118; a3 = 0.083; a4 = 0.045;
e = ETS3.Rz('q1') * ETS3.Ry('q2') * ETS3.Tz(a1) * ...
    ETS3.Ry('q3') * ETS3.Tz(a2) * ETS3.Ry('q4') * ETS3.Tz(a3) * ETS3.Tx(a4);
robot = ets2rbt(e);
robot.DataFormat = "row";
eeName = robot.BodyNames{end};

%% ======================= Load Hershey dataset ============================
load hershey      % has hershey{A}, hershey{B}, etc.

chars = ['A':'Z' '0':'9'];   % 36 characters dataset
nChars = numel(chars);

%% ======================= IK Solvers (custom) =============================
solverNames = ["BFGSGradientProjection", "LevenbergMarquardt"];
nSolvers = numel(solverNames);

% Tham số solver
ikOpts.lambda = 1e-3;     % hệ số damping (LM)
ikOpts.alpha = 0.2;       % hệ số bước cho Jacobian transpose (giả lập GP)
ikOpts.maxIter = 200;
ikOpts.tol = 1e-3;
ikOpts.dq_max = 0.25;     % giới hạn bước để tránh nhảy quá lớn

%% =================== Metrics storage over dataset ===================
mean_RMSE = zeros(nSolvers,1);
mean_MaxErr = zeros(nSolvers,1);
mean_Manip = zeros(nSolvers,1);
mean_Time = zeros(nSolvers,1);
successRate = zeros(nSolvers,1);

%% ================= Helper: pose error 6D ============================
    function d = tr2delta_fun(Tact, Tdes)
        R1 = Tact(1:3,1:3); R2 = Tdes(1:3,1:3);
        p1 = Tact(1:3,4);   p2 = Tdes(1:3,4);
        dp = p2 - p1;
        axang = rotm2axang(R1' * R2);
        dr = axang(4) * axang(1:3)';    % axis * angle
        d = [dr; dp];
    end

%% ================= Helper: IK solver tự cài đặt =====================
    function q = solveIKIter(robot, eeName, Tdes, q0, solver, opts)
        % BFGSGradientProjection: quasi-Newton với cập nhật BFGS (H~inv Hessian),
        % bước bị chặn bởi dq_max để tránh nhảy lớn.
        % LevenbergMarquardt: damped least squares cổ điển.
        q = q0(:)'; % đảm bảo row vector
        n = numel(q);
        H = eye(n); % cho BFGS (xấp xỉ inverse Hessian)
        for iter = 1:opts.maxIter
            Tcur = getTransform(robot, q, eeName);
            err = tr2delta_fun(Tcur, Tdes); % 6x1
            if norm(err) < opts.tol
                return;
            end
            J = geometricJacobian(robot, q, eeName); % 6 x n
            grad = J.' * err; % gradient của hàm lỗi 1/2||err||^2
            switch solver
                case "LevenbergMarquardt"
                    % Damped least squares
                    A = J.'*J + opts.lambda * eye(n);
                    dq = -A \ grad; % bước giảm lỗi
                otherwise % "BFGSGradientProjection"
                    % bước quasi-Newton: p = -H * grad
                    dq = -opts.alpha * (H * grad);
            end
            % Giới hạn bước để tránh nhảy quá lớn
            dq = max(min(dq, opts.dq_max), -opts.dq_max);
            q_new = q + dq.';
            % Cập nhật BFGS (chỉ cho solver BFGS*)
            if solver == "BFGSGradientProjection"
                Tnew = getTransform(robot, q_new, eeName);
                err_new = tr2delta_fun(Tnew, Tdes);
                Jnew = geometricJacobian(robot, q_new, eeName);
                grad_new = Jnew.' * err_new;
                s = dq(:);                 % bước
                y = grad_new - grad;       % chênh gradient
                sy = s.' * y;
                if sy > 1e-9  % tránh chia nhỏ và giữ H dương xác định
                    rho = 1 / sy;
                    I = eye(n);
                    H = (I - rho * (s * y.')) * H * (I - rho * (y * s.')) + rho * (s * s.');
                end
            end
            q = q_new; % bước tiếp
        end
        % Nếu hết vòng lặp mà chưa đạt ngưỡng, trả về q cuối
    end

%% ================= PROCESS EACH SOLVER ================================
for s = 1:nSolvers
    solver = solverNames(s);
    fprintf("\n======== Solver: %s (custom) ========\n", solver);

    RMSE_list = zeros(nChars,1);
    MaxErr_list = zeros(nChars,1);
    Manip_list = zeros(nChars,1);
    Time_list = zeros(nChars,1);
    Succ_list = zeros(nChars,1);

    for c = 1:nChars
        ch = chars(c);
        fprintf("Character %c ...\n", ch);
        
        %% ======== Build Trajectory from Hershey strokes ============
        H = hershey{ch};

        scale = 0.15;
        p = [scale * H.stroke; zeros(1,size(H.stroke,2))];
        k = find(isnan(p(1,:)));
        p(:,k) = p(:,k-1);
        p(3,k) = 0.03;   % pen-up

        dt = 0.02;
        vmax = [0.05 0.05 0.05];
        accTime = 0.2;

        traj = mstraj(p(:,2:end)', vmax, [], p(:,1)', dt, accTime);
        N = size(traj,1);

        Tp = zeros(4,4,N);
        T_offset = trvec2tform([0.15 0 -0.055]);
        T_orient = axang2tform([0 1 0 pi/2]);
        xyz_des = zeros(N,3);

        for i = 1:N
            T_traj = trvec2tform(traj(i,:));
            Tp(:,:,i) = T_offset * T_traj * T_orient;
            xyz_des(i,:) = Tp(1:3,4,i)';
        end

        %% ===== Run IK for whole trajectory =====
        qc = robot.homeConfiguration;
        if isstruct(qc)
            q0 = cellfun(@(x)x, {qc.JointPosition});
        else
            q0 = qc;
        end

        XYZ = zeros(N,3);
        manip = zeros(N,1);
        poseErr = zeros(N,1);
        successCnt = 0;

        tStart = tic;
        for i = 1:N
            q = solveIKIter(robot, eeName, Tp(:,:,i), q0, solver, ikOpts);
            q0 = q;     % warm start

            Tact = getTransform(robot, q, eeName);
            XYZ(i,:) = Tact(1:3,4)';

            d = tr2delta_fun(Tact, Tp(:,:,i));
            poseErr(i) = norm(d);

            if poseErr(i) < 0.01   % threshold for success
                successCnt = successCnt + 1;
            end

            J = geometricJacobian(robot, q, eeName);
            Jv = J(1:3,:);
            M = Jv*Jv';
            if det(M) > 0
                manip(i) = sqrt(det(M));
            else
                manip(i) = 0;
            end
        end
        tElapsed = toc(tStart);

        %% ===== Collect metrics per character =====
        posErr = sqrt(sum((XYZ - xyz_des).^2,2));
        RMSE_list(c) = sqrt(mean(posErr.^2));
        MaxErr_list(c) = max(poseErr);
        Manip_list(c) = mean(manip);
        Time_list(c) = tElapsed;
        Succ_list(c) = successCnt / N;
    end

    %% ====== Summaries over dataset =======
    mean_RMSE(s) = mean(RMSE_list);
    mean_MaxErr(s) = mean(MaxErr_list);
    mean_Manip(s) = mean(Manip_list);
    mean_Time(s) = mean(Time_list);
    successRate(s) = mean(Succ_list);
end

%% ================= PRINT FINAL TABLE ==========================
T = table(solverNames', mean_RMSE, mean_MaxErr, mean_Manip, mean_Time, successRate);
T.Properties.VariableNames = ...
    {'Solver','Mean_RMSE','Mean_MaxPoseErr','Mean_Manip','Mean_Time_s','SuccessRate'};

disp("===== FINAL IK BENCHMARK RESULTS (CUSTOM) =====");
disp(T);

