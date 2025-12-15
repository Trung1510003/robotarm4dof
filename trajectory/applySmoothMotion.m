function [traj_smooth, dt_new] = applySmoothMotion(traj, config)
    fprintf('\n=== S-CURVE SMOOTHING ===\n');
    
    if ~config.USE_SCURVE || size(traj,1) <= 2
        traj_smooth = traj;
        dt_new = config.SYNC_DELAY;
        return;
    end
    
    % S-curve interpolation
    traj_smooth = [];
    for i = 1:size(traj,1)-1
        p1 = traj(i,:);
        p2 = traj(i+1,:);
        for j = 0:config.INTERP_FACTOR
            t = j / config.INTERP_FACTOR;
            s = 3*t^2 - 2*t^3;  % S-curve function
            p_interp = p1 + s * (p2 - p1);
            traj_smooth = [traj_smooth; p_interp];
        end
    end
    traj_smooth = [traj_smooth; traj(end,:)];
    
    dt_new = config.SYNC_DELAY / (config.INTERP_FACTOR + 1);
    
    fprintf('✓ %d -> %d points (x%d smooth)\n', ...
            size(traj,1), size(traj_smooth,1), config.INTERP_FACTOR);
    fprintf('✓ dt = %.4fs\n', dt_new);
end
