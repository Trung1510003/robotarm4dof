function [traj, ch] = generateTrajectory(config)
    fprintf('\n=== TẠO QUỸ ĐẠO ===\n');
    
    load hershey
    ch = config.letter;
    B = hershey{ch};
    
    % Scale and prepare path
    p = [config.scale * B.stroke; zeros(1, size(B.stroke,2))];
    
    % Handle pen up
    k = find(isnan(p(1,:)));
    p(:,k) = p(:,k-1);
    p(3,k) = config.penUp;
    
    % Generate trajectory with mstraj
    traj = mstraj(p(:,2:end)', config.vmax, [], p(:,1)', ...
                  config.SYNC_DELAY, config.accTime);
    
    fprintf('✓ Chữ: "%c", waypoints: %d\n', ch, size(traj,1));
end
