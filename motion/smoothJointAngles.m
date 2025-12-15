function qj_smooth = smoothJointAngles(qj)
    fprintf('\n=== JOINT SMOOTHING ===\n');
    
    N = size(qj, 1);
    nJoints = size(qj, 2);
    
    if N <= 5
        qj_smooth = qj;
        return;
    end
    
    qj_smooth = qj;
    window_size = 1;
    
    for j = 1:nJoints
        qj_smooth(:,j) = movmean(qj(:,j), window_size);
    end
    
    jerk_before = sum(abs(diff(diff(diff(qj)))));
    jerk_after = sum(abs(diff(diff(diff(qj_smooth)))));
    
    fprintf('✓ Jerk reduction: %.1f%%\n', (1-jerk_after/jerk_before)*100);
end