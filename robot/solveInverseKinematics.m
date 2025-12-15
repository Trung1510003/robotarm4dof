function qj = solveInverseKinematics(arm, Tp, config)
    fprintf('\n=== INVERSE KINEMATICS ===\n');
    
    ik = inverseKinematics("RigidBodyTree", arm);
    weights = [0 1 0 1 1 1];
    
    cfg0 = arm.homeConfiguration;
    nJoints = numel(cfg0);
    N = size(Tp, 3);
    qj = zeros(N, nJoints);
    
    fprintf('Computing IK');
    success_count = 0;
    
    for i = 1:N
        eeName = arm.BodyNames{end};
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
    
    fprintf(' Done!\n✓ Success: %d/%d (%.1f%%)\n', ...
            success_count, N, success_count/N*100);
end