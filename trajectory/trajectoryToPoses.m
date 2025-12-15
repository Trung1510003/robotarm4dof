function Tp = trajectoryToPoses(traj, config)
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
end