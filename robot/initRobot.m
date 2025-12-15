function arm = initRobot(config)
    fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
    fprintf('🤖 ROBOT 4DOF - SMOOTH MOTION SYSTEM\n');
    fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n');
    fprintf('=== KHỞI TẠO ROBOT ===\n');
    
    % Create robot using ETS
    e = ETS3.Rz('q1') * ETS3.Ry('q2') * ETS3.Tz(config.a1) * ...
        ETS3.Ry('q3') * ETS3.Tz(config.a2) * ETS3.Ry('q4') * ...
        ETS3.Tz(config.a3) * ETS3.Tx(config.a4);
    
    arm = ets2rbt(e);
    arm.DataFormat = "row";
    
    fprintf('✓ Links: %.3f, %.3f, %.3f, %.3f m\n', ...
            config.a1, config.a2, config.a3, config.a4);
end
