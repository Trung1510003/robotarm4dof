function config = RobotConfig()
    % Hardware settings
    config.esp32_com_port = '/dev/ttyUSB0';
    config.esp32_baudrate = 115200;
    config.servo_min_angle = 0;
    config.servo_max_angle = 180;
    config.joint_limits = [-90 90; -90 90; -90 90; -90 90];
    
    % Motion parameters
    config.SERVO_MOVE_TIME = 0.15;
    config.SAFETY_MARGIN = 0.02;
    config.SYNC_DELAY = config.SERVO_MOVE_TIME + config.SAFETY_MARGIN;
    
    % Smoothing parameters
    config.INTERP_FACTOR = 3;
    config.USE_SCURVE = true;
    
    % Robot dimensions
    config.a1 = 0.12;    % link 1
    config.a2 = 0.118;   % link 2
    config.a3 = 0.083;   % wrist + pen
    config.a4 = 0.045;
    
    % Trajectory parameters
    config.letter = 'B';
    config.scale = 0.12;
    config.penUp = 0.07;
    config.vmax = [0.01 0.01 0.01];
    config.accTime = 0.1;
    
    % Visualization
    config.update_interval_percent = 5;
end