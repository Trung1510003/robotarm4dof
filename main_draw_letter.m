function main_draw_letter()
    % Khởi động hệ thống
    close all; clear; clc;
    
    % 1. Load configuration
    config = RobotConfig();
    
    % 2. Initialize robot
    arm = initRobot(config);
    
    % 3. Generate trajectory
    [traj, ch] = generateTrajectory(config);
    
    % 4. Apply smooth motion
    [traj_smooth, dt] = applySmoothMotion(traj, config);
    
    % 5. Transform to poses
    Tp = trajectoryToPoses(traj_smooth, config);
    
    % 6. Solve inverse kinematics
    qj = solveInverseKinematics(arm, Tp, config);
    
    % 7. Smooth joint angles
    qj = smoothJointAngles(qj);
    
    % 8. Convert to servo angles
    qj_servo = convertToServoAngles(qj, config);
    
    % 9. Connect to ESP32
    esp32 = connectESP32(config);
    
    % 10. Setup visualization
    [fig, ax_robot, ax_info, ui_elements] = setupVisualization(arm, Tp, ch, config);
    
    % 11. Execute motion
    executeMotion(arm, qj, qj_servo, esp32, ax_robot, ui_elements, config);
    
    % 12. Cleanup
    cleanupESP32(esp32);
end