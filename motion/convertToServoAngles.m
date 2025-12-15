function qj_servo = convertToServoAngles(qj, config)
    fprintf('\n=== SERVO CONVERSION ===\n');
    
    qj_degrees = rad2deg(qj);
    qj_servo = zeros(size(qj_degrees));
    nJoints = size(qj, 2);
    
    for i = 1:nJoints
        switch i
            case 1
                qj_servo(:,i) = qj_degrees(:,i);
            case 2
                qj_servo(:,i) = 180 - qj_degrees(:,i);
            case 3
                qj_servo(:,i) = qj_degrees(:,i);
            case 4
                qj_servo(:,i) = abs(qj_degrees(:,i));
        end
        
        % Clamp to servo limits
        qj_servo(:,i) = max(config.servo_min_angle, ...
                           min(config.servo_max_angle, qj_servo(:,i)));
    end
    
    fprintf('✓ Converted to servo angles\n');
end