function executeMotion(arm, qj, qj_servo, esp32, ax_robot, ui, config)
    fprintf('\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
    fprintf('=== EXECUTION ===\n');
    
    N = size(qj, 1);
    dt = config.SYNC_DELAY / (config.INTERP_FACTOR + 1);
    
    fprintf('⏱️  Estimated time: %.1fs\n', N*dt);
    fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n');
    
    % Initialize
    tic;
    sent_commands = 0;
    max_delta_seen = 0;
    eeName = arm.BodyNames{end};
    
    if ~isempty(esp32)
        fprintf('🤖 REAL ROBOT MODE\n\n');
        writeline(esp32, "START ABSOLUTE");
        pause(1);
        mode_text = '🤖 REAL';
    else
        fprintf('🎬 SIMULATION MODE\n\n');
        mode_text = '🎬 SIM';
    end
    
    update_interval = max(1, round(N * config.update_interval_percent / 100));
    
    % Main loop
    for i = 1:N
        step_start = tic;
        
        try
            % Send command to ESP32
            if ~isempty(esp32)
                cmd = sprintf("SERVO,%.1f,%.1f,%.1f,%.1f", ...
                             qj_servo(i,1), qj_servo(i,2), ...
                             qj_servo(i,3), qj_servo(i,4));
                writeline(esp32, cmd);
                sent_commands = sent_commands + 1;
            end
            
            % Update trail
            Tee = getTransform(arm, qj(i,:), eeName);
            addpoints(ui.trail, Tee(1,4), Tee(2,4), Tee(3,4));
            
            % Update UI periodically
            if mod(i-1, update_interval) == 0 || i == 1 || i == N
                updateUI(ui, i, N, qj_servo, dt, mode_text, max_delta_seen);
                drawnow limitrate;
                fprintf('  [%3.0f%%] %d/%d | %.1fs\n', ...
                        i/N*100, i, N, toc);
            end
            
            % Update max delta
            if i > 1
                delta = abs(qj_servo(i,:) - qj_servo(i-1,:));
                max_delta_seen = max(max_delta_seen, max(delta));
            end
            
            % Sync wait
            step_elapsed = toc(step_start);
            wait_time = dt - step_elapsed;
            if wait_time > 0
                pause(wait_time);
            end
            
        catch ME
            if mod(i-1, update_interval) == 0
                fprintf('  ❌ Step %d: %s\n', i, ME.message);
            end
        end
    end
    
    total_time = toc;
    
    % Print summary
    printSummary(N, sent_commands, total_time, max_delta_seen, config);
    
    set(ui.txt_status, 'String', '✅ DONE!', 'Color', [0 0.5 0]);
end