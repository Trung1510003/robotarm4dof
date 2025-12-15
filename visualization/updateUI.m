function updateUI(ui, i, N, qj_servo, dt, mode_text, max_delta_seen)
    elapsed = toc;
    progress = i / N * 100;
    
    set(ui.txt_step, 'String', sprintf('Step %d/%d', i, N));
    set(ui.txt_time, 'String', sprintf('⏱️  %.1fs / %.1fs', elapsed, N*dt));
    set(ui.txt_progress, 'String', sprintf('▓%.0f%%', progress));
    
    set(ui.txt_servo1, 'String', sprintf('J1: %.1f°', qj_servo(i,1)));
    set(ui.txt_servo2, 'String', sprintf('J2: %.1f°', qj_servo(i,2)));
    set(ui.txt_servo3, 'String', sprintf('J3: %.1f°', qj_servo(i,3)));
    set(ui.txt_servo4, 'String', sprintf('J4: %.1f°', qj_servo(i,4)));
    
    if i > 1
        delta = qj_servo(i,:) - qj_servo(i-1,:);
        set(ui.txt_delta, 'String', sprintf('[%.2f,%.2f,%.2f,%.2f]°', ...
            delta(1), delta(2), delta(3), delta(4)));
        set(ui.txt_max_delta, 'String', sprintf('Max: %.2f°', max_delta_seen));
    end
    
    set(ui.txt_status, 'String', sprintf('%s RUNNING', mode_text), ...
        'Color', [0 0.7 0]);
end
