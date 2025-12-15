function [fig, ax_robot, ax_info, ui] = setupVisualization(arm, Tp, ch, config)
    fprintf('\n=== VISUALIZATION SETUP ===\n');
    
    fig = figure('Position', [100 100 1400 700], ...
                 'Name', 'Robot Smooth Motion');
    
    % Robot subplot
    ax_robot = subplot(1,2,1);
    hold(ax_robot, 'on');
    view(ax_robot, 135, 25);
    grid(ax_robot, 'on');
    axis(ax_robot, 'equal');
    title(ax_robot, sprintf('Robot 4DOF - "%c" (SMOOTH)', ch));
    
    % Desired path
    X = squeeze(Tp(1,4,:));
    Y = squeeze(Tp(2,4,:));
    Z = squeeze(Tp(3,4,:));
    plot3(ax_robot, X, Y, Z, 'm--', 'LineWidth', 1, 'DisplayName', 'Target');
    
    % Trail
    ui.trail = animatedline('Parent', ax_robot, 'LineWidth', 2.5, ...
                            'Color', 'red', 'DisplayName', 'Actual');
    legend(ax_robot, 'Location', 'best');
    
    % Info subplot
    ax_info = subplot(1,2,2);
    axis(ax_info, 'off');
    xlim(ax_info, [0 1]);
    ylim(ax_info, [0 1]);
    
    % Create UI text elements
    text(0.5, 0.95, '🤖 MONITOR', 'FontSize', 16, ...
         'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    ui.txt_step = text(0.05, 0.85, '', 'FontSize', 13, 'FontWeight', 'bold');
    ui.txt_time = text(0.05, 0.77, '', 'FontSize', 11);
    ui.txt_progress = text(0.05, 0.70, '', 'FontSize', 10, 'Color', [0 0.5 0]);
    
    text(0.05, 0.62, '📊 ANGLES:', 'FontSize', 11, 'FontWeight', 'bold');
    ui.txt_servo1 = text(0.08, 0.56, '', 'FontSize', 10);
    ui.txt_servo2 = text(0.08, 0.50, '', 'FontSize', 10);
    ui.txt_servo3 = text(0.08, 0.44, '', 'FontSize', 10);
    ui.txt_servo4 = text(0.08, 0.38, '', 'FontSize', 10);
    
    text(0.05, 0.30, '🔄 DELTA:', 'FontSize', 11, 'FontWeight', 'bold');
    ui.txt_delta = text(0.08, 0.24, '', 'FontSize', 10, 'Color', 'blue');
    ui.txt_max_delta = text(0.08, 0.18, '', 'FontSize', 9, ...
                           'Color', [0.5 0.5 0.5]);
    
    ui.txt_status = text(0.5, 0.08, '', 'FontSize', 13, ...
                        'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    
    fprintf('✓ Ready\n');
end