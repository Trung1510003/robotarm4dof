function printSummary(N, sent_commands, total_time, max_delta, config)
    fprintf('\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
    fprintf('✅ COMPLETED!\n');
    fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
    fprintf('📊 Points: %d\n', N);
    if sent_commands > 0
        fprintf('📤 Sent: %d/%d\n', sent_commands, N);
    end
    fprintf('⏱️  Time: %.2fs (%.3fs/point)\n', total_time, total_time/N);
    fprintf('🎯 Max Δ: %.2f°\n', max_delta);
    fprintf('✨ Smooth: x%d + S-curve + Filter\n', config.INTERP_FACTOR);
    fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
end