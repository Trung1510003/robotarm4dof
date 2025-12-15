function esp32 = connectESP32(config)
    fprintf('\n=== ESP32 CONNECTION ===\n');
    
    esp32 = [];
    
    try
        if ~isempty(instrfind)
            fclose(instrfind);
            delete(instrfind);
        end
        
        fprintf('Connecting...');
        esp32 = serialport(config.esp32_com_port, config.esp32_baudrate);
        configureTerminator(esp32, "LF");
        pause(2);
        
        writeline(esp32, "TEST");
        pause(0.5);
        
        if esp32.NumBytesAvailable > 0
            response = readline(esp32);
            fprintf(' OK\n✓ %s\n', strtrim(response));
        else
            fprintf(' OK\n');
        end
        
    catch ME
        fprintf(' FAILED\n⚠️  %s\n➜ Simulation mode\n', ME.message);
        esp32 = [];
    end
end