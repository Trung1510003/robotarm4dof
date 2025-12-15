function cleanupESP32(esp32)
    if ~isempty(esp32)
        try
            writeline(esp32, "STOP");
            clear esp32;
        catch
            % Ignore cleanup errors
        end
    end
end