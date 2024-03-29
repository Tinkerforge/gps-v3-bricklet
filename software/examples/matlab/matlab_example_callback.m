function matlab_example_callback()
    import com.tinkerforge.IPConnection;
    import com.tinkerforge.BrickletGPSV3;

    HOST = 'localhost';
    PORT = 4223;
    UID = 'XYZ'; % Change XYZ to the UID of your GPS Bricklet 3.0

    ipcon = IPConnection(); % Create IP connection
    gps = handle(BrickletGPSV3(UID, ipcon), 'CallbackProperties'); % Create device object

    ipcon.connect(HOST, PORT); % Connect to brickd
    % Don't use device before ipcon is connected

    % Register coordinates callback to function cb_coordinates
    set(gps, 'CoordinatesCallback', @(h, e) cb_coordinates(e));

    % Set period for coordinates callback to 1s (1000ms)
    % Note: The coordinates callback is only called every second
    %       if the coordinates has changed since the last call!
    gps.setCoordinatesCallbackPeriod(1000);

    input('Press key to exit\n', 's');
    ipcon.disconnect();
end

% Callback function for coordinates callback
function cb_coordinates(e)
    fprintf('Latitude: %g °\n', e.latitude/1000000.0);
    fprintf('N/S: %s\n', e.ns);
    fprintf('Longitude: %g °\n', e.longitude/1000000.0);
    fprintf('E/W: %s\n', e.ew);
    fprintf('\n');
end
