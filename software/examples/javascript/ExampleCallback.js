var Tinkerforge = require('tinkerforge');

var HOST = 'localhost';
var PORT = 4223;
var UID = 'XYZ'; // Change XYZ to the UID of your GPS Bricklet 3.0

var ipcon = new Tinkerforge.IPConnection(); // Create IP connection
var gps = new Tinkerforge.BrickletGPSV3(UID, ipcon); // Create device object

ipcon.connect(HOST, PORT,
    function (error) {
        console.log('Error: ' + error);
    }
); // Connect to brickd
// Don't use device before ipcon is connected

ipcon.on(Tinkerforge.IPConnection.CALLBACK_CONNECTED,
    function (connectReason) {
        // Set period for coordinates callback to 1s (1000ms)
        // Note: The coordinates callback is only called every second
        //       if the coordinates has changed since the last call!
        gps.setCoordinatesCallbackPeriod(1000);
    }
);

// Register coordinates callback
gps.on(Tinkerforge.BrickletGPSV3.CALLBACK_COORDINATES,
    // Callback function for coordinates callback
    function (latitude, ns, longitude, ew) {
        console.log('Latitude: ' + latitude/1000000.0 + ' °');
        console.log('N/S: ' + ns);
        console.log('Longitude: ' + longitude/1000000.0 + ' °');
        console.log('E/W: ' + ew);
        console.log();
    }
);

console.log('Press key to exit');
process.stdin.on('data',
    function (data) {
        ipcon.disconnect();
        process.exit(0);
    }
);
