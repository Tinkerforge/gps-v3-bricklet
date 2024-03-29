use std::{error::Error, io, thread};
use tinkerforge::{gps_v3_bricklet::*, ip_connection::IpConnection};

const HOST: &str = "localhost";
const PORT: u16 = 4223;
const UID: &str = "XYZ"; // Change XYZ to the UID of your GPS Bricklet 3.0.

fn main() -> Result<(), Box<dyn Error>> {
    let ipcon = IpConnection::new(); // Create IP connection.
    let gps = GpsV3Bricklet::new(UID, &ipcon); // Create device object.

    ipcon.connect((HOST, PORT)).recv()??; // Connect to brickd.
                                          // Don't use device before ipcon is connected.

    let coordinates_receiver = gps.get_coordinates_callback_receiver();

    // Spawn thread to handle received callback messages.
    // This thread ends when the `gps` object
    // is dropped, so there is no need for manual cleanup.
    thread::spawn(move || {
        for coordinates in coordinates_receiver {
            println!("Latitude: {} °", coordinates.latitude as f32 / 1000000.0);
            println!("N/S: {}", coordinates.ns);
            println!("Longitude: {} °", coordinates.longitude as f32 / 1000000.0);
            println!("E/W: {}", coordinates.ew);
            println!();
        }
    });

    // Set period for coordinates receiver to 1s (1000ms).
    // Note: The coordinates callback is only called every second
    //       if the coordinates has changed since the last call!
    gps.set_coordinates_callback_period(1000);

    println!("Press enter to exit.");
    let mut _input = String::new();
    io::stdin().read_line(&mut _input)?;
    ipcon.disconnect();
    Ok(())
}
