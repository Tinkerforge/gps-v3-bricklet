import com.tinkerforge.IPConnection;
import com.tinkerforge.BrickletGPSV3;
import com.tinkerforge.BrickletGPSV3.Coordinates;

public class ExampleSimple {
	private static final String HOST = "localhost";
	private static final int PORT = 4223;

	// Change XYZ to the UID of your GPS Bricklet 3.0
	private static final String UID = "XYZ";

	// Note: To make the example code cleaner we do not handle exceptions. Exceptions
	//       you might normally want to catch are described in the documentation
	public static void main(String args[]) throws Exception {
		IPConnection ipcon = new IPConnection(); // Create IP connection
		BrickletGPSV3 gps = new BrickletGPSV3(UID, ipcon); // Create device object

		ipcon.connect(HOST, PORT); // Connect to brickd
		// Don't use device before ipcon is connected

		// Get current coordinates
		Coordinates coordinates = gps.getCoordinates(); // Can throw com.tinkerforge.TimeoutException

		System.out.println("Latitude: " + coordinates.latitude/1000000.0 + " °");
		System.out.println("N/S: " + coordinates.ns);
		System.out.println("Longitude: " + coordinates.longitude/1000000.0 + " °");
		System.out.println("E/W: " + coordinates.ew);

		System.out.println("Press key to exit"); System.in.read();
		ipcon.disconnect();
	}
}
