using System;
using Tinkerforge;

class Example
{
	private static string HOST = "localhost";
	private static int PORT = 4223;
	private static string UID = "XYZ"; // Change XYZ to the UID of your GPS Bricklet 3.0

	static void Main()
	{
		IPConnection ipcon = new IPConnection(); // Create IP connection
		BrickletGPSV3 gps = new BrickletGPSV3(UID, ipcon); // Create device object

		ipcon.Connect(HOST, PORT); // Connect to brickd
		// Don't use device before ipcon is connected

		// Get current coordinates
		long latitude, longitude; char ns, ew;
		gps.GetCoordinates(out latitude, out ns, out longitude, out ew);

		Console.WriteLine("Latitude: " + latitude/1000000.0 + " °");
		Console.WriteLine("N/S: " + ns);
		Console.WriteLine("Longitude: " + longitude/1000000.0 + " °");
		Console.WriteLine("E/W: " + ew);

		Console.WriteLine("Press enter to exit");
		Console.ReadLine();
		ipcon.Disconnect();
	}
}
