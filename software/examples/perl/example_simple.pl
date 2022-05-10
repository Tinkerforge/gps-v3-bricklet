#!/usr/bin/perl

use strict;
use Tinkerforge::IPConnection;
use Tinkerforge::BrickletGPSV3;

use constant HOST => 'localhost';
use constant PORT => 4223;
use constant UID => 'XYZ'; # Change XYZ to the UID of your GPS Bricklet 3.0

my $ipcon = Tinkerforge::IPConnection->new(); # Create IP connection
my $gps = Tinkerforge::BrickletGPSV3->new(&UID, $ipcon); # Create device object

$ipcon->connect(&HOST, &PORT); # Connect to brickd
# Don't use device before ipcon is connected

# Get current coordinates
my ($latitude, $ns, $longitude, $ew) = $gps->get_coordinates();

print "Latitude: " . $latitude/1000000.0 . " °\n";
print "N/S: $ns\n";
print "Longitude: " . $longitude/1000000.0 . " °\n";
print "E/W: $ew\n";

print "Press key to exit\n";
<STDIN>;
$ipcon->disconnect();
