#!/usr/bin/perl

use strict;
use Tinkerforge::IPConnection;
use Tinkerforge::BrickletGPSV3;

use constant HOST => 'localhost';
use constant PORT => 4223;
use constant UID => 'XYZ'; # Change XYZ to the UID of your GPS Bricklet 3.0

# Callback subroutine for coordinates callback
sub cb_coordinates
{
    my ($latitude, $ns, $longitude, $ew) = @_;

    print "Latitude: " . $latitude/1000000.0 . " °\n";
    print "N/S: $ns\n";
    print "Longitude: " . $longitude/1000000.0 . " °\n";
    print "E/W: $ew\n";
    print "\n";
}

my $ipcon = Tinkerforge::IPConnection->new(); # Create IP connection
my $gps = Tinkerforge::BrickletGPSV3->new(&UID, $ipcon); # Create device object

$ipcon->connect(&HOST, &PORT); # Connect to brickd
# Don't use device before ipcon is connected

# Register coordinates callback to subroutine cb_coordinates
$gps->register_callback($gps->CALLBACK_COORDINATES, 'cb_coordinates');

# Set period for coordinates callback to 1s (1000ms)
# Note: The coordinates callback is only called every second
#       if the coordinates has changed since the last call!
$gps->set_coordinates_callback_period(1000);

print "Press key to exit\n";
<STDIN>;
$ipcon->disconnect();
