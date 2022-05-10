#!/bin/sh
# Connects to localhost:4223 by default, use --host and --port to change this

uid=XYZ # Change XYZ to the UID of your GPS Bricklet 3.0

# Get current coordinates
tinkerforge call gps-v3-bricklet $uid get-coordinates
