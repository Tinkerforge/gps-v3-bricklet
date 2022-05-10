#!/usr/bin/env ruby
# -*- ruby encoding: utf-8 -*-

require 'tinkerforge/ip_connection'
require 'tinkerforge/bricklet_gps_v3'

include Tinkerforge

HOST = 'localhost'
PORT = 4223
UID = 'XYZ' # Change XYZ to the UID of your GPS Bricklet 3.0

ipcon = IPConnection.new # Create IP connection
gps = BrickletGPSV3.new UID, ipcon # Create device object

ipcon.connect HOST, PORT # Connect to brickd
# Don't use device before ipcon is connected

# Register coordinates callback
gps.register_callback(BrickletGPSV3::CALLBACK_COORDINATES) do |latitude, ns, longitude,
                                                               ew|
  puts "Latitude: #{latitude/1000000.0} °"
  puts "N/S: #{ns}"
  puts "Longitude: #{longitude/1000000.0} °"
  puts "E/W: #{ew}"
  puts ''
end

# Set period for coordinates callback to 1s (1000ms)
# Note: The coordinates callback is only called every second
#       if the coordinates has changed since the last call!
gps.set_coordinates_callback_period 1000

puts 'Press key to exit'
$stdin.gets
ipcon.disconnect
