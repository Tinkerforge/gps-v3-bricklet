EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "GPS V3 Bricklet"
Date "2020-10-09"
Rev "3.0"
Comp "Tinkerforge GmbH"
Comment1 "Licensed under CERN OHL v.1.1"
Comment2 "Copyright (©) 2020, T.Schneidermann <tim@tinkerforge.com>"
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L tinkerforge:GND #PWR01
U 1 1 507EAA92
P 10100 2300
F 0 "#PWR01" H 10100 2300 30  0001 C CNN
F 1 "GND" H 10100 2230 30  0001 C CNN
F 2 "" H 10100 2300 60  0001 C CNN
F 3 "" H 10100 2300 60  0001 C CNN
	1    10100 2300
	1    0    0    -1  
$EndComp
$Comp
L tinkerforge:CONN_2 P104
U 1 1 506C19FC
P 11000 3000
F 0 "P104" V 10950 3000 40  0000 C CNN
F 1 "U.FL" V 11050 3000 40  0000 C CNN
F 2 "kicad-libraries:U.FL-CON" H 11000 3000 60  0001 C CNN
F 3 "" H 11000 3000 60  0001 C CNN
	1    11000 3000
	1    0    0    -1  
$EndComp
Text Notes 550  7700 0    40   ~ 0
Copyright Tinkerforge GmbH 2020.\nThis documentation describes Open Hardware and is licensed under the\nCERN OHL v. 1.1.\nYou may redistribute and modify this documentation under the terms of the\nCERN OHL v.1.1. (http://ohwr.org/cernohl). This documentation is distributed\nWITHOUT ANY EXPRESS OR IMPLIED WARRANTY, INCLUDING OF\nMERCHANTABILITY, SATISFACTORY QUALITY AND FITNESS FOR A\nPARTICULAR PURPOSE. Please see the CERN OHL v.1.1 for applicable\nconditions\n
NoConn ~ 1550 1100
$Comp
L tinkerforge:DRILL U103
U 1 1 4C6050A5
P 10650 6150
F 0 "U103" H 10700 6200 60  0001 C CNN
F 1 "DRILL" H 10650 6150 60  0000 C CNN
F 2 "kicad-libraries:DRILL_NP" H 10650 6150 60  0001 C CNN
F 3 "" H 10650 6150 60  0001 C CNN
	1    10650 6150
	1    0    0    -1  
$EndComp
$Comp
L tinkerforge:DRILL U104
U 1 1 4C6050A2
P 10650 6350
F 0 "U104" H 10700 6400 60  0001 C CNN
F 1 "DRILL" H 10650 6350 60  0000 C CNN
F 2 "kicad-libraries:DRILL_NP" H 10650 6350 60  0001 C CNN
F 3 "" H 10650 6350 60  0001 C CNN
	1    10650 6350
	1    0    0    -1  
$EndComp
$Comp
L tinkerforge:DRILL U106
U 1 1 4C60509F
P 11000 6350
F 0 "U106" H 11050 6400 60  0001 C CNN
F 1 "DRILL" H 11000 6350 60  0000 C CNN
F 2 "kicad-libraries:DRILL_NP" H 11000 6350 60  0001 C CNN
F 3 "" H 11000 6350 60  0001 C CNN
	1    11000 6350
	1    0    0    -1  
$EndComp
$Comp
L tinkerforge:DRILL U105
U 1 1 4C605099
P 11000 6150
F 0 "U105" H 11050 6200 60  0001 C CNN
F 1 "DRILL" H 11000 6150 60  0000 C CNN
F 2 "kicad-libraries:DRILL_NP" H 11000 6150 60  0001 C CNN
F 3 "" H 11000 6150 60  0001 C CNN
	1    11000 6150
	1    0    0    -1  
$EndComp
$Comp
L tinkerforge:GND #PWR05
U 1 1 4C5FCF5E
P 1800 1200
F 0 "#PWR05" H 1800 1200 30  0001 C CNN
F 1 "GND" H 1800 1130 30  0001 C CNN
F 2 "" H 1800 1200 60  0001 C CNN
F 3 "" H 1800 1200 60  0001 C CNN
	1    1800 1200
	0    -1   -1   0   
$EndComp
$Comp
L tinkerforge:CON-SENSOR2 P101
U 1 1 4C5FCF27
P 1200 1400
F 0 "P101" H 1100 1800 60  0000 C CNN
F 1 "CON-SENSOR" V 1350 1400 60  0000 C CNN
F 2 "kicad-libraries:CON-SENSOR2" H 1200 1400 60  0001 C CNN
F 3 "" H 1200 1400 60  0001 C CNN
	1    1200 1400
	-1   0    0    -1  
$EndComp
$Comp
L tinkerforge:XMC1XXX24 U101
U 1 1 5805EA54
P 3600 2750
F 0 "U101" H 3450 3150 60  0000 C CNN
F 1 "XMC1302-32" H 3600 2300 60  0000 C CNN
F 2 "kicad-libraries:QFN24-4x4mm-0.5mm" H 3750 3500 60  0001 C CNN
F 3 "" H 3750 3500 60  0000 C CNN
	1    3600 2750
	1    0    0    -1  
$EndComp
$Comp
L tinkerforge:XMC1XXX24 U101
U 2 1 5805EBB8
P 3600 4000
F 0 "U101" H 3450 4600 60  0000 C CNN
F 1 "XMC1302-32" H 3600 3400 60  0000 C CNN
F 2 "kicad-libraries:QFN24-4x4mm-0.5mm" H 3750 4750 60  0001 C CNN
F 3 "" H 3750 4750 60  0000 C CNN
	2    3600 4000
	1    0    0    -1  
$EndComp
$Comp
L tinkerforge:XMC1XXX24 U101
U 4 1 5805EC1F
P 3600 6100
F 0 "U101" H 3450 6550 60  0000 C CNN
F 1 "XMC1302-32" H 3600 5600 60  0000 C CNN
F 2 "kicad-libraries:QFN24-4x4mm-0.5mm" H 3750 6850 60  0001 C CNN
F 3 "" H 3750 6850 60  0000 C CNN
	4    3600 6100
	1    0    0    -1  
$EndComp
$Comp
L tinkerforge:XMC1XXX24 U101
U 3 1 5805ECD4
P 3600 5200
F 0 "U101" H 3450 5450 60  0000 C CNN
F 1 "XMC1302-32" H 3600 4900 60  0000 C CNN
F 2 "kicad-libraries:QFN24-4x4mm-0.5mm" H 3750 5950 60  0001 C CNN
F 3 "" H 3750 5950 60  0000 C CNN
	3    3600 5200
	1    0    0    -1  
$EndComp
$Comp
L tinkerforge:GND #PWR013
U 1 1 58060579
P 1200 1950
F 0 "#PWR013" H 1200 1950 30  0001 C CNN
F 1 "GND" H 1200 1880 30  0001 C CNN
F 2 "" H 1200 1950 60  0001 C CNN
F 3 "" H 1200 1950 60  0001 C CNN
	1    1200 1950
	1    0    0    -1  
$EndComp
$Comp
L tinkerforge:GND #PWR014
U 1 1 58061ED9
P 3150 3150
F 0 "#PWR014" H 3150 3150 30  0001 C CNN
F 1 "GND" H 3150 3080 30  0001 C CNN
F 2 "" H 3150 3150 60  0001 C CNN
F 3 "" H 3150 3150 60  0001 C CNN
	1    3150 3150
	1    0    0    -1  
$EndComp
$Comp
L tinkerforge:C C101
U 1 1 56AA387E
P 3350 950
F 0 "C101" V 3400 1050 50  0000 L CNN
F 1 "1uF" V 3200 850 50  0000 L CNN
F 2 "kicad-libraries:C0603F" H 3350 950 60  0001 C CNN
F 3 "" H 3350 950 60  0001 C CNN
	1    3350 950 
	-1   0    0    1   
$EndComp
$Comp
L tinkerforge:C C102
U 1 1 580625D7
P 2650 950
F 0 "C102" V 2700 1050 50  0000 L CNN
F 1 "10uF" V 2500 850 50  0000 L CNN
F 2 "kicad-libraries:C0805E" H 2650 950 60  0001 C CNN
F 3 "" H 2650 950 60  0001 C CNN
	1    2650 950 
	-1   0    0    1   
$EndComp
$Comp
L tinkerforge:GND #PWR015
U 1 1 58062658
P 3350 1250
F 0 "#PWR015" H 3350 1250 30  0001 C CNN
F 1 "GND" H 3350 1180 30  0001 C CNN
F 2 "" H 3350 1250 60  0001 C CNN
F 3 "" H 3350 1250 60  0001 C CNN
	1    3350 1250
	1    0    0    -1  
$EndComp
$Comp
L tinkerforge:GND #PWR016
U 1 1 58062696
P 2650 1250
F 0 "#PWR016" H 2650 1250 30  0001 C CNN
F 1 "GND" H 2650 1180 30  0001 C CNN
F 2 "" H 2650 1250 60  0001 C CNN
F 3 "" H 2650 1250 60  0001 C CNN
	1    2650 1250
	1    0    0    -1  
$EndComp
$Comp
L tinkerforge:VCC #PWR017
U 1 1 580627DF
P 3350 650
F 0 "#PWR017" H 3350 750 30  0001 C CNN
F 1 "VCC" H 3350 750 30  0000 C CNN
F 2 "" H 3350 650 60  0001 C CNN
F 3 "" H 3350 650 60  0001 C CNN
	1    3350 650 
	1    0    0    -1  
$EndComp
$Comp
L tinkerforge:C C103
U 1 1 58062F0B
P 3150 2700
F 0 "C103" V 3200 2800 50  0000 L CNN
F 1 "100nF" V 3000 2600 50  0000 L CNN
F 2 "kicad-libraries:C0603F" H 3150 2700 60  0001 C CNN
F 3 "" H 3150 2700 60  0001 C CNN
	1    3150 2700
	-1   0    0    1   
$EndComp
$Comp
L tinkerforge:VCC #PWR018
U 1 1 58063180
P 3150 2400
F 0 "#PWR018" H 3150 2500 30  0001 C CNN
F 1 "VCC" H 3150 2500 30  0000 C CNN
F 2 "" H 3150 2400 60  0001 C CNN
F 3 "" H 3150 2400 60  0001 C CNN
	1    3150 2400
	1    0    0    -1  
$EndComp
Text GLabel 3050 5750 0    60   Input ~ 0
S-MISO
Text GLabel 2950 4450 0    60   Output ~ 0
S-MOSI
Text GLabel 2950 4350 0    60   Output ~ 0
S-CLK
Text GLabel 2950 4250 0    60   Output ~ 0
S-CS
Text GLabel 2950 3550 0    39   Output ~ 0
RESET
Text GLabel 3050 6450 0    60   Input ~ 0
TXD
Text GLabel 3050 6350 0    60   Output ~ 0
RXD
Text Notes 3900 4400 0    47   ~ 0
SPI Slave\nP0.13 : CH0-DX2 : SEL\nP0.14 : CH0-DX1 : CLK\nP0.15 : CH0-DX0 : MOSI
Text Notes 3900 6450 0    47   ~ 0
TX/RX\nP2.10 : CH1_DOUT0/CH1_DX0 : RX (in/output)\nP2.11 : CH1_DOUT0/CH1_DX0 : TX (in/output)
Text Notes 3900 5800 0    47   ~ 0
SPI Slave\nP2.0 :  CH0-DOUT0 : MISO\n
$Comp
L tinkerforge:LED D101
U 1 1 5806CAB6
P 1600 5450
F 0 "D101" H 1600 5550 50  0000 C CNN
F 1 "LED blue" H 1600 5350 50  0000 C CNN
F 2 "kicad-libraries:D0603E" H 1600 5450 50  0001 C CNN
F 3 "" H 1600 5450 50  0000 C CNN
	1    1600 5450
	1    0    0    -1  
$EndComp
$Comp
L tinkerforge:VCC #PWR021
U 1 1 5806DDBC
P 1150 5000
F 0 "#PWR021" H 1150 5100 30  0001 C CNN
F 1 "VCC" H 1150 5100 30  0000 C CNN
F 2 "" H 1150 5000 60  0001 C CNN
F 3 "" H 1150 5000 60  0001 C CNN
	1    1150 5000
	1    0    0    -1  
$EndComp
$Comp
L tinkerforge:CONN_01X02 P103
U 1 1 5806FC08
P 2050 4200
F 0 "P103" H 2050 4350 50  0000 C CNN
F 1 "Boot" V 2150 4200 50  0000 C CNN
F 2 "kicad-libraries:SolderJumper" H 2050 4100 50  0001 C CNN
F 3 "" H 2050 4100 50  0000 C CNN
	1    2050 4200
	-1   0    0    1   
$EndComp
$Comp
L tinkerforge:GND #PWR022
U 1 1 5806FFC7
P 2300 4250
F 0 "#PWR022" H 2300 4250 30  0001 C CNN
F 1 "GND" H 2300 4180 30  0001 C CNN
F 2 "" H 2300 4250 60  0001 C CNN
F 3 "" H 2300 4250 60  0001 C CNN
	1    2300 4250
	0    -1   1    0   
$EndComp
Text GLabel 2150 1700 2    60   Input ~ 0
S-MISO
Text GLabel 2150 1600 2    60   Output ~ 0
S-MOSI
Text GLabel 2150 1500 2    60   Output ~ 0
S-CLK
Text GLabel 2150 1400 2    60   Output ~ 0
S-CS
NoConn ~ 3250 5350
Wire Wire Line
	1550 1200 1800 1200
Wire Wire Line
	10650 3200 10650 3100
Wire Wire Line
	3150 2900 3150 2950
Wire Wire Line
	3150 2950 3250 2950
Wire Wire Line
	3250 3050 3150 3050
Connection ~ 3150 3050
Wire Wire Line
	1650 700  1650 1300
Wire Wire Line
	2650 750  2650 700 
Wire Wire Line
	3250 2450 3150 2450
Wire Wire Line
	3150 2400 3150 2450
Connection ~ 3150 2950
Connection ~ 3150 2450
Wire Wire Line
	1200 1950 1200 1850
Wire Wire Line
	2300 4250 2250 4250
Wire Wire Line
	1550 1700 1600 1700
Wire Wire Line
	3250 5750 3050 5750
Wire Wire Line
	3050 6350 3250 6350
Wire Wire Line
	3050 6450 3250 6450
Text GLabel 2950 3650 0    39   Input ~ 0
1PPS
NoConn ~ 3250 6050
NoConn ~ 3250 5950
Wire Wire Line
	1650 1300 1550 1300
Wire Wire Line
	3150 3050 3150 3150
Wire Wire Line
	3150 2950 3150 3050
Wire Wire Line
	3150 2450 3150 2500
Wire Wire Line
	3350 650  3350 700 
Connection ~ 3350 700 
Wire Wire Line
	3350 700  3350 750 
Wire Wire Line
	2650 700  3350 700 
Connection ~ 2650 700 
Wire Wire Line
	2950 4250 3250 4250
Wire Wire Line
	2950 4350 3250 4350
Wire Wire Line
	2950 4450 3250 4450
$Comp
L tinkerforge:Cs C1
U 1 1 5F842A02
P 1600 1800
F 0 "C1" H 1500 1850 31  0000 L CNN
F 1 "220pF" H 1450 1750 31  0000 L CNN
F 2 "kicad-libraries:C0402F" H 1700 1738 60  0001 L CNN
F 3 "" H 1600 1800 60  0000 C CNN
	1    1600 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 1700 1650 1700
Connection ~ 1600 1700
Wire Wire Line
	1550 1600 1650 1600
Wire Wire Line
	1550 1500 1650 1500
Wire Wire Line
	1550 1400 1650 1400
Wire Wire Line
	2050 1400 2150 1400
Wire Wire Line
	2050 1500 2150 1500
Wire Wire Line
	2050 1600 2150 1600
Wire Wire Line
	2050 1700 2150 1700
$Comp
L tinkerforge:R_PACK4 RP1
U 1 1 5F84CAE2
P 1850 1750
F 0 "RP1" H 1850 2200 50  0000 C CNN
F 1 "220" H 1850 1700 50  0000 C CNN
F 2 "kicad-libraries:4X0402" H 1850 1750 50  0001 C CNN
F 3 "" H 1850 1750 50  0000 C CNN
	1    1850 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	2250 4150 3250 4150
$Comp
L tinkerforge:GND #PWR02
U 1 1 5F8CDAB2
P 1600 1950
F 0 "#PWR02" H 1600 1950 30  0001 C CNN
F 1 "GND" H 1600 1880 30  0001 C CNN
F 2 "" H 1600 1950 60  0001 C CNN
F 3 "" H 1600 1950 60  0001 C CNN
	1    1600 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 1950 1600 1900
Wire Wire Line
	2650 1250 2650 1150
Wire Wire Line
	3350 1150 3350 1250
Wire Wire Line
	1650 700  2650 700 
$Comp
L tinkerforge:PA1616D U1
U 1 1 5F87DE8B
P 8750 2600
F 0 "U1" H 8725 3153 20  0000 C CNN
F 1 "PA1616D" H 8725 3110 20  0000 C CNN
F 2 "kicad-libraries:PA1616D" H 8550 3000 20  0001 C CNN
F 3 "" H 8550 3000 20  0001 C CNN
	1    8750 2600
	1    0    0    -1  
$EndComp
$Comp
L tinkerforge:VCC #PWR04
U 1 1 5F880AE3
P 6450 1750
F 0 "#PWR04" H 6450 1850 30  0001 C CNN
F 1 "VCC" H 6450 1850 30  0000 C CNN
F 2 "" H 6450 1750 60  0001 C CNN
F 3 "" H 6450 1750 60  0001 C CNN
	1    6450 1750
	1    0    0    -1  
$EndComp
$Comp
L tinkerforge:GND #PWR07
U 1 1 5F882BE5
P 8750 3500
F 0 "#PWR07" H 8750 3500 30  0001 C CNN
F 1 "GND" H 8750 3430 30  0001 C CNN
F 2 "" H 8750 3500 60  0001 C CNN
F 3 "" H 8750 3500 60  0001 C CNN
	1    8750 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	8600 3350 8600 3500
Wire Wire Line
	8600 3500 8700 3500
Wire Wire Line
	8900 3350 8900 3500
Wire Wire Line
	8900 3500 8800 3500
Connection ~ 8750 3500
Wire Wire Line
	8700 3350 8700 3500
Connection ~ 8700 3500
Wire Wire Line
	8700 3500 8750 3500
Wire Wire Line
	8800 3350 8800 3500
Connection ~ 8800 3500
Wire Wire Line
	8800 3500 8750 3500
NoConn ~ 9200 2200
NoConn ~ 9200 2300
NoConn ~ 9200 2400
NoConn ~ 9200 2500
NoConn ~ 9200 2700
NoConn ~ 9200 2600
Text GLabel 7400 2800 0    39   Output ~ 0
TXD
Text GLabel 7400 2900 0    39   Input ~ 0
RXD
$Comp
L tinkerforge:R_PACK4 RP3
U 1 1 5F89D58F
P 5950 2150
F 0 "RP3" V 5900 1950 50  0000 L CNN
F 1 "4,7k" V 6000 1950 50  0000 L CNN
F 2 "kicad-libraries:4X0603" H 5950 2150 50  0001 C CNN
F 3 "" H 5950 2150 50  0000 C CNN
	1    5950 2150
	0    1    1    0   
$EndComp
Text GLabel 9550 2800 2    39   Output ~ 0
1PPS
Wire Wire Line
	9550 2800 9200 2800
Text GLabel 7750 2300 0    39   Input ~ 0
RESET
$Comp
L tinkerforge:Rs R1
U 1 1 5F8AC90D
P 7900 2100
F 0 "R1" V 7950 2100 31  0000 C CNN
F 1 "10k" V 7900 2100 31  0000 C CNN
F 2 "kicad-libraries:R0603F" V 7794 2100 60  0001 C CNN
F 3 "" H 7900 2100 60  0000 C CNN
	1    7900 2100
	-1   0    0    1   
$EndComp
Wire Wire Line
	8100 2200 8250 2200
Wire Wire Line
	8100 1900 8100 2200
Wire Wire Line
	7900 2000 7900 1900
Connection ~ 7900 1900
Wire Wire Line
	7900 1900 8100 1900
Wire Wire Line
	7900 2200 7900 2300
Wire Wire Line
	7900 2300 8250 2300
$Comp
L tinkerforge:Cs C3
U 1 1 5F8C52BE
P 6800 2000
F 0 "C3" V 6850 2100 50  0000 L CNN
F 1 "1uF" V 6650 1950 50  0000 L CNN
F 2 "kicad-libraries:C0603F" H 6800 2000 60  0001 C CNN
F 3 "" H 6800 2000 60  0001 C CNN
	1    6800 2000
	-1   0    0    1   
$EndComp
Connection ~ 6800 1900
Wire Wire Line
	6800 1900 7150 1900
$Comp
L tinkerforge:Cs C4
U 1 1 5F8C82E3
P 7150 2000
F 0 "C4" V 7200 2100 50  0000 L CNN
F 1 "100nF" V 7000 1900 50  0000 L CNN
F 2 "kicad-libraries:C0603F" H 7150 2000 60  0001 C CNN
F 3 "" H 7150 2000 60  0001 C CNN
	1    7150 2000
	-1   0    0    1   
$EndComp
Connection ~ 7150 1900
Wire Wire Line
	7150 1900 7900 1900
Wire Wire Line
	7750 2300 7900 2300
Connection ~ 7900 2300
Wire Wire Line
	6800 2100 6800 2250
Wire Wire Line
	6800 2250 7000 2250
Wire Wire Line
	7150 2250 7150 2100
$Comp
L tinkerforge:GND #PWR06
U 1 1 5F8CDB1C
P 7000 2250
F 0 "#PWR06" H 7000 2250 30  0001 C CNN
F 1 "GND" H 7000 2180 30  0001 C CNN
F 2 "" H 7000 2250 60  0001 C CNN
F 3 "" H 7000 2250 60  0001 C CNN
	1    7000 2250
	1    0    0    -1  
$EndComp
Connection ~ 7000 2250
Wire Wire Line
	7000 2250 7150 2250
$Comp
L tinkerforge:INDUCTs L1
U 1 1 5F8CEEE6
P 6600 1900
F 0 "L1" V 6454 1900 31  0000 C CNN
F 1 "FB" V 6516 1900 31  0000 C CNN
F 2 "kicad-libraries:R0603F" V 6494 1900 60  0001 C CNN
F 3 "" H 6600 1900 60  0000 C CNN
	1    6600 1900
	0    1    1    0   
$EndComp
Wire Wire Line
	6700 1900 6800 1900
$Comp
L tinkerforge:BATTERY BT1
U 1 1 5F8D1681
P 6150 3200
F 0 "BT1" V 6100 2850 50  0000 L CNN
F 1 "CR1025 3V" V 6250 2600 50  0000 L CNN
F 2 "kicad-libraries:BATTERY_HOLDER_BK870" H 6150 3200 60  0001 C CNN
F 3 "" H 6150 3200 60  0000 C CNN
	1    6150 3200
	0    1    1    0   
$EndComp
$Comp
L tinkerforge:Cs C2
U 1 1 5F8D2623
P 6450 3200
F 0 "C2" V 6500 3300 50  0000 L CNN
F 1 "1uF" V 6300 3150 50  0000 L CNN
F 2 "kicad-libraries:C0603E" H 6450 3200 60  0001 C CNN
F 3 "" H 6450 3200 60  0001 C CNN
	1    6450 3200
	-1   0    0    1   
$EndComp
Wire Wire Line
	6300 2600 6300 2350
Wire Wire Line
	6300 2600 8250 2600
Wire Wire Line
	6000 2700 6000 2350
Wire Wire Line
	6000 1950 6000 1900
Wire Wire Line
	6300 1950 6300 1900
Connection ~ 6300 1900
Wire Wire Line
	6100 1900 6100 1950
Wire Wire Line
	6000 1900 6100 1900
Connection ~ 6100 1900
Wire Wire Line
	6100 1900 6200 1900
Wire Wire Line
	6200 1900 6200 1950
Connection ~ 6200 1900
Wire Wire Line
	6200 1900 6300 1900
NoConn ~ 6100 2350
NoConn ~ 6200 2350
Wire Wire Line
	6000 2700 8250 2700
Wire Wire Line
	6300 2600 5800 2600
Connection ~ 6300 2600
Wire Wire Line
	6000 2700 5800 2700
Connection ~ 6000 2700
Text GLabel 2950 3850 0    39   BiDi ~ 0
SDA
Text GLabel 5800 2700 0    39   Input ~ 0
SCL
Wire Wire Line
	6450 1750 6450 1900
Wire Wire Line
	6300 1900 6450 1900
Connection ~ 6450 1900
Wire Wire Line
	6450 1900 6500 1900
$Comp
L tinkerforge:Rs R2
U 1 1 5F9185F6
P 7900 2800
F 0 "R2" V 7950 2800 31  0000 C CNN
F 1 "470" V 7900 2800 31  0000 C CNN
F 2 "kicad-libraries:R0603F" V 7794 2800 60  0001 C CNN
F 3 "" H 7900 2800 60  0000 C CNN
	1    7900 2800
	0    -1   -1   0   
$EndComp
$Comp
L tinkerforge:Rs R3
U 1 1 5F918C35
P 7900 2900
F 0 "R3" V 7950 2900 31  0000 C CNN
F 1 "470" V 7900 2900 31  0000 C CNN
F 2 "kicad-libraries:R0603F" V 7794 2900 60  0001 C CNN
F 3 "" H 7900 2900 60  0000 C CNN
	1    7900 2900
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7400 2800 7800 2800
Wire Wire Line
	7400 2900 7800 2900
Wire Wire Line
	8000 2900 8250 2900
Wire Wire Line
	8250 2800 8000 2800
Wire Wire Line
	6150 2900 6450 2900
Wire Wire Line
	6450 2900 6450 3100
Wire Wire Line
	6450 3300 6450 3500
Wire Wire Line
	6450 3500 6150 3500
Wire Wire Line
	6450 2900 6850 2900
Wire Wire Line
	6850 2900 6850 2400
Wire Wire Line
	6850 2400 8250 2400
Connection ~ 6450 2900
$Comp
L tinkerforge:GND #PWR03
U 1 1 5F9498DE
P 6150 3600
F 0 "#PWR03" H 6150 3600 30  0001 C CNN
F 1 "GND" H 6150 3530 30  0001 C CNN
F 2 "" H 6150 3600 60  0001 C CNN
F 3 "" H 6150 3600 60  0001 C CNN
	1    6150 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	6150 3600 6150 3500
Connection ~ 6150 3500
Text GLabel 2950 3950 0    39   Input ~ 0
SCL
Text GLabel 5800 2600 0    39   BiDi ~ 0
SDA
Wire Wire Line
	2950 3850 3250 3850
Wire Wire Line
	3250 3950 2950 3950
Text Notes 3900 4000 0    39   ~ 0
I2C:\nSDA: USIC0-CH1-DOUT0\nSCL: USIC0-CH1-SCKLOUT
$Comp
L tinkerforge:CONN_1 P1
U 1 1 5F97FAE3
P 2050 4050
F 0 "P1" H 2017 3884 39  0000 C CNN
F 1 "Debug" H 2017 3959 39  0000 C CNN
F 2 "kicad-libraries:DEBUG_PAD" H 2050 4050 60  0001 C CNN
F 3 "" H 2050 4050 60  0000 C CNN
	1    2050 4050
	-1   0    0    1   
$EndComp
Wire Wire Line
	2150 4050 3250 4050
Wire Wire Line
	1150 5000 1150 5150
Wire Wire Line
	1150 5150 1400 5150
$Comp
L tinkerforge:MOSFET_N_CH Q1
U 1 1 5F99F60F
P 10350 1500
F 0 "Q1" H 10540 1454 50  0000 L CNN
F 1 "2N7002" H 10540 1545 50  0000 L CNN
F 2 "kicad-libraries:SOT23GDS" H 10550 1600 50  0001 C CNN
F 3 "" H 10350 1500 50  0000 C CNN
	1    10350 1500
	1    0    0    1   
$EndComp
Wire Wire Line
	10450 1700 10450 1800
$Comp
L tinkerforge:VCC #PWR08
U 1 1 5F9B0229
P 10450 650
F 0 "#PWR08" H 10450 750 30  0001 C CNN
F 1 "VCC" H 10450 750 30  0000 C CNN
F 2 "" H 10450 650 60  0001 C CNN
F 3 "" H 10450 650 60  0001 C CNN
	1    10450 650 
	1    0    0    -1  
$EndComp
Text GLabel 9300 1500 0    39   Input ~ 0
ANT_SWITCH
Text GLabel 7400 2500 0    39   Output ~ 0
3D_FIX
Wire Wire Line
	7400 2500 8250 2500
Text GLabel 2950 5050 0    39   Input ~ 0
3D_FIX
Wire Wire Line
	2950 3750 3250 3750
$Comp
L tinkerforge:LED D1
U 1 1 5F9FABF0
P 1600 5150
F 0 "D1" H 1600 5250 50  0000 C CNN
F 1 "LED green" H 1600 5050 50  0000 C CNN
F 2 "kicad-libraries:D0603E" H 1600 5150 50  0001 C CNN
F 3 "" H 1600 5150 50  0000 C CNN
	1    1600 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	1150 5150 1150 5450
Wire Wire Line
	1150 5450 1400 5450
Connection ~ 1150 5150
$Comp
L tinkerforge:R_PACK4 RP2
U 1 1 5F9FFAC1
P 2200 5500
F 0 "RP2" H 2150 5450 50  0000 L CNN
F 1 "1k" H 2150 5950 50  0000 L CNN
F 2 "kicad-libraries:4X0603" H 2200 5500 50  0001 C CNN
F 3 "" H 2200 5500 50  0000 C CNN
	1    2200 5500
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1800 5150 2000 5150
Wire Wire Line
	2000 5450 1800 5450
Wire Wire Line
	2400 5150 3250 5150
Wire Wire Line
	3250 5250 2550 5250
Wire Wire Line
	2550 5250 2550 5450
Wire Wire Line
	2550 5450 2400 5450
$Comp
L tinkerforge:Cs C5
U 1 1 5FA16BE2
P 10100 2050
F 0 "C5" V 10150 2150 50  0000 L CNN
F 1 "1uF" V 9950 1950 50  0000 L CNN
F 2 "kicad-libraries:C0603F" H 10100 2050 60  0001 C CNN
F 3 "" H 10100 2050 60  0001 C CNN
	1    10100 2050
	-1   0    0    1   
$EndComp
Wire Wire Line
	9200 2900 10450 2900
Wire Wire Line
	10100 2300 10100 2150
Wire Wire Line
	10100 1950 10100 1800
Wire Wire Line
	10100 1800 10450 1800
$Comp
L tinkerforge:INDUCTs L2
U 1 1 5FA321A8
P 10450 2050
F 0 "L2" H 10503 2081 31  0000 L CNN
F 1 "33nH" H 10503 2019 31  0000 L CNN
F 2 "kicad-libraries:0603F" H 10450 2050 60  0001 C CNN
F 3 "" H 10450 2050 60  0000 C CNN
	1    10450 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	10450 1950 10450 1800
Connection ~ 10450 1800
Wire Wire Line
	10450 2150 10450 2900
Connection ~ 10450 2900
Wire Wire Line
	10450 2900 10650 2900
$Comp
L tinkerforge:GND #PWR09
U 1 1 5FA3BFC9
P 10650 3200
F 0 "#PWR09" H 10650 3200 30  0001 C CNN
F 1 "GND" H 10650 3130 30  0001 C CNN
F 2 "" H 10650 3200 60  0001 C CNN
F 3 "" H 10650 3200 60  0001 C CNN
	1    10650 3200
	1    0    0    -1  
$EndComp
$Comp
L tinkerforge:Cs C6
U 1 1 5F884C89
P 10100 950
F 0 "C6" V 10150 1050 50  0000 L CNN
F 1 "1uF" V 9950 850 50  0000 L CNN
F 2 "kicad-libraries:C0603F" H 10100 950 60  0001 C CNN
F 3 "" H 10100 950 60  0001 C CNN
	1    10100 950 
	-1   0    0    1   
$EndComp
$Comp
L tinkerforge:GND #PWR010
U 1 1 5F887A57
P 10100 1250
F 0 "#PWR010" H 10100 1250 30  0001 C CNN
F 1 "GND" H 10100 1180 30  0001 C CNN
F 2 "" H 10100 1250 60  0001 C CNN
F 3 "" H 10100 1250 60  0001 C CNN
	1    10100 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	10100 1250 10100 1050
$Comp
L tinkerforge:INDUCTs L3
U 1 1 5F88D021
P 10450 950
F 0 "L3" V 10304 950 31  0000 C CNN
F 1 "FB" V 10366 950 31  0000 C CNN
F 2 "kicad-libraries:R0603F" V 10344 950 60  0001 C CNN
F 3 "" H 10450 950 60  0000 C CNN
	1    10450 950 
	-1   0    0    1   
$EndComp
Wire Wire Line
	10100 850  10100 750 
Wire Wire Line
	10100 750  10450 750 
Wire Wire Line
	10450 750  10450 650 
Wire Wire Line
	10450 750  10450 850 
Connection ~ 10450 750 
Wire Wire Line
	10450 1050 10450 1300
Wire Wire Line
	3250 3650 2950 3650
Wire Wire Line
	2950 3550 3250 3550
NoConn ~ 2000 5250
NoConn ~ 2000 5350
NoConn ~ 2400 5250
NoConn ~ 2400 5350
$Comp
L tinkerforge:Rs R4
U 1 1 5F924702
P 9650 1500
F 0 "R4" V 9700 1500 31  0000 C CNN
F 1 "1k" V 9650 1500 31  0000 C CNN
F 2 "kicad-libraries:R0603F" V 9544 1500 60  0001 C CNN
F 3 "" H 9650 1500 60  0000 C CNN
	1    9650 1500
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9300 1500 9450 1500
Wire Wire Line
	9750 1500 10150 1500
Text GLabel 2950 3750 0    39   Input ~ 0
ANT_SWITCH
Wire Wire Line
	2950 5050 3250 5050
$Comp
L tinkerforge:Rs R5
U 1 1 5F966BB2
P 9450 1600
F 0 "R5" V 9500 1600 31  0000 C CNN
F 1 "10k" V 9450 1600 31  0000 C CNN
F 2 "kicad-libraries:R0603F" V 9344 1600 60  0001 C CNN
F 3 "" H 9450 1600 60  0000 C CNN
	1    9450 1600
	1    0    0    -1  
$EndComp
Connection ~ 9450 1500
Wire Wire Line
	9450 1500 9550 1500
$Comp
L tinkerforge:GND #PWR011
U 1 1 5F9693EA
P 9450 1700
F 0 "#PWR011" H 9450 1700 30  0001 C CNN
F 1 "GND" H 9450 1630 30  0001 C CNN
F 2 "" H 9450 1700 60  0001 C CNN
F 3 "" H 9450 1700 60  0001 C CNN
	1    9450 1700
	1    0    0    -1  
$EndComp
$EndSCHEMATC
