EESchema Schematic File Version 4
LIBS:LedLamp-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Transistor_FET:IRF540N Q2
U 1 1 5BD621DF
P 3850 2000
F 0 "Q2" V 4193 2000 50  0000 C CNN
F 1 "IRF540N" V 4102 2000 50  0000 C CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 4100 1925 50  0001 L CIN
F 3 "http://www.irf.com/product-info/datasheets/data/irf540n.pdf" H 3850 2000 50  0001 L CNN
	1    3850 2000
	0    -1   -1   0   
$EndComp
$Comp
L Transistor_FET:IRF540N Q3
U 1 1 5BD622D4
P 4750 2000
F 0 "Q3" V 5093 2000 50  0000 C CNN
F 1 "IRF540N" V 5002 2000 50  0000 C CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 5000 1925 50  0001 L CIN
F 3 "http://www.irf.com/product-info/datasheets/data/irf540n.pdf" H 4750 2000 50  0001 L CNN
	1    4750 2000
	0    -1   -1   0   
$EndComp
$Comp
L Transistor_FET:IRF540N Q1
U 1 1 5BD6246A
P 3100 2000
F 0 "Q1" V 3443 2000 50  0000 C CNN
F 1 "IRF540N" V 3352 2000 50  0000 C CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 3350 1925 50  0001 L CIN
F 3 "http://www.irf.com/product-info/datasheets/data/irf540n.pdf" H 3100 2000 50  0001 L CNN
	1    3100 2000
	0    -1   -1   0   
$EndComp
$Comp
L LedLamp-rescue:ATmega328P-PU-MCU_Microchip_ATmega U1
U 1 1 5BD6257B
P 3850 4050
F 0 "U1" V 3896 2509 50  0000 R CNN
F 1 "ATmega328P-PU" V 3805 2509 50  0000 R CNN
F 2 "Package_DIP:DIP-28_W7.62mm" H 3850 4050 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/ATmega328_P%20AVR%20MCU%20with%20picoPower%20Technology%20Data%20Sheet%2040001984A.pdf" H 3850 4050 50  0001 C CNN
	1    3850 4050
	0    -1   -1   0   
$EndComp
$Comp
L Connector:Conn_01x04_Male J1
U 1 1 5BD62808
P 3800 850
F 0 "J1" V 3860 990 50  0000 L CNN
F 1 "RGB+Vcc" V 3951 990 50  0000 L CNN
F 2 "Connector:FanPinHeader_1x04_P2.54mm_Vertical" H 3800 850 50  0001 C CNN
F 3 "~" H 3800 850 50  0001 C CNN
	1    3800 850 
	0    1    1    0   
$EndComp
$Comp
L Switch:SW_Push SW1
U 1 1 5BD62CB3
P 6750 2250
F 0 "SW1" H 6750 2535 50  0000 C CNN
F 1 "button" H 6750 2444 50  0000 C CNN
F 2 "Button_Switch_THT:SW_PUSH_6mm_H5mm" H 6750 2450 50  0001 C CNN
F 3 "" H 6750 2450 50  0001 C CNN
	1    6750 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4650 2650 4650 3450
Wire Wire Line
	2750 3450 2750 2700
Wire Wire Line
	2750 2700 3100 2700
Wire Wire Line
	4750 2550 4950 2550
Wire Wire Line
	4950 2550 4950 3450
$Comp
L power:GND #PWR0101
U 1 1 5BD63C3C
P 3400 1950
F 0 "#PWR0101" H 3400 1700 50  0001 C CNN
F 1 "GND" H 3405 1777 50  0000 C CNN
F 2 "" H 3400 1950 50  0001 C CNN
F 3 "" H 3400 1950 50  0001 C CNN
	1    3400 1950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 5BD63C80
P 4150 1950
F 0 "#PWR0102" H 4150 1700 50  0001 C CNN
F 1 "GND" H 4155 1777 50  0000 C CNN
F 2 "" H 4150 1950 50  0001 C CNN
F 3 "" H 4150 1950 50  0001 C CNN
	1    4150 1950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 5BD63CB6
P 5050 2000
F 0 "#PWR0103" H 5050 1750 50  0001 C CNN
F 1 "GND" H 5055 1827 50  0000 C CNN
F 2 "" H 5050 2000 50  0001 C CNN
F 3 "" H 5050 2000 50  0001 C CNN
	1    5050 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	4950 1900 5050 1900
Wire Wire Line
	5050 1900 5050 2000
Wire Wire Line
	4050 1900 4150 1900
Wire Wire Line
	4150 1900 4150 1950
Wire Wire Line
	3300 1900 3400 1900
Wire Wire Line
	3400 1900 3400 1950
Wire Wire Line
	2900 1900 2900 1400
Wire Wire Line
	3700 1400 3700 1050
Wire Wire Line
	2900 1400 3700 1400
Wire Wire Line
	3650 1900 3650 1500
Wire Wire Line
	3650 1500 3800 1500
Wire Wire Line
	3800 1500 3800 1050
Wire Wire Line
	4550 1900 4550 1500
Wire Wire Line
	4550 1500 3900 1500
Wire Wire Line
	3900 1500 3900 1050
Wire Wire Line
	4650 2650 3850 2650
Text Label 4300 3000 0    50   ~ 0
Button
Wire Wire Line
	4350 3450 4350 3000
Wire Wire Line
	4350 3000 4300 3000
Text Label 7400 1900 0    50   ~ 0
Button
$Comp
L power:VCC #PWR0106
U 1 1 5BD6A609
P 3250 1050
F 0 "#PWR0106" H 3250 900 50  0001 C CNN
F 1 "VCC" H 3267 1223 50  0000 C CNN
F 2 "" H 3250 1050 50  0001 C CNN
F 3 "" H 3250 1050 50  0001 C CNN
	1    3250 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 1050 3600 1050
$Comp
L Device:CP1 C4
U 1 1 5BD6AD21
P 2150 1200
F 0 "C4" H 2265 1246 50  0000 L CNN
F 1 "100u" H 2265 1155 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D8.0mm_P5.00mm" H 2150 1200 50  0001 C CNN
F 3 "~" H 2150 1200 50  0001 C CNN
	1    2150 1200
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1 C3
U 1 1 5BD6AECC
P 1750 1200
F 0 "C3" H 1865 1246 50  0000 L CNN
F 1 "100u" H 1865 1155 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D8.0mm_P5.00mm" H 1750 1200 50  0001 C CNN
F 3 "~" H 1750 1200 50  0001 C CNN
	1    1750 1200
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1 C2
U 1 1 5BD6AF68
P 1350 1200
F 0 "C2" H 1465 1246 50  0000 L CNN
F 1 "100u" H 1465 1155 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D8.0mm_P5.00mm" H 1350 1200 50  0001 C CNN
F 3 "~" H 1350 1200 50  0001 C CNN
	1    1350 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	1350 1050 1750 1050
Wire Wire Line
	1750 1050 2150 1050
Connection ~ 1750 1050
Wire Wire Line
	1350 1350 1750 1350
Wire Wire Line
	1750 1350 2150 1350
Connection ~ 1750 1350
Wire Wire Line
	1750 1050 1750 950 
$Comp
L power:VCC #PWR0107
U 1 1 5BD6DCFF
P 1750 950
F 0 "#PWR0107" H 1750 800 50  0001 C CNN
F 1 "VCC" H 1767 1123 50  0000 C CNN
F 2 "" H 1750 950 50  0001 C CNN
F 3 "" H 1750 950 50  0001 C CNN
	1    1750 950 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0108
U 1 1 5BD6DD4D
P 1750 1450
F 0 "#PWR0108" H 1750 1200 50  0001 C CNN
F 1 "GND" H 1755 1277 50  0000 C CNN
F 2 "" H 1750 1450 50  0001 C CNN
F 3 "" H 1750 1450 50  0001 C CNN
	1    1750 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	1750 1350 1750 1450
$Comp
L Connector:Screw_Terminal_01x02 J2
U 1 1 5BD77473
P 1200 2200
F 0 "J2" H 1280 2192 50  0000 L CNN
F 1 "Bornier" H 1280 2101 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x03_P2.54mm_Vertical" H 1200 2200 50  0001 C CNN
F 3 "~" H 1200 2200 50  0001 C CNN
	1    1200 2200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0112
U 1 1 5BD77645
P 850 2300
F 0 "#PWR0112" H 850 2050 50  0001 C CNN
F 1 "GND" H 855 2127 50  0000 C CNN
F 2 "" H 850 2300 50  0001 C CNN
F 3 "" H 850 2300 50  0001 C CNN
	1    850  2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	850  2300 1000 2300
$Comp
L power:VCC #PWR0113
U 1 1 5BD781E8
P 800 2200
F 0 "#PWR0113" H 800 2050 50  0001 C CNN
F 1 "VCC" H 817 2373 50  0000 C CNN
F 2 "" H 800 2200 50  0001 C CNN
F 3 "" H 800 2200 50  0001 C CNN
	1    800  2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	800  2200 1000 2200
Wire Wire Line
	7350 2250 7400 2250
$Comp
L Device:R R4
U 1 1 5BDCECC9
P 7200 2250
F 0 "R4" V 7200 2250 50  0000 C CNN
F 1 "1k" V 7084 2250 50  0000 C CNN
F 2 "" V 7130 2250 50  0001 C CNN
F 3 "~" H 7200 2250 50  0001 C CNN
	1    7200 2250
	0    1    1    0   
$EndComp
Wire Wire Line
	7050 2250 6950 2250
Wire Wire Line
	7400 2250 7400 1900
$Comp
L power:GND #PWR01
U 1 1 5BDD1639
P 6550 2450
F 0 "#PWR01" H 6550 2200 50  0001 C CNN
F 1 "GND" H 6555 2277 50  0000 C CNN
F 2 "" H 6550 2450 50  0001 C CNN
F 3 "" H 6550 2450 50  0001 C CNN
	1    6550 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	6550 2250 6550 2450
Text Notes 7350 2650 0    50   ~ 0
Button
Wire Wire Line
	3100 2200 3100 2700
Wire Wire Line
	3850 2200 3850 2650
Wire Wire Line
	4750 2200 4750 2550
Wire Notes Line
	7800 1650 7800 2800
Wire Notes Line
	7800 2800 6300 2800
Wire Notes Line
	6300 2800 6300 1650
Wire Notes Line
	6300 1650 7800 1650
$EndSCHEMATC
