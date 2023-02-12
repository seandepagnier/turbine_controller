EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 2
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text GLabel 4600 1750 2    50   Input ~ 0
SYNC
Wire Wire Line
	3950 5200 3950 5050
Wire Wire Line
	3900 5200 3950 5200
Text GLabel 3600 5200 0    50   Input ~ 0
SYNC
$Comp
L power:GND #PWR?
U 1 1 60BA3D58
P 4350 5050
AR Path="/60BA3D58" Ref="#PWR?"  Part="1" 
AR Path="/60AD8114/60BA3D58" Ref="#PWR0124"  Part="1" 
F 0 "#PWR0124" H 4350 4800 50  0001 C CNN
F 1 "GND" H 4355 4877 50  0001 C CNN
F 2 "" H 4350 5050 50  0001 C CNN
F 3 "" H 4350 5050 50  0001 C CNN
	1    4350 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	4350 4750 4650 4750
Wire Wire Line
	6500 4750 6350 4750
$Comp
L power:GND #PWR?
U 1 1 60BA3D60
P 6500 5050
AR Path="/60BA3D60" Ref="#PWR?"  Part="1" 
AR Path="/60AD8114/60BA3D60" Ref="#PWR0125"  Part="1" 
F 0 "#PWR0125" H 6500 4800 50  0001 C CNN
F 1 "GND" H 6505 4877 50  0001 C CNN
F 2 "" H 6500 5050 50  0001 C CNN
F 3 "" H 6500 5050 50  0001 C CNN
	1    6500 5050
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 60BA3D67
P 6500 4900
AR Path="/60BA3D67" Ref="C?"  Part="1" 
AR Path="/60AD8114/60BA3D67" Ref="C24"  Part="1" 
F 0 "C24" H 6550 4800 50  0000 L CNN
F 1 "1uF" H 6550 5000 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 6538 4750 50  0001 C CNN
F 3 "~" H 6500 4900 50  0001 C CNN
F 4 "C28323" H 6500 4900 50  0001 C CNN "LCSC"
	1    6500 4900
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 60BA3D6E
P 4350 4900
AR Path="/60BA3D6E" Ref="C?"  Part="1" 
AR Path="/60AD8114/60BA3D6E" Ref="C22"  Part="1" 
F 0 "C22" H 4400 4800 50  0000 L CNN
F 1 "1uF" H 4400 5000 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4388 4750 50  0001 C CNN
F 3 "~" H 4350 4900 50  0001 C CNN
F 4 "C28323" H 4350 4900 50  0001 C CNN "LCSC"
	1    4350 4900
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 60BA3D74
P 3750 4750
AR Path="/60BA3D74" Ref="#PWR?"  Part="1" 
AR Path="/60AD8114/60BA3D74" Ref="#PWR0128"  Part="1" 
F 0 "#PWR0128" H 3750 4600 50  0001 C CNN
F 1 "+5V" H 3800 4900 50  0000 C CNN
F 2 "" H 3750 4750 50  0001 C CNN
F 3 "" H 3750 4750 50  0001 C CNN
	1    3750 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 5050 3950 5050
$Comp
L Device:R R?
U 1 1 60BA3D7C
P 3750 4900
AR Path="/60BA3D7C" Ref="R?"  Part="1" 
AR Path="/60AD8114/60BA3D7C" Ref="R14"  Part="1" 
F 0 "R14" H 3550 4900 50  0000 L CNN
F 1 "100k" V 3750 4800 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 3680 4900 50  0001 C CNN
F 3 "~" H 3750 4900 50  0001 C CNN
F 4 "C17407" H 3750 4900 50  0001 C CNN "LCSC"
	1    3750 4900
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 60BA3D83
P 5800 4900
AR Path="/60BA3D83" Ref="R?"  Part="1" 
AR Path="/60AD8114/60BA3D83" Ref="R18"  Part="1" 
F 0 "R18" V 5900 4850 50  0000 L CNN
F 1 "100k" V 5800 4800 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 5730 4900 50  0001 C CNN
F 3 "~" H 5800 4900 50  0001 C CNN
F 4 "C17407" H 5800 4900 50  0001 C CNN "LCSC"
	1    5800 4900
	1    0    0    -1  
$EndComp
Connection ~ 3750 4750
Connection ~ 3950 5050
$Comp
L Device:Q_PMOS_GSD Q?
U 1 1 60BA3D8C
P 3950 4850
AR Path="/60BA3D8C" Ref="Q?"  Part="1" 
AR Path="/60AD8114/60BA3D8C" Ref="Q1"  Part="1" 
F 0 "Q1" V 3850 4700 50  0000 L CNN
F 1 "pFET" H 3800 5000 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 4150 4950 50  0001 C CNN
F 3 "~" H 3950 4850 50  0001 C CNN
F 4 " C15127" H 3950 4850 50  0001 C CNN "LCSC"
	1    3950 4850
	0    1    -1   0   
$EndComp
$Comp
L power:VCC #PWR0129
U 1 1 60BA3DA2
P 3350 3800
F 0 "#PWR0129" H 3350 3650 50  0001 C CNN
F 1 "VCC" H 3367 3973 50  0000 C CNN
F 2 "" H 3350 3800 50  0001 C CNN
F 3 "" H 3350 3800 50  0001 C CNN
	1    3350 3800
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR0130
U 1 1 60BA3DB0
P 3350 4400
F 0 "#PWR0130" H 3350 4150 50  0001 C CNN
F 1 "GND" H 3355 4227 50  0001 C CNN
F 2 "" H 3350 4400 50  0001 C CNN
F 3 "" H 3350 4400 50  0001 C CNN
	1    3350 4400
	-1   0    0    -1  
$EndComp
Connection ~ 3350 4100
$Comp
L Device:R R7
U 1 1 60BA3DB9
P 3350 3950
F 0 "R7" H 3400 4000 50  0000 L CNN
F 1 "150k" V 3350 3850 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 3280 3950 50  0001 C CNN
F 3 "~" H 3350 3950 50  0001 C CNN
F 4 "C17470" H 3350 3950 50  0001 C CNN "LCSC"
	1    3350 3950
	1    0    0    1   
$EndComp
$Comp
L Device:R R8
U 1 1 60BA3DC1
P 3350 4250
F 0 "R8" H 3400 4300 50  0000 L CNN
F 1 "10k" V 3350 4150 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 3280 4250 50  0001 C CNN
F 3 "~" H 3350 4250 50  0001 C CNN
F 4 "C17407" H 3350 4250 50  0001 C CNN "LCSC"
	1    3350 4250
	1    0    0    1   
$EndComp
$Comp
L power:+5VD #PWR0131
U 1 1 60BA3DC7
P 1850 2150
F 0 "#PWR0131" H 1850 2000 50  0001 C CNN
F 1 "+5VD" H 1700 2250 50  0000 C CNN
F 2 "" H 1850 2150 50  0001 C CNN
F 3 "" H 1850 2150 50  0001 C CNN
	1    1850 2150
	1    0    0    -1  
$EndComp
Connection ~ 4350 4750
Wire Wire Line
	4150 4750 4350 4750
$Comp
L power:GND #PWR?
U 1 1 60BA3DD5
P 4650 5050
AR Path="/60BA3DD5" Ref="#PWR?"  Part="1" 
AR Path="/60AD8114/60BA3DD5" Ref="#PWR0133"  Part="1" 
F 0 "#PWR0133" H 4650 4800 50  0001 C CNN
F 1 "GND" H 4655 4877 50  0001 C CNN
F 2 "" H 4650 5050 50  0001 C CNN
F 3 "" H 4650 5050 50  0001 C CNN
	1    4650 5050
	1    0    0    -1  
$EndComp
$Comp
L Device:R R15
U 1 1 60BA3DDC
P 4650 4900
F 0 "R15" V 4750 4850 50  0000 L CNN
F 1 "100k" V 4650 4800 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 4580 4900 50  0001 C CNN
F 3 "~" H 4650 4900 50  0001 C CNN
F 4 "C17407" H 4650 4900 50  0001 C CNN "LCSC"
	1    4650 4900
	1    0    0    -1  
$EndComp
Connection ~ 4650 4750
$Comp
L power:+5VD #PWR?
U 1 1 60BA3DE3
P 4650 4750
AR Path="/60BA3DE3" Ref="#PWR?"  Part="1" 
AR Path="/60AD8114/60BA3DE3" Ref="#PWR0135"  Part="1" 
F 0 "#PWR0135" H 4650 4600 50  0001 C CNN
F 1 "+5VD" H 4665 4923 50  0000 C CNN
F 2 "" H 4650 4750 50  0001 C CNN
F 3 "" H 4650 4750 50  0001 C CNN
	1    4650 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	5950 4750 5800 4750
Wire Wire Line
	5800 5050 6150 5050
$Comp
L power:GND #PWR?
U 1 1 60BA3DF2
P 6150 5450
AR Path="/60BA3DF2" Ref="#PWR?"  Part="1" 
AR Path="/60AD8114/60BA3DF2" Ref="#PWR0136"  Part="1" 
F 0 "#PWR0136" H 6150 5200 50  0001 C CNN
F 1 "GND" H 6155 5277 50  0001 C CNN
F 2 "" H 6150 5450 50  0001 C CNN
F 3 "" H 6150 5450 50  0001 C CNN
	1    6150 5450
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_NPN_BEC Q?
U 1 1 60BA3DF9
P 6050 5250
AR Path="/60BA3DF9" Ref="Q?"  Part="1" 
AR Path="/60AD8114/60BA3DF9" Ref="Q2"  Part="1" 
F 0 "Q2" H 6240 5296 50  0000 L CNN
F 1 "npn" H 6240 5205 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 6250 5350 50  0001 C CNN
F 3 "~" H 6050 5250 50  0001 C CNN
F 4 "C20526" H 6050 5250 50  0001 C CNN "LCSC"
	1    6050 5250
	1    0    0    -1  
$EndComp
Connection ~ 5800 4750
$Comp
L power:VCC #PWR0138
U 1 1 60BA3E00
P 5800 4750
F 0 "#PWR0138" H 5800 4600 50  0001 C CNN
F 1 "VCC" H 5817 4923 50  0000 C CNN
F 2 "" H 5800 4750 50  0001 C CNN
F 3 "" H 5800 4750 50  0001 C CNN
	1    5800 4750
	1    0    0    -1  
$EndComp
Connection ~ 6500 4750
$Comp
L power:VDD #PWR?
U 1 1 60BA3E07
P 6500 4750
AR Path="/60BA3E07" Ref="#PWR?"  Part="1" 
AR Path="/60AD8114/60BA3E07" Ref="#PWR0139"  Part="1" 
F 0 "#PWR0139" H 6500 4600 50  0001 C CNN
F 1 "VDD" H 6517 4923 50  0000 C CNN
F 2 "" H 6500 4750 50  0001 C CNN
F 3 "" H 6500 4750 50  0001 C CNN
	1    6500 4750
	1    0    0    -1  
$EndComp
Connection ~ 6150 5050
$Comp
L Device:Q_PMOS_GSD Q?
U 1 1 60BA3E0F
P 6150 4850
AR Path="/60BA3E0F" Ref="Q?"  Part="1" 
AR Path="/60AD8114/60BA3E0F" Ref="Q3"  Part="1" 
F 0 "Q3" V 6050 4700 50  0000 L CNN
F 1 "pFET" H 6000 5000 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 6350 4950 50  0001 C CNN
F 3 "~" H 6150 4850 50  0001 C CNN
F 4 "C15127" H 6150 4850 50  0001 C CNN "LCSC"
	1    6150 4850
	0    1    -1   0   
$EndComp
$Comp
L power:GND #PWR0140
U 1 1 60BA3E15
P 1850 2450
F 0 "#PWR0140" H 1850 2200 50  0001 C CNN
F 1 "GND" H 1855 2277 50  0001 C CNN
F 2 "" H 1850 2450 50  0001 C CNN
F 3 "" H 1850 2450 50  0001 C CNN
	1    1850 2450
	1    0    0    -1  
$EndComp
$Comp
L Device:C C23
U 1 1 60BA3E2E
P 5800 6000
F 0 "C23" H 5800 6100 50  0000 L CNN
F 1 ".1uF" H 5600 6200 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5838 5850 50  0001 C CNN
F 3 "~" H 5800 6000 50  0001 C CNN
F 4 "C49678" H 5800 6000 50  0001 C CNN "LCSC"
	1    5800 6000
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 60BA3E3F
P 1850 2300
AR Path="/60BA3E3F" Ref="C?"  Part="1" 
AR Path="/60AD8114/60BA3E3F" Ref="C16"  Part="1" 
F 0 "C16" H 1650 2200 50  0000 L CNN
F 1 ".1uF" H 1650 2400 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 1888 2150 50  0001 C CNN
F 3 "~" H 1850 2300 50  0001 C CNN
F 4 "C49678" H 1850 2300 50  0001 C CNN "LCSC"
	1    1850 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	4250 5950 4250 6050
Wire Wire Line
	4800 5950 4250 5950
Wire Wire Line
	4800 6250 4700 6250
Wire Wire Line
	4650 6050 4800 6050
Wire Wire Line
	4650 6250 4650 6050
Wire Wire Line
	4550 6150 4800 6150
Wire Wire Line
	5950 6250 5700 6250
$Comp
L power:GND #PWR0141
U 1 1 60BA3E6A
P 5800 6150
F 0 "#PWR0141" H 5800 5900 50  0001 C CNN
F 1 "GND" H 5805 5977 50  0001 C CNN
F 2 "" H 5800 6150 50  0001 C CNN
F 3 "" H 5800 6150 50  0001 C CNN
	1    5800 6150
	1    0    0    -1  
$EndComp
Connection ~ 5800 6150
Wire Wire Line
	5700 6150 5800 6150
Wire Wire Line
	5800 5750 5800 5850
Wire Wire Line
	6000 5750 5800 5750
Connection ~ 5800 5850
Wire Wire Line
	5700 5850 5800 5850
Wire Wire Line
	5700 5950 5700 5850
Text GLabel 4600 2950 2    50   Output ~ 0
TX
Text GLabel 5950 6250 2    50   Input ~ 0
TX
Text GLabel 4600 2850 2    50   Input ~ 0
RX
Text GLabel 6050 6050 2    50   Output ~ 0
RX
Wire Wire Line
	6000 6050 5700 6050
Wire Wire Line
	6050 6050 6000 6050
Connection ~ 6000 6050
$Comp
L Device:R R16
U 1 1 60BA3E80
P 6000 5900
F 0 "R16" H 5800 5850 50  0000 L CNN
F 1 "2.4k" V 6000 5800 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 5930 5900 50  0001 C CNN
F 3 "~" H 6000 5900 50  0001 C CNN
F 4 "C17526" H 6000 5900 50  0001 C CNN "LCSC"
	1    6000 5900
	-1   0    0    1   
$EndComp
$Comp
L uart:cpc5001 U10
U 1 1 60BA3E86
P 5250 6100
F 0 "U10" H 5250 6497 60  0000 C CNN
F 1 "cpc5001" H 5250 6391 60  0000 C CNN
F 2 "Package_DIP:DIP-8_W7.62mm" H 5250 6100 60  0001 C CNN
F 3 "" H 5250 6100 60  0001 C CNN
	1    5250 6100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 2750 4600 2650
Text GLabel 6450 1450 2    50   Input ~ 0
MOSI
Text GLabel 5200 1650 2    50   Input ~ 0
SCK
Text GLabel 5800 1550 2    50   Input ~ 0
MISO
$Comp
L power:+5V #PWR?
U 1 1 60BA3EA0
P 2650 5050
AR Path="/60BA3EA0" Ref="#PWR?"  Part="1" 
AR Path="/60AD8114/60BA3EA0" Ref="#PWR0143"  Part="1" 
F 0 "#PWR0143" H 2650 4900 50  0001 C CNN
F 1 "+5V" H 2700 5200 50  0000 C CNN
F 2 "" H 2650 5050 50  0001 C CNN
F 3 "" H 2650 5050 50  0001 C CNN
	1    2650 5050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0144
U 1 1 60BA3EA6
P 2650 5250
F 0 "#PWR0144" H 2650 5000 50  0001 C CNN
F 1 "GND" H 2655 5077 50  0001 C CNN
F 2 "" H 2650 5250 50  0001 C CNN
F 3 "" H 2650 5250 50  0001 C CNN
	1    2650 5250
	-1   0    0    -1  
$EndComp
Text GLabel 2650 5150 2    50   Input ~ 0
MOSI
Text GLabel 2150 5250 0    50   Output ~ 0
RESET
Text GLabel 2150 5150 0    50   Input ~ 0
SCK
Text GLabel 2150 5050 0    50   Input ~ 0
MISO
$Comp
L Connector_Generic:Conn_02x03_Top_Bottom J5
U 1 1 60BA3EB0
P 2350 5150
F 0 "J5" H 2400 5467 50  0000 C CNN
F 1 "ISP" H 2400 5376 50  0000 C CNN
F 2 "footprints:ISP" H 2350 5150 50  0001 C CNN
F 3 "~" H 2350 5150 50  0001 C CNN
	1    2350 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 1650 4600 1650
$Comp
L power:GND #PWR?
U 1 1 60BA3EB9
P 6550 3050
AR Path="/60BA3EB9" Ref="#PWR?"  Part="1" 
AR Path="/60AD8114/60BA3EB9" Ref="#PWR0145"  Part="1" 
F 0 "#PWR0145" H 6550 2800 50  0001 C CNN
F 1 "GND" H 6555 2877 50  0001 C CNN
F 2 "" H 6550 3050 50  0001 C CNN
F 3 "" H 6550 3050 50  0001 C CNN
	1    6550 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	1300 1600 1250 1600
$Comp
L MCU_Microchip_ATmega:ATmega328-AU U9
U 1 1 60BA3EC6
P 4000 2350
F 0 "U9" H 3900 2650 50  0000 C CNN
F 1 "ATmega328-AU" H 3950 2300 50  0000 C CNN
F 2 "Package_QFP:TQFP-32_7x7mm_P0.8mm" H 4000 2350 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/ATmega328_P%20AVR%20MCU%20with%20picoPower%20Technology%20Data%20Sheet%2040001984A.pdf" H 4000 2350 50  0001 C CNN
F 4 "C14877" H 4000 2350 50  0001 C CNN "LCSC"
	1    4000 2350
	1    0    0    -1  
$EndComp
$Comp
L Device:C C19
U 1 1 60BA3ECD
P 3250 1150
F 0 "C19" V 3400 1000 50  0000 L CNN
F 1 ".1uF" V 3200 900 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3288 1000 50  0001 C CNN
F 3 "~" H 3250 1150 50  0001 C CNN
F 4 "C49678" H 3250 1150 50  0001 C CNN "LCSC"
	1    3250 1150
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR0148
U 1 1 60BA3ED9
P 4000 850
F 0 "#PWR0148" H 4000 700 50  0001 C CNN
F 1 "+5V" H 4015 1023 50  0000 C CNN
F 2 "" H 4000 850 50  0001 C CNN
F 3 "" H 4000 850 50  0001 C CNN
	1    4000 850 
	1    0    0    -1  
$EndComp
$Comp
L Device:C C20
U 1 1 60BA3EE0
P 4250 850
F 0 "C20" V 4300 650 50  0000 L CNN
F 1 "10uF" V 4400 650 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 4288 700 50  0001 C CNN
F 3 "~" H 4250 850 50  0001 C CNN
F 4 "C13585" H 4250 850 50  0001 C CNN "LCSC"
	1    4250 850 
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0149
U 1 1 60BA3EE6
P 4000 3850
F 0 "#PWR0149" H 4000 3600 50  0001 C CNN
F 1 "GND" H 4005 3677 50  0001 C CNN
F 2 "" H 4000 3850 50  0001 C CNN
F 3 "" H 4000 3850 50  0001 C CNN
	1    4000 3850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0150
U 1 1 60BA3EEC
P 4400 850
F 0 "#PWR0150" H 4400 600 50  0001 C CNN
F 1 "GND" H 4405 677 50  0001 C CNN
F 2 "" H 4400 850 50  0001 C CNN
F 3 "" H 4400 850 50  0001 C CNN
	1    4400 850 
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3400 1350 3050 1350
$Comp
L Device:LED D6
U 1 1 60BA3EFB
P 5200 1800
F 0 "D6" V 5239 1683 50  0000 R CNN
F 1 "BLUE" V 5148 1683 50  0000 R CNN
F 2 "LED_SMD:LED_0805_2012Metric" H 5200 1800 50  0001 C CNN
F 3 "~" H 5200 1800 50  0001 C CNN
F 4 "C2293" V 5200 1800 50  0001 C CNN "LCSC"
	1    5200 1800
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R10
U 1 1 60BA3F02
P 6550 2900
F 0 "R10" V 6650 2850 50  0000 L CNN
F 1 "2.4k" V 6550 2800 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 6480 2900 50  0001 C CNN
F 3 "~" H 6550 2900 50  0001 C CNN
F 4 "C17526" H 6550 2900 50  0001 C CNN "LCSC"
	1    6550 2900
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D5
U 1 1 60BA3F09
P 6550 2600
F 0 "D5" V 6589 2482 50  0000 R CNN
F 1 "RED" V 6498 2482 50  0000 R CNN
F 2 "LED_SMD:LED_0805_2012Metric" H 6550 2600 50  0001 C CNN
F 3 "~" H 6550 2600 50  0001 C CNN
F 4 "C17526" V 6550 2600 50  0001 C CNN "LCSC"
	1    6550 2600
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3050 1350 3050 1650
$Comp
L Device:R R6
U 1 1 60BA3F11
P 2850 1800
F 0 "R6" V 2950 1700 50  0000 L CNN
F 1 "10k" V 2850 1700 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2780 1800 50  0001 C CNN
F 3 "~" H 2850 1800 50  0001 C CNN
F 4 "C17414" V 2850 1800 50  0001 C CNN "LCSC"
	1    2850 1800
	-1   0    0    1   
$EndComp
$Comp
L Device:R R4
U 1 1 60BA3F18
P 2700 2150
F 0 "R4" V 2600 2100 50  0000 L CNN
F 1 "100k" V 2700 2050 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2630 2150 50  0001 C CNN
F 3 "~" H 2700 2150 50  0001 C CNN
F 4 "C17407" H 2700 2150 50  0001 C CNN "LCSC"
	1    2700 2150
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C17
U 1 1 60BA3F1F
P 2500 3300
F 0 "C17" H 2350 3200 50  0000 L CNN
F 1 "10uF" H 2300 3400 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 2538 3150 50  0001 C CNN
F 3 "~" H 2500 3300 50  0001 C CNN
F 4 "C13585" H 2500 3300 50  0001 C CNN "LCSC"
	1    2500 3300
	1    0    0    -1  
$EndComp
$Comp
L Device:C C15
U 1 1 60BA3F26
P 1400 3300
F 0 "C15" H 1150 3200 50  0000 L CNN
F 1 "10uF" H 1200 3400 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 1438 3150 50  0001 C CNN
F 3 "~" H 1400 3300 50  0001 C CNN
F 4 "C13585" H 1400 3300 50  0001 C CNN "LCSC"
	1    1400 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	2500 3150 2250 3150
Wire Wire Line
	2500 3450 1950 3450
Wire Wire Line
	1950 3450 1400 3450
Connection ~ 1950 3450
Wire Wire Line
	1950 3500 1950 3450
Wire Wire Line
	1650 3150 1400 3150
$Comp
L Regulator_Linear:HT75xx-1-SOT89 U7
U 1 1 60BA3F32
P 1950 3250
F 0 "U7" H 1950 3617 50  0000 C CNN
F 1 "HT75xx-1-SOT89" H 1950 3526 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-89-3" H 1950 3575 50  0001 C CIN
F 3 "https://www.holtek.com/documents/10179/116711/HT75xx-1v250.pdf" H 1950 3350 50  0001 C CNN
F 4 "C16106" H 1950 3250 50  0001 C CNN "LCSC"
	1    1950 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 1350 2550 1350
Wire Wire Line
	2850 1650 3050 1650
Connection ~ 2550 1350
$Comp
L power:GND #PWR?
U 1 1 60BA3F3F
P 1700 1750
AR Path="/60BA3F3F" Ref="#PWR?"  Part="1" 
AR Path="/60AD8114/60BA3F3F" Ref="#PWR0151"  Part="1" 
F 0 "#PWR0151" H 1700 1500 50  0001 C CNN
F 1 "GND" H 1705 1577 50  0001 C CNN
F 2 "" H 1700 1750 50  0001 C CNN
F 3 "" H 1700 1750 50  0001 C CNN
	1    1700 1750
	1    0    0    -1  
$EndComp
Connection ~ 2850 1650
$Comp
L Device:R R5
U 1 1 60BA3F5B
P 2850 1500
F 0 "R5" V 2750 1450 50  0000 L CNN
F 1 "56k" V 2850 1450 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 2780 1500 50  0001 C CNN
F 3 "~" H 2850 1500 50  0001 C CNN
F 4 "C17756" V 2850 1500 50  0001 C CNN "LCSC"
	1    2850 1500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0155
U 1 1 60BA3F61
P 2550 2050
F 0 "#PWR0155" H 2550 1800 50  0001 C CNN
F 1 "GND" H 2555 1877 50  0001 C CNN
F 2 "" H 2550 2050 50  0001 C CNN
F 3 "" H 2550 2050 50  0001 C CNN
	1    2550 2050
	0    -1   -1   0   
$EndComp
Connection ~ 1850 2150
$Comp
L pypilot_components:INA180 U?
U 1 1 60BA3F68
P 2200 1850
AR Path="/60BA3F68" Ref="U?"  Part="1" 
AR Path="/60AD8114/60BA3F68" Ref="U8"  Part="1" 
F 0 "U8" H 2200 1713 60  0000 C CNN
F 1 "INA180" H 2200 1819 60  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5_HandSoldering" H 2150 2200 60  0001 C CNN
F 3 "" H 2200 1850 60  0000 C CNN
F 4 "C133969" H 2200 1850 50  0001 C CNN "LCSC"
	1    2200 1850
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0156
U 1 1 60BA3F6E
P 3200 2450
F 0 "#PWR0156" H 3200 2200 50  0001 C CNN
F 1 "GND" H 3205 2277 50  0001 C CNN
F 2 "" H 3200 2450 50  0001 C CNN
F 3 "" H 3200 2450 50  0001 C CNN
	1    3200 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 2150 3400 2150
Wire Wire Line
	2850 2150 3200 2150
Connection ~ 3200 2150
$Comp
L Device:C C?
U 1 1 60BA3F78
P 3200 2300
AR Path="/60BA3F78" Ref="C?"  Part="1" 
AR Path="/60AD8114/60BA3F78" Ref="C18"  Part="1" 
F 0 "C18" H 3050 2200 50  0000 L CNN
F 1 "1uF" H 2950 2300 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3238 2150 50  0001 C CNN
F 3 "~" H 3200 2300 50  0001 C CNN
F 4 "C28323" H 3200 2300 50  0001 C CNN "LCSC"
	1    3200 2300
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 60BA3F7F
P 1450 1600
AR Path="/60BA3F7F" Ref="R?"  Part="1" 
AR Path="/60AD8114/60BA3F7F" Ref="R3"  Part="1" 
F 0 "R3" V 1550 1550 50  0000 L CNN
F 1 ".002" V 1450 1500 50  0000 L CNN
F 2 "Resistor_SMD:R_2512_6332Metric" V 1380 1600 50  0001 C CNN
F 3 "~" H 1450 1600 50  0001 C CNN
F 4 "C76234" V 1450 1600 50  0001 C CNN "LCSC"
	1    1450 1600
	0    1    1    0   
$EndComp
$Comp
L power:VCC #PWR?
U 1 1 60BA3F85
P 2550 1350
AR Path="/60BA3F85" Ref="#PWR?"  Part="1" 
AR Path="/60AD8114/60BA3F85" Ref="#PWR0157"  Part="1" 
F 0 "#PWR0157" H 2550 1200 50  0001 C CNN
F 1 "VCC" H 2567 1523 50  0000 C CNN
F 2 "" H 2550 1350 50  0001 C CNN
F 3 "" H 2550 1350 50  0001 C CNN
	1    2550 1350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0158
U 1 1 60BA3F91
P 1950 3500
F 0 "#PWR0158" H 1950 3250 50  0001 C CNN
F 1 "GND" H 1955 3327 50  0001 C CNN
F 2 "" H 1950 3500 50  0001 C CNN
F 3 "" H 1950 3500 50  0001 C CNN
	1    1950 3500
	1    0    0    -1  
$EndComp
Connection ~ 2500 3150
$Comp
L power:+5V #PWR0159
U 1 1 60BA3F98
P 2500 3150
F 0 "#PWR0159" H 2500 3000 50  0001 C CNN
F 1 "+5V" H 2450 3350 50  0000 C CNN
F 2 "" H 2500 3150 50  0001 C CNN
F 3 "" H 2500 3150 50  0001 C CNN
	1    2500 3150
	1    0    0    -1  
$EndComp
Connection ~ 1400 3150
$Comp
L power:VCC #PWR0160
U 1 1 60BA3F9F
P 1400 3150
F 0 "#PWR0160" H 1400 3000 50  0001 C CNN
F 1 "VCC" H 1400 3300 50  0000 C CNN
F 2 "" H 1400 3150 50  0001 C CNN
F 3 "" H 1400 3150 50  0001 C CNN
	1    1400 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 3450 4850 3450
Wire Wire Line
	4850 3450 4850 4100
Wire Wire Line
	3350 4100 4850 4100
$Comp
L Device:R R17
U 1 1 603E9473
P 5700 5250
F 0 "R17" V 5800 5150 50  0000 L CNN
F 1 "100k" V 5700 5150 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 5630 5250 50  0001 C CNN
F 3 "~" H 5700 5250 50  0001 C CNN
F 4 "C17407" V 5700 5250 50  0001 C CNN "LCSC"
	1    5700 5250
	0    -1   -1   0   
$EndComp
Text GLabel 4600 1850 2    50   Input ~ 0
DRIVE
Text GLabel 5550 5250 0    50   Input ~ 0
DRIVE
$Comp
L Device:R RA?
U 1 1 605F16A4
P 3750 5200
AR Path="/605F16A4" Ref="RA?"  Part="1" 
AR Path="/60AD8114/605F16A4" Ref="R22"  Part="1" 
F 0 "R22" V 3650 5150 50  0000 L CNN
F 1 "470" V 3750 5100 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 3680 5200 50  0001 C CNN
F 3 "~" H 3750 5200 50  0001 C CNN
F 4 "C17710" V 3750 5200 50  0001 C CNN "LCSC"
	1    3750 5200
	0    -1   -1   0   
$EndComp
$Comp
L Connector_Generic:Conn_01x01 Battery+1
U 1 1 606ED5AA
P 1050 1350
F 0 "Battery+1" H 968 1475 50  0000 C CNN
F 1 "Conn_01x01" H 968 1476 50  0001 C CNN
F 2 "footprints:screw_terminal" H 1050 1350 50  0001 C CNN
F 3 "~" H 1050 1350 50  0001 C CNN
	1    1050 1350
	-1   0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 Battery-1
U 1 1 606EDDF0
P 1050 1600
F 0 "Battery-1" H 968 1725 50  0000 C CNN
F 1 "Conn_01x01" H 968 1726 50  0001 C CNN
F 2 "footprints:screw_terminal" H 1050 1600 50  0001 C CNN
F 3 "~" H 1050 1600 50  0001 C CNN
	1    1050 1600
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR0147
U 1 1 6075FBA1
P 3100 1150
F 0 "#PWR0147" H 3100 900 50  0001 C CNN
F 1 "GND" H 3105 977 50  0001 C CNN
F 2 "" H 3100 1150 50  0001 C CNN
F 3 "" H 3100 1150 50  0001 C CNN
	1    3100 1150
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 6076EE7E
P 5800 5750
AR Path="/6076EE7E" Ref="#PWR?"  Part="1" 
AR Path="/60AD8114/6076EE7E" Ref="#PWR0132"  Part="1" 
F 0 "#PWR0132" H 5800 5600 50  0001 C CNN
F 1 "+5V" H 5850 5900 50  0000 C CNN
F 2 "" H 5800 5750 50  0001 C CNN
F 3 "" H 5800 5750 50  0001 C CNN
	1    5800 5750
	1    0    0    -1  
$EndComp
Connection ~ 5800 5750
Text GLabel 4600 1150 2    50   Input ~ 0
ALS
Text GLabel 4600 1250 2    50   Input ~ 0
BLS
Text GLabel 4600 2250 2    50   Input ~ 0
CLS
Text GLabel 4600 2150 2    50   Input ~ 0
CURA
Text GLabel 4600 3050 2    50   Input ~ 0
AHS
Text GLabel 4600 3150 2    50   Input ~ 0
BHS
Text GLabel 4600 3250 2    50   Input ~ 0
CHS
Text GLabel 4600 2350 2    50   Input ~ 0
AL
Text GLabel 4600 2450 2    50   Input ~ 0
BL
Text GLabel 4600 2550 2    50   Input ~ 0
CL
Text GLabel 4600 3350 2    50   Input ~ 0
AH
Text GLabel 4600 3550 2    50   Input ~ 0
BH
Wire Wire Line
	5800 1550 5700 1550
Text GLabel 4600 1350 2    50   Input ~ 0
CH
Wire Wire Line
	3200 4100 3350 4100
Connection ~ 3350 4400
Wire Wire Line
	3200 4400 3350 4400
$Comp
L Device:D_Zener D4
U 1 1 60BA3DAA
P 3200 4250
F 0 "D4" V 3154 4329 50  0000 L CNN
F 1 "3v3" V 3245 4329 50  0000 L CNN
F 2 "Diode_SMD:D_0805_2012Metric" H 3200 4250 50  0001 C CNN
F 3 "~" H 3200 4250 50  0001 C CNN
F 4 "C8056" V 3200 4250 50  0001 C CNN "LCSC"
	1    3200 4250
	0    -1   1    0   
$EndComp
$Comp
L power:GND #PWR0153
U 1 1 60BA3F53
P 3050 1950
F 0 "#PWR0153" H 3050 1700 50  0001 C CNN
F 1 "GND" H 3055 1777 50  0001 C CNN
F 2 "" H 3050 1950 50  0001 C CNN
F 3 "" H 3050 1950 50  0001 C CNN
	1    3050 1950
	1    0    0    -1  
$EndComp
Connection ~ 3050 1650
$Comp
L Device:D_Zener D3
U 1 1 60BA3F4D
P 3050 1800
F 0 "D3" V 2950 1700 50  0000 L CNN
F 1 "3v3" V 3150 1700 50  0000 L CNN
F 2 "Diode_SMD:D_0805_2012Metric" H 3050 1800 50  0001 C CNN
F 3 "~" H 3050 1800 50  0001 C CNN
F 4 "C8056" V 3050 1800 50  0001 C CNN "LCSC"
	1    3050 1800
	0    1    1    0   
$EndComp
$Comp
L Device:LED D8
U 1 1 6152AAA1
P 6450 1600
F 0 "D8" V 6489 1483 50  0000 R CNN
F 1 "GREEN" V 6398 1483 50  0000 R CNN
F 2 "LED_SMD:LED_0805_2012Metric" H 6450 1600 50  0001 C CNN
F 3 "~" H 6450 1600 50  0001 C CNN
F 4 "C2297" V 6450 1600 50  0001 C CNN "LCSC"
	1    6450 1600
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR0112
U 1 1 6152C739
P 6550 2450
F 0 "#PWR0112" H 6550 2300 50  0001 C CNN
F 1 "+5V" H 6565 2623 50  0000 C CNN
F 2 "" H 6550 2450 50  0001 C CNN
F 3 "" H 6550 2450 50  0001 C CNN
	1    6550 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	6450 1450 4600 1450
$Comp
L Device:R R23
U 1 1 615324AF
P 5200 2100
F 0 "R23" H 5250 2100 50  0000 L CNN
F 1 "2.4k" V 5200 2000 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 5130 2100 50  0001 C CNN
F 3 "~" H 5200 2100 50  0001 C CNN
F 4 "C17526" H 5200 2100 50  0001 C CNN "LCSC"
	1    5200 2100
	1    0    0    -1  
$EndComp
$Comp
L Device:R R26
U 1 1 61533213
P 6450 1900
F 0 "R26" V 6350 1900 50  0000 L CNN
F 1 "2.4k" V 6450 1800 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 6380 1900 50  0001 C CNN
F 3 "~" H 6450 1900 50  0001 C CNN
F 4 "C17526" H 6450 1900 50  0001 C CNN "LCSC"
	1    6450 1900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0114
U 1 1 61534E82
P 6450 2050
F 0 "#PWR0114" H 6450 1800 50  0001 C CNN
F 1 "GND" H 6455 1877 50  0001 C CNN
F 2 "" H 6450 2050 50  0001 C CNN
F 3 "" H 6450 2050 50  0001 C CNN
	1    6450 2050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0115
U 1 1 6153510C
P 5200 2250
F 0 "#PWR0115" H 5200 2000 50  0001 C CNN
F 1 "GND" H 5205 2077 50  0001 C CNN
F 2 "" H 5200 2250 50  0001 C CNN
F 3 "" H 5200 2250 50  0001 C CNN
	1    5200 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 1350 2550 1350
Wire Wire Line
	1600 1600 1700 1600
Wire Wire Line
	1700 1600 1700 1750
Wire Wire Line
	1850 1950 1600 1950
Wire Wire Line
	1600 1950 1600 1600
Connection ~ 1600 1600
Wire Wire Line
	2550 1950 2550 1500
Wire Wire Line
	2550 1500 1300 1500
Wire Wire Line
	1300 1500 1300 1600
Connection ~ 1300 1600
$Comp
L Interface_USB:CH340G U1
U 1 1 61595435
P 2750 6450
F 0 "U1" H 2750 6500 50  0000 C CNN
F 1 "CH340G" H 2750 6300 50  0000 C CNN
F 2 "Package_SO:SOIC-16_3.9x9.9mm_P1.27mm" H 2800 5900 50  0001 L CNN
F 3 "http://www.datasheet5.com/pdf-local-2195953" H 2400 7250 50  0001 C CNN
F 4 "C14267" H 2750 6450 50  0001 C CNN "LCSC"
	1    2750 6450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3150 6050 4250 6050
Wire Wire Line
	2750 7050 3700 7050
Wire Wire Line
	3700 7050 3700 6250
$Comp
L Connector:USB_B_Micro J4
U 1 1 615A552D
P 1550 6350
F 0 "J4" H 1607 6817 50  0000 C CNN
F 1 "USB_B_Micro" H 1607 6726 50  0000 C CNN
F 2 "footprints:USB_Mini-B_Tensility_54-00023_Vertical" H 1700 6300 50  0001 C CNN
F 3 "~" H 1700 6300 50  0001 C CNN
	1    1550 6350
	1    0    0    -1  
$EndComp
Wire Wire Line
	1850 6150 1850 5750
Wire Wire Line
	1850 5750 2300 5750
Wire Wire Line
	2750 5750 2750 5850
$Comp
L Device:C C9
U 1 1 615AC3EB
P 2300 5900
F 0 "C9" H 2200 5800 50  0000 L CNN
F 1 ".1uF" H 2100 6000 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 2338 5750 50  0001 C CNN
F 3 "~" H 2300 5900 50  0001 C CNN
F 4 "C49678" H 2300 5900 50  0001 C CNN "LCSC"
	1    2300 5900
	1    0    0    -1  
$EndComp
Connection ~ 2300 5750
Wire Wire Line
	2300 5750 2750 5750
Wire Wire Line
	2350 6350 1850 6350
Wire Wire Line
	1850 6450 2350 6450
Wire Wire Line
	2750 7050 2300 7050
Wire Wire Line
	1550 7050 1550 6750
Connection ~ 2750 7050
$Comp
L Device:Crystal_GND24 Y1
U 1 1 615B8268
P 2050 6850
F 0 "Y1" V 1950 7000 50  0000 L CNN
F 1 "12mhz" V 2050 6750 39  0000 L CNN
F 2 "Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm" H 2050 6850 50  0001 C CNN
F 3 "~" H 2050 6850 50  0001 C CNN
F 4 "C9002" V 2050 6850 50  0001 C CNN "LCSC"
	1    2050 6850
	0    1    1    0   
$EndComp
Wire Wire Line
	2250 6850 2250 7050
Connection ~ 2250 7050
Wire Wire Line
	1850 6850 1850 7050
Connection ~ 1850 7050
Wire Wire Line
	1850 7050 1750 7050
Wire Wire Line
	2050 7000 2350 7000
Wire Wire Line
	2350 7000 2350 6850
Wire Wire Line
	3700 6250 4050 6250
Connection ~ 4050 6250
Wire Wire Line
	4050 6250 4650 6250
$Comp
L Device:C C21
U 1 1 60BA3E36
P 4050 6100
F 0 "C21" H 3950 6000 50  0000 L CNN
F 1 ".1uF" H 3850 6200 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4088 5950 50  0001 C CNN
F 3 "~" H 4050 6100 50  0001 C CNN
F 4 "C49678" H 4050 6100 50  0001 C CNN "LCSC"
	1    4050 6100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 5850 2650 5800
Wire Wire Line
	2650 5800 4050 5800
Connection ~ 4050 5800
Wire Wire Line
	4050 5800 4550 5800
Wire Wire Line
	4050 5800 4050 5950
Wire Wire Line
	4550 5850 4550 5800
Connection ~ 4550 5800
Wire Wire Line
	4550 5800 4700 5800
Wire Wire Line
	4700 5800 4700 6250
$Comp
L Device:R R21
U 1 1 615DFE16
P 4550 6000
F 0 "R21" H 4650 6050 50  0000 L CNN
F 1 "2.4k" V 4550 5900 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 4480 6000 50  0001 C CNN
F 3 "~" H 4550 6000 50  0001 C CNN
F 4 "C17526" H 4550 6000 50  0001 C CNN "LCSC"
	1    4550 6000
	-1   0    0    1   
$EndComp
Wire Wire Line
	3150 6150 4550 6150
Connection ~ 4550 6150
$Comp
L Device:C C7
U 1 1 615E3BA8
P 1750 6900
F 0 "C7" H 1650 6800 50  0000 L CNN
F 1 "22pF" H 1550 7000 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 1788 6750 50  0001 C CNN
F 3 "~" H 1750 6900 50  0001 C CNN
F 4 "C1804" H 1750 6900 50  0001 C CNN "LCSC"
	1    1750 6900
	1    0    0    -1  
$EndComp
Connection ~ 1750 7050
Wire Wire Line
	1750 7050 1550 7050
Wire Wire Line
	1750 6750 1750 6700
Wire Wire Line
	1750 6700 2050 6700
Wire Wire Line
	2350 6650 2350 6700
Wire Wire Line
	2350 6700 2050 6700
Connection ~ 2050 6700
Wire Wire Line
	1850 7050 2250 7050
Wire Wire Line
	2300 6050 2300 7050
Connection ~ 2300 7050
Wire Wire Line
	2300 7050 2250 7050
$Comp
L Device:C C8
U 1 1 615FF15C
P 2050 7150
F 0 "C8" H 1950 7050 50  0000 L CNN
F 1 "22pF" H 2200 7150 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 2088 7000 50  0001 C CNN
F 3 "~" H 2050 7150 50  0001 C CNN
F 4 "C1804" H 2050 7150 50  0001 C CNN "LCSC"
	1    2050 7150
	1    0    0    -1  
$EndComp
Connection ~ 2050 7000
Wire Wire Line
	2050 7300 1750 7300
Wire Wire Line
	1750 7300 1750 7050
Wire Wire Line
	4950 2750 4900 2750
$Comp
L power:+5V #PWR0142
U 1 1 60BA3E94
P 4950 2750
F 0 "#PWR0142" H 4950 2600 50  0001 C CNN
F 1 "+5V" H 4950 2900 50  0000 C CNN
F 2 "" H 4950 2750 50  0001 C CNN
F 3 "" H 4950 2750 50  0001 C CNN
	1    4950 2750
	1    0    0    -1  
$EndComp
$Comp
L Device:R R9
U 1 1 60BA3E8E
P 4750 2750
F 0 "R9" V 4750 2700 50  0000 L CNN
F 1 "10k" V 4700 2500 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 4680 2750 50  0001 C CNN
F 3 "~" H 4750 2750 50  0001 C CNN
F 4 "C17414" V 4750 2750 50  0001 C CNN "LCSC"
	1    4750 2750
	0    -1   -1   0   
$EndComp
Text GLabel 4600 2650 2    50   Input ~ 0
RESET
Wire Wire Line
	3400 1450 3400 2150
$Comp
L Device:R R24
U 1 1 6163ED38
P 5550 2850
F 0 "R24" V 5450 2800 50  0000 L CNN
F 1 "560k" V 5550 2750 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 5480 2850 50  0001 C CNN
F 3 "~" H 5550 2850 50  0001 C CNN
F 4 "C15785" V 5550 2850 50  0001 C CNN "LCSC"
	1    5550 2850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0116
U 1 1 6163FA25
P 5550 3300
F 0 "#PWR0116" H 5550 3050 50  0001 C CNN
F 1 "GND" H 5555 3127 50  0001 C CNN
F 2 "" H 5550 3300 50  0001 C CNN
F 3 "" H 5550 3300 50  0001 C CNN
	1    5550 3300
	1    0    0    -1  
$EndComp
$Comp
L Device:D_Zener D1
U 1 1 6163FFF8
P 5100 3150
F 0 "D1" V 5100 3000 50  0000 L CNN
F 1 "3v3" V 5200 3000 50  0000 L CNN
F 2 "Diode_SMD:D_0805_2012Metric" H 5100 3150 50  0001 C CNN
F 3 "~" H 5100 3150 50  0001 C CNN
F 4 "C8056" V 5100 3150 50  0001 C CNN "LCSC"
	1    5100 3150
	0    1    1    0   
$EndComp
Wire Wire Line
	5100 3000 5550 3000
Text GLabel 6200 2700 1    50   Input ~ 0
PHASEA
$Comp
L Device:C C?
U 1 1 616610B8
P 5900 3150
AR Path="/616610B8" Ref="C?"  Part="1" 
AR Path="/60AD8114/616610B8" Ref="C34"  Part="1" 
F 0 "C34" H 5950 3250 50  0000 L CNN
F 1 "1uF" H 5650 3150 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5938 3000 50  0001 C CNN
F 3 "~" H 5900 3150 50  0001 C CNN
F 4 "C28323" H 5900 3150 50  0001 C CNN "LCSC"
	1    5900 3150
	1    0    0    -1  
$EndComp
$Comp
L Device:R R25
U 1 1 616654B9
P 5550 3150
F 0 "R25" V 5450 3100 50  0000 L CNN
F 1 "100k" V 5550 3050 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" V 5480 3150 50  0001 C CNN
F 3 "~" H 5550 3150 50  0001 C CNN
F 4 "C17407" H 5550 3150 50  0001 C CNN "LCSC"
	1    5550 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 3300 5550 3300
Wire Wire Line
	5100 3000 5100 2050
Wire Wire Line
	5100 2050 4600 2050
$Comp
L Device:C C11
U 1 1 6168FC7C
P 3300 1800
F 0 "C11" H 3150 1700 50  0000 L CNN
F 1 ".1uF" H 3200 2000 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3338 1650 50  0001 C CNN
F 3 "~" H 3300 1800 50  0001 C CNN
F 4 "C49678" H 3300 1800 50  0001 C CNN "LCSC"
	1    3300 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3300 1650 3050 1650
Wire Wire Line
	3300 1950 3050 1950
Connection ~ 3050 1950
Wire Wire Line
	3050 1950 2850 1950
$Comp
L Device:D D7
U 1 1 6169C4BC
P 6050 2700
F 0 "D7" H 6050 2824 50  0000 C CNN
F 1 "D" H 6050 2825 50  0001 C CNN
F 2 "Diode_SMD:D_SOD-123" H 6050 2700 50  0001 C CNN
F 3 "~" H 6050 2700 50  0001 C CNN
F 4 "C64898" H 6050 2700 50  0001 C CNN "LCSC"
	1    6050 2700
	1    0    0    -1  
$EndComp
Connection ~ 5100 3000
Connection ~ 5550 3000
Connection ~ 5550 3300
Wire Wire Line
	5900 3300 5550 3300
Wire Wire Line
	5900 3000 5900 2700
Wire Wire Line
	5900 2700 5550 2700
Connection ~ 5900 2700
$Comp
L Connector_Generic:Conn_01x02 J1
U 1 1 61CF41C9
P 5900 1750
F 0 "J1" H 5980 1742 50  0000 L CNN
F 1 "stop" H 5980 1651 50  0000 L CNN
F 2 "" H 5900 1750 50  0001 C CNN
F 3 "~" H 5900 1750 50  0001 C CNN
	1    5900 1750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0146
U 1 1 61CF4E1E
P 5700 1850
F 0 "#PWR0146" H 5700 1600 50  0001 C CNN
F 1 "GND" H 5705 1677 50  0001 C CNN
F 2 "" H 5700 1850 50  0001 C CNN
F 3 "" H 5700 1850 50  0001 C CNN
	1    5700 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 1750 5700 1550
Connection ~ 5700 1550
Wire Wire Line
	5700 1550 4600 1550
$EndSCHEMATC
