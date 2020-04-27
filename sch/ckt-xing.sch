v 20130925 2
T 55700 36100 9 10 1 0 0 0 1
Simple Grade Crossing Module
T 55600 35800 9 10 1 0 0 0 1
ckt-xing.sch
T 55600 35500 9 10 1 0 0 0 1
1
T 57100 35500 9 10 1 0 0 0 1
1
T 59500 35500 9 10 1 0 0 0 1
Nathan D. Holmes
T 52600 36900 9 10 1 0 0 0 3
Notes:
1) All unpolarized capacitors are ceramic (X7R/X5R) unless otherwise noted.
2) All capacitors and resistors are 0805 unless otherwise noted.
C 34500 56200 1 90 1 Cap_H-2.sym
{
T 34200 56900 5 10 1 1 180 6 1
refdes=C1
T 33000 56200 5 10 0 0 270 2 1
device=Capacitor
T 34600 56700 5 10 1 1 0 8 1
value=68uF
T 34200 56300 5 10 1 1 0 0 1
description=25V
T 34500 56200 5 10 0 0 0 0 1
footprint=cap-elec-Panasonic-FK--D6.30-H5.80-mm
}
C 32200 55600 1 0 1 termblk2-1.sym
{
T 31200 56250 5 10 0 0 0 6 1
device=TERMBLK2
T 31800 56500 5 10 1 1 0 6 1
refdes=J1
T 32200 55600 5 10 0 0 0 0 1
footprint=TERMBLK2_200MIL
}
C 32100 54500 1 0 0 gnd-1.sym
N 32200 55800 32200 54800 4
N 32200 55300 35800 55300 4
N 32900 56200 32200 56200 4
N 33800 56200 35000 56200 4
C 32900 55900 1 0 0 schottky-diode-1.sym
{
T 33300 56500 5 10 0 0 0 0 1
device=DIODE
T 33200 56500 5 10 1 1 0 0 1
refdes=D1
T 32900 55900 5 10 0 0 0 0 1
footprint=SOD123T
T 33300 55900 5 10 1 1 0 5 1
device=CDBM140
}
C 59700 40200 1 0 0 termblk2-1.sym
{
T 60700 40850 5 10 0 0 0 0 1
device=TERMBLK2
T 60100 41100 5 10 1 1 0 0 1
refdes=J12
T 59700 40200 5 10 0 0 0 6 1
footprint=TERMBLK2_200MIL
}
C 60600 50200 1 180 0 qwiic-1.sym
{
T 59600 49550 5 10 0 0 180 0 1
device=QWIIC
T 60300 48350 5 10 1 1 180 3 1
refdes=J9
T 59800 48000 5 10 0 1 180 0 1
footprint=SM04B-SRSS
}
C 60600 52400 1 180 0 qwiic-1.sym
{
T 59600 51750 5 10 0 0 180 0 1
device=QWIIC
T 60300 50550 5 10 1 1 180 3 1
refdes=J8
T 59800 50200 5 10 0 1 180 0 1
footprint=SM04B-SRSS
}
C 60600 54600 1 180 0 qwiic-1.sym
{
T 59600 53950 5 10 0 0 180 0 1
device=QWIIC
T 60300 52750 5 10 1 1 180 3 1
refdes=J7
T 59800 52400 5 10 0 1 180 0 1
footprint=SM04B-SRSS
}
C 60600 56800 1 180 0 qwiic-1.sym
{
T 59600 56150 5 10 0 0 180 0 1
device=QWIIC
T 60300 54950 5 10 1 1 180 3 1
refdes=J6
T 59800 54600 5 10 0 1 180 0 1
footprint=SM04B-SRSS
}
N 59700 55200 59200 55200 4
N 59200 55200 59200 48600 4
C 59100 48300 1 0 0 gnd-1.sym
N 59700 48600 59200 48600 4
N 59700 50800 59200 50800 4
N 59700 53000 59200 53000 4
T 60800 55700 9 10 1 0 0 0 1
EAST APPROACH
T 60800 49100 9 10 1 0 0 0 1
WEST APPROACH
T 60800 51300 9 10 1 0 0 0 1
WEST ISLAND
T 60800 53500 9 10 1 0 0 0 1
EAST ISLAND
C 57000 56800 1 0 0 3.3V-plus-1.sym
N 59700 55600 57200 55600 4
N 57200 49000 57200 56800 4
N 59700 53400 57200 53400 4
N 59700 51200 57200 51200 4
N 59700 49000 57200 49000 4
N 59700 49800 58800 49800 4
N 58800 49800 58800 56400 4
N 58800 56400 59700 56400 4
N 59700 54200 58800 54200 4
N 59700 52000 58800 52000 4
C 57000 49300 1 0 0 res-pack4-1.sym
{
T 57000 49300 5 10 0 0 0 0 1
slot=4
T 57600 49600 5 10 1 1 0 0 1
refdes=R5
T 58000 49600 5 10 1 1 0 0 1
value=2k
T 57000 49300 5 10 0 0 0 0 1
footprint=RPACK4-1206
}
N 57400 49400 57200 49400 4
N 57400 51600 57200 51600 4
C 57000 51500 1 0 0 res-pack4-1.sym
{
T 57000 51500 5 10 0 0 0 0 1
slot=3
T 57600 51800 5 10 1 1 0 0 1
refdes=R5
T 58000 51800 5 10 1 1 0 0 1
value=2k
T 57000 51500 5 10 0 0 0 0 1
footprint=RPACK4-1206
}
N 57400 53800 57200 53800 4
C 57000 53700 1 0 0 res-pack4-1.sym
{
T 57000 53700 5 10 0 0 0 0 1
slot=2
T 57600 54000 5 10 1 1 0 0 1
refdes=R5
T 58000 54000 5 10 1 1 0 0 1
value=2k
T 57000 53700 5 10 0 0 0 0 1
footprint=RPACK4-1206
}
N 57400 56000 57200 56000 4
C 57000 55900 1 0 0 res-pack4-1.sym
{
T 57000 55900 5 10 0 0 0 0 1
slot=1
T 57600 56200 5 10 1 1 0 0 1
refdes=R5
T 58000 56200 5 10 1 1 0 0 1
value=2k
T 57000 55900 5 10 0 0 0 0 1
footprint=RPACK4-1206
}
N 58300 56000 59700 56000 4
N 58300 53800 59700 53800 4
N 58300 51600 59700 51600 4
N 58300 49400 59700 49400 4
N 58500 56000 58500 56700 4
N 58500 56700 57900 56700 4
{
T 57900 56800 5 10 1 1 0 0 1
netname=SDA_1
}
N 58500 54500 57900 54500 4
{
T 57900 54600 5 10 1 1 0 0 1
netname=SDA_2
}
N 58500 53800 58500 54500 4
N 58500 52300 57900 52300 4
{
T 57900 52400 5 10 1 1 0 0 1
netname=SDA_3
}
N 58500 51600 58500 52300 4
N 58500 50100 57900 50100 4
{
T 57900 50200 5 10 1 1 0 0 1
netname=SDA_4
}
N 58500 49400 58500 50100 4
N 48400 53800 47800 53800 4
{
T 48500 53800 5 10 1 1 0 1 1
netname=SDA_1
}
N 48400 53500 47800 53500 4
{
T 48500 53500 5 10 1 1 0 1 1
netname=SDA_2
}
N 48400 53200 47800 53200 4
{
T 48500 53200 5 10 1 1 0 1 1
netname=SDA_3
}
N 48400 52900 47800 52900 4
{
T 48500 52900 5 10 1 1 0 1 1
netname=SDA_4
}
N 48400 52600 47800 52600 4
{
T 48500 52600 5 10 1 1 0 1 1
netname=SCL
}
N 58200 50800 58800 50800 4
{
T 58100 50800 5 10 1 1 0 7 1
netname=SCL
}
C 37000 55200 1 270 1 led-3.sym
{
T 36850 55350 5 10 1 1 0 6 1
device=GREEN
T 36850 55550 5 10 1 1 0 6 1
refdes=D2
T 37000 55200 5 10 0 0 270 0 1
footprint=0805
}
N 37200 55200 37200 55100 4
C 37100 53700 1 0 0 gnd-1.sym
N 37200 54000 37200 54200 4
C 35000 55600 1 0 0 lm7805-1.sym
{
T 36400 56600 5 10 1 1 0 6 1
device=R-78E3.3-0.5
T 36400 56800 5 10 1 1 0 6 1
refdes=U1
T 35000 55600 5 10 0 1 0 0 1
footprint=RECOM-TO220
}
N 35800 55300 35800 55600 4
N 37200 56100 37200 56200 4
N 36600 56200 38000 56200 4
C 32800 50700 1 0 1 termblk2-1.sym
{
T 31800 51350 5 10 0 0 0 6 1
device=TERMBLK2
T 32400 51600 5 10 1 1 0 6 1
refdes=J2
T 32800 50700 5 10 0 0 0 0 1
footprint=TERMBLK2_200MIL
}
C 35500 50700 1 0 1 DMN5L06DMK.sym
{
T 33900 52800 5 10 1 1 0 6 1
refdes=U5
T 34105 52800 5 10 1 1 0 0 1
device=DMN5L06DMK
}
C 35500 48400 1 0 1 DMN5L06DMK.sym
{
T 33900 50500 5 10 1 1 0 6 1
refdes=U4
T 34105 50500 5 10 1 1 0 0 1
device=DMN5L06DMK
}
N 35500 52400 35900 52400 4
N 35900 52400 35900 49300 4
N 35500 51600 35900 51600 4
N 35500 50100 35900 50100 4
N 35500 49300 35900 49300 4
C 35800 49000 1 0 0 gnd-1.sym
N 35500 51200 36300 51200 4
{
T 36400 51200 5 10 1 1 0 1 1
netname=LPWM_RIGHT
}
N 35500 49700 36300 49700 4
{
T 36400 49700 5 10 1 1 0 1 1
netname=LPWM_CONST
}
C 55800 56600 1 90 1 pot-bourns.sym
{
T 54900 55800 5 10 0 0 90 6 1
device=VARIABLE_RESISTOR
T 55800 56600 5 10 0 0 180 2 1
footprint=TC33_trimmer
T 56000 56000 5 10 1 1 0 3 1
refdes=R2
T 55500 56500 5 10 1 1 180 0 1
value=10k
}
C 55500 56700 1 0 0 3.3V-plus-1.sym
N 55700 56600 55700 56700 4
C 55600 55300 1 0 0 gnd-1.sym
N 55700 55600 55700 55700 4
T 55400 54600 9 10 1 0 0 0 3
DEPARTURE
LOCKOUT
TIMEOUT
T 32000 42200 9 10 1 0 0 7 1
FORCE ACTIVE
N 33100 52400 33500 52400 4
N 33500 51600 33500 50900 4
N 33500 50900 32800 50900 4
T 32000 51300 9 10 1 0 0 7 1
LEFT LIGHT
T 32000 50900 9 10 1 0 0 7 1
RIGHT LIGHT
T 32000 49700 9 10 1 0 0 7 1
CONST LIGHT
C 33100 40900 1 0 0 gnd-1.sym
N 36100 42200 37200 42200 4
{
T 37300 42200 5 10 1 1 0 1 1
netname=FORCE_ON
}
C 34000 42000 1 90 0 res-pack4-1.sym
{
T 34000 42000 5 10 0 0 90 0 1
slot=1
T 33700 42600 5 10 1 1 90 0 1
refdes=R4
T 33700 43000 5 10 1 1 90 0 1
value=100k
T 34000 42000 5 10 0 0 0 0 1
footprint=RPACK4-1206
}
C 34600 42000 1 90 0 res-pack4-1.sym
{
T 34600 42000 5 10 0 0 90 0 1
slot=2
T 34300 42600 5 10 1 1 90 0 1
refdes=R4
T 34300 43000 5 10 1 1 90 0 1
value=100k
T 34600 42000 5 10 0 0 0 0 1
footprint=RPACK4-1206
}
N 34500 41800 34500 42400 4
C 29700 35200 0 0 0 title-bordered-A1.sym
C 32900 38300 1 0 1 termblk2-1.sym
{
T 31900 38950 5 10 0 0 0 6 1
device=TERMBLK2
T 32900 38300 5 10 0 0 0 0 1
footprint=TERMBLK2_200MIL
T 32500 39200 5 10 1 1 0 6 1
refdes=J5
}
T 32100 38900 9 10 1 0 0 7 1
WEST APPROACH OCC
T 32100 38500 9 10 1 0 0 7 1
EAST APPROACH OCC
N 36100 38900 37200 38900 4
{
T 37400 38900 5 10 1 1 0 1 1
netname=WEST_OCC
}
N 37200 38500 37500 38500 4
{
T 37700 38500 5 10 1 1 0 1 1
netname=EAST_OCC
}
N 33900 43300 33900 43500 4
N 33900 43500 34500 43500 4
N 34500 43300 34500 43500 4
C 34300 43500 1 0 0 3.3V-plus-1.sym
C 59700 45400 1 0 0 termblk2-1.sym
{
T 60700 46050 5 10 0 0 0 0 1
device=TERMBLK2
T 60100 46300 5 10 1 1 0 0 1
refdes=J10
T 59700 45400 5 10 0 0 0 6 1
footprint=TERMBLK2_200MIL
}
C 59700 44400 1 0 0 termblk2-1.sym
{
T 60700 45050 5 10 0 0 0 0 1
device=TERMBLK2
T 60100 44100 5 10 1 1 0 0 1
refdes=J11
T 59700 44400 5 10 0 0 0 6 1
footprint=TERMBLK2_200MIL
}
C 56400 44200 1 0 0 lv8548mc-1.sym
{
T 58000 44250 5 10 1 1 0 5 1
device=LV8548MC
T 56900 46450 5 10 1 1 0 6 1
refdes=U7
}
N 59700 46000 58400 46000 4
N 58400 45600 59700 45600 4
N 58400 45000 59700 45000 4
N 59700 44600 58400 44600 4
C 57900 47600 1 270 0 capacitor-1.sym
{
T 58600 47400 5 10 0 1 270 0 1
device=CAPACITOR
T 58200 47300 5 10 1 1 0 0 1
refdes=C4
T 58800 47400 5 10 0 0 270 0 1
symversion=0.1
T 58400 46900 5 10 1 1 0 0 1
value=10uF
T 57900 47600 5 10 0 0 0 0 1
footprint=0805
T 58400 46700 5 10 1 1 0 0 1
description=16V
}
C 57300 43800 1 0 0 gnd-1.sym
C 57200 47600 1 0 0 12V-plus-1.sym
C 34600 57000 1 0 0 12V-plus-1.sym
N 34800 57000 34800 56200 4
N 57400 47600 58100 47600 4
C 58000 46400 1 0 0 gnd-1.sym
N 57400 47600 57400 46600 4
C 32600 50300 1 0 0 12V-plus-1.sym
N 32800 50300 32800 50100 4
T 32000 50100 9 10 1 0 0 7 1
SIGNAL COMMON +12
C 32800 49100 1 0 1 termblk3-1.sym
{
T 31800 49750 5 10 0 0 0 6 1
device=HEADER3
T 32400 50400 5 10 1 1 0 6 1
refdes=J3
}
C 32900 41200 1 0 1 termblk3-1.sym
{
T 31900 41850 5 10 0 0 0 6 1
device=HEADER3
T 32500 42500 5 10 1 1 0 6 1
refdes=J4
}
N 32900 41400 33200 41400 4
N 33200 41400 33200 41200 4
N 33100 52400 33100 51300 4
N 33100 51300 32800 51300 4
N 32800 49700 33100 49700 4
N 33100 49700 33100 50100 4
N 33100 50100 33500 50100 4
N 33500 49300 32800 49300 4
T 32000 41400 9 10 1 0 0 7 1
GND
N 53100 46000 56400 46000 4
{
T 52900 46000 5 10 1 1 0 7 1
netname=GATE_A
}
N 53000 45000 56400 45000 4
{
T 52900 45000 5 10 1 1 0 7 1
netname=GATE_B
}
T 60700 45700 9 10 1 0 0 0 1
PRIMARY GATES
T 60700 44700 9 10 1 0 0 0 1
4-QUAD GATES
T 60600 40500 9 10 1 0 0 0 1
SPEAKER (4-8 ohm)
T 32000 49300 9 10 1 0 0 7 1
AUX OUT
C 34500 45500 1 0 0 gnd-1.sym
N 34200 45800 35000 45800 4
C 34300 45400 1 90 0 res-pack4-1.sym
{
T 34300 45400 5 10 0 0 90 0 1
slot=2
T 34000 46000 5 10 1 1 90 0 1
refdes=R1
T 34000 46400 5 10 1 1 90 0 1
value=330
T 34300 45400 5 10 0 0 0 0 1
footprint=RPACK4-1206
}
C 35100 45400 1 90 0 res-pack4-1.sym
{
T 35100 45400 5 10 0 0 90 0 1
slot=3
T 34800 46000 5 10 1 1 90 0 1
refdes=R1
T 34800 46400 5 10 1 1 90 0 1
value=330
T 35100 45400 5 10 0 0 0 0 1
footprint=RPACK4-1206
}
C 37300 53800 1 90 0 res-pack4-1.sym
{
T 37300 53800 5 10 0 0 90 0 1
slot=1
T 37000 54400 5 10 1 1 90 0 1
refdes=R1
T 37000 54800 5 10 1 1 90 0 1
value=330
T 37300 53800 5 10 0 0 0 0 1
footprint=RPACK4-1206
}
C 34000 46700 1 270 1 led-3.sym
{
T 33850 46950 5 10 1 1 0 6 1
device=RED
T 33850 47150 5 10 1 1 0 6 1
refdes=D3
T 34000 46700 5 10 0 0 270 0 1
footprint=0805
}
C 34800 46700 1 270 1 led-3.sym
{
T 35850 46950 5 10 1 1 0 6 1
device=RED
T 35750 47150 5 10 1 1 0 6 1
refdes=D4
T 34800 46700 5 10 0 0 270 0 1
footprint=0805
}
N 34200 47600 34200 48300 4
N 35000 47600 35000 47900 4
C 48800 36800 1 0 0 max98357-wlp.sym
{
T 50300 39150 5 10 1 1 0 0 1
refdes=U3A
T 50200 38100 5 10 1 1 0 4 1
device=MAX98357
T 49100 43800 5 10 0 0 0 0 1
footprint=MAXIM_W91
}
C 56100 38900 1 0 0 max98357-qfn.sym
{
T 57600 41250 5 10 1 1 0 0 1
refdes=U3
T 57500 40200 5 10 1 1 0 4 1
device=MAX98357
T 56400 45900 5 10 0 0 0 0 1
footprint=MAXIM_T1633-4
}
C 43000 38900 1 0 0 SAMD21GxxA-TQFP.sym
{
T 47500 54900 5 10 1 1 0 6 1
refdes=U2
T 43400 55200 5 10 0 0 0 0 1
device=ATmega165P-16AU
T 43400 55400 5 10 0 0 0 0 1
footprint=TQFP64
}
C 37800 56200 1 0 0 3.3V-plus-1.sym
C 44500 56500 1 0 0 3.3V-plus-1.sym
N 44700 56500 44700 55100 4
N 44700 55100 45900 55100 4
C 46300 55300 1 0 0 capacitor-1.sym
{
T 46500 56000 5 10 0 1 0 0 1
device=CAPACITOR
T 46500 55900 5 10 1 1 180 0 1
refdes=C10
T 46500 56200 5 10 0 0 0 0 1
symversion=0.1
T 47300 56000 5 10 1 1 180 0 1
value=1uF
T 46300 55300 5 10 0 0 90 0 1
footprint=0805
T 47300 55800 5 10 1 1 180 0 1
description=16V
}
N 46300 55100 46300 55500 4
C 47100 55200 1 0 0 gnd-1.sym
C 43800 56300 1 270 0 capacitor-1.sym
{
T 44500 56100 5 10 0 1 270 0 1
device=CAPACITOR
T 44100 56000 5 10 1 1 0 0 1
refdes=C7
T 44700 56100 5 10 0 0 270 0 1
symversion=0.1
T 44100 55600 5 10 1 1 0 0 1
value=0.1uF
T 43800 56300 5 10 0 0 0 0 1
footprint=0805
T 44100 55400 5 10 1 1 0 0 1
description=16V
}
C 43100 56300 1 270 0 capacitor-1.sym
{
T 43800 56100 5 10 0 1 270 0 1
device=CAPACITOR
T 43400 56000 5 10 1 1 0 0 1
refdes=C6
T 44000 56100 5 10 0 0 270 0 1
symversion=0.1
T 43400 55600 5 10 1 1 0 0 1
value=0.1uF
T 43100 56300 5 10 0 0 0 0 1
footprint=0805
T 43400 55400 5 10 1 1 0 0 1
description=16V
}
C 42300 56300 1 270 0 capacitor-1.sym
{
T 43000 56100 5 10 0 1 270 0 1
device=CAPACITOR
T 42600 56000 5 10 1 1 0 0 1
refdes=C5
T 43200 56100 5 10 0 0 270 0 1
symversion=0.1
T 42600 55600 5 10 1 1 0 0 1
value=0.1uF
T 42300 56300 5 10 0 0 0 0 1
footprint=0805
T 42600 55400 5 10 1 1 0 0 1
description=16V
}
N 42500 56300 44700 56300 4
N 44000 55100 42500 55100 4
N 44000 55100 44000 55400 4
N 43300 55400 43300 55100 4
N 42500 55400 42500 54900 4
C 42400 54600 1 0 0 gnd-1.sym
C 37800 56200 1 270 0 capacitor-1.sym
{
T 38500 56000 5 10 0 1 270 0 1
device=CAPACITOR
T 38100 55900 5 10 1 1 0 0 1
refdes=C2
T 38700 56000 5 10 0 0 270 0 1
symversion=0.1
T 38100 55500 5 10 1 1 0 0 1
value=10uF
T 37800 56200 5 10 0 0 0 0 1
footprint=0805
T 38100 55300 5 10 1 1 0 0 1
description=6.3V
}
C 37900 55000 1 0 0 gnd-1.sym
C 41900 53500 1 90 0 crystal-1.sym
{
T 41400 53700 5 10 0 0 90 0 1
device=CRYSTAL
T 41900 54700 5 10 1 1 180 0 1
refdes=X1
T 41200 53700 5 10 0 0 90 0 1
symversion=0.1
T 41400 54300 5 10 1 1 0 0 1
value=32.768kHz
}
N 41300 53500 43100 53500 4
N 41300 54200 42200 54200 4
N 42200 54200 42200 53800 4
N 42200 53800 43100 53800 4
C 40400 53300 1 0 0 capacitor-1.sym
{
T 40600 54000 5 10 0 1 0 0 1
device=CAPACITOR
T 40700 53400 5 10 1 1 180 0 1
refdes=C12
T 40600 54200 5 10 0 0 0 0 1
symversion=0.1
T 41400 53400 5 10 1 1 180 0 1
value=18pF
T 40400 53300 5 10 0 0 90 0 1
footprint=0805
T 41300 53200 5 10 1 1 180 0 1
description=16V
}
C 40400 54000 1 0 0 capacitor-1.sym
{
T 40600 54700 5 10 0 1 0 0 1
device=CAPACITOR
T 40700 54100 5 10 1 1 180 0 1
refdes=C11
T 40600 54900 5 10 0 0 0 0 1
symversion=0.1
T 41400 54100 5 10 1 1 180 0 1
value=18pF
T 40400 54000 5 10 0 0 90 0 1
footprint=0805
T 41300 53900 5 10 1 1 180 0 1
description=16V
}
N 40400 54200 39700 54200 4
N 40000 54200 40000 53500 4
N 40400 53500 40000 53500 4
C 39600 53900 1 0 0 gnd-1.sym
N 35500 52000 36300 52000 4
{
T 36400 52000 5 10 1 1 0 1 1
netname=LPWM_LEFT
}
N 35500 48900 36300 48900 4
{
T 36400 48900 5 10 1 1 0 1 1
netname=AUX_OUT
}
N 42500 51700 43100 51700 4
{
T 42400 51700 5 10 1 1 0 7 1
netname=I2S_SD
}
N 42500 50500 43100 50500 4
{
T 42400 50500 5 10 1 1 0 7 1
netname=I2S_BCLK
}
N 42500 50200 43100 50200 4
{
T 42400 50200 5 10 1 1 0 7 1
netname=I2S_LRCLK
}
N 43100 49900 42500 49900 4
{
T 42400 49900 5 10 1 1 0 7 1
netname=LPWM_LEFT
}
N 43100 49300 42500 49300 4
{
T 42400 49300 5 10 1 1 0 7 1
netname=LPWM_RIGHT
}
N 43100 49000 42500 49000 4
{
T 42400 49000 5 10 1 1 0 7 1
netname=LPWM_CONST
}
N 43100 49600 42500 49600 4
{
T 42400 49600 5 10 1 1 0 7 1
netname=AUX_OUT
}
N 35000 47900 35800 47900 4
{
T 35900 47900 5 10 1 1 0 1 1
netname=LPWM_RIGHT
}
N 34200 48300 35000 48300 4
{
T 35100 48300 5 10 1 1 0 1 1
netname=LPWM_LEFT
}
C 39900 44200 1 0 0 swd-jtag-1.sym
{
T 39900 45800 5 10 0 1 0 0 1
device=AVRPROG
T 40500 46300 5 10 1 1 0 0 1
refdes=J13
}
N 47800 49900 48600 49900 4
{
T 48700 49900 5 10 1 1 0 1 1
netname=SWCLK
}
N 47800 49600 48600 49600 4
{
T 48700 49600 5 10 1 1 0 1 1
netname=SWDIO
}
N 41500 46000 41800 46000 4
{
T 41900 46000 5 10 1 1 0 1 1
netname=SWDIO
}
N 41500 45600 41800 45600 4
{
T 41900 45600 5 10 1 1 0 1 1
netname=SWCLK
}
N 43100 46900 43000 46900 4
N 43000 46900 43000 44100 4
N 41500 44400 43000 44400 4
N 39900 44100 39900 45600 4
C 39800 43800 1 0 0 gnd-1.sym
C 39700 46500 1 0 0 3.3V-plus-1.sym
N 39900 46500 39900 46000 4
C 45300 45600 1 0 0 gnd-1.sym
N 44800 46200 44800 45900 4
N 44800 45900 46000 45900 4
N 46000 46200 46000 45900 4
N 45600 46200 45600 45900 4
N 45200 46200 45200 45900 4
N 58300 40800 59700 40800 4
{
T 58800 40900 5 10 1 1 0 0 1
netname=SPK+
}
N 58300 40400 59700 40400 4
{
T 58800 40500 5 10 1 1 0 0 1
netname=SPK-
}
N 58300 40400 58300 40500 4
C 56900 38400 1 0 0 gnd-1.sym
N 57000 39000 57600 39000 4
N 57000 38700 57000 39000 4
C 56800 42700 1 0 0 3.3V-plus-1.sym
N 57000 42700 57000 41500 4
N 57300 41500 57000 41500 4
C 57600 42500 1 270 0 capacitor-1.sym
{
T 58300 42300 5 10 0 1 270 0 1
device=CAPACITOR
T 57900 42200 5 10 1 1 0 0 1
refdes=C3
T 58500 42300 5 10 0 0 270 0 1
symversion=0.1
T 58100 41900 5 10 1 1 0 0 1
value=10uF
T 57600 42500 5 10 0 0 0 0 1
footprint=0805
T 58100 41700 5 10 1 1 0 0 1
description=16V
}
N 57000 42500 58800 42500 4
C 58600 42500 1 270 0 capacitor-1.sym
{
T 59300 42300 5 10 0 1 270 0 1
device=CAPACITOR
T 58900 42200 5 10 1 1 0 0 1
refdes=C6
T 59500 42300 5 10 0 0 270 0 1
symversion=0.1
T 59100 41900 5 10 1 1 0 0 1
value=0.1uF
T 58600 42500 5 10 0 0 0 0 1
footprint=0805
T 59100 41700 5 10 1 1 0 0 1
description=16V
}
N 57800 41600 58800 41600 4
C 58700 41300 1 0 0 gnd-1.sym
N 55500 40500 56100 40500 4
{
T 55400 40500 5 10 1 1 0 7 1
netname=I2S_BCLK
}
N 55500 40200 56100 40200 4
{
T 55400 40200 5 10 1 1 0 7 1
netname=I2S_LRCLK
}
N 55500 39900 56100 39900 4
{
T 55400 39900 5 10 1 1 0 7 1
netname=\_I2S_SHUTDOWN\_
}
N 55500 40800 56100 40800 4
{
T 55400 40800 5 10 1 1 0 7 1
netname=I2S_SD
}
C 49800 39400 1 0 0 3.3V-plus-1.sym
N 48200 38700 48800 38700 4
{
T 48100 38700 5 10 1 1 0 7 1
netname=I2S_SD
}
N 48200 37800 48800 37800 4
{
T 48100 37800 5 10 1 1 0 7 1
netname=\_I2S_SHUTDOWN\_
}
N 48200 38100 48800 38100 4
{
T 48100 38100 5 10 1 1 0 7 1
netname=I2S_LRCLK
}
N 48200 38400 48800 38400 4
{
T 48100 38400 5 10 1 1 0 7 1
netname=I2S_BCLK
}
N 51000 38700 52400 38700 4
{
T 52500 38700 5 10 1 1 0 1 1
netname=SPK+
}
N 51000 38400 52400 38400 4
{
T 52500 38400 5 10 1 1 0 1 1
netname=SPK-
}
C 49900 36800 1 0 0 gnd-1.sym
T 50500 39500 9 10 1 0 0 0 2
Note: U3A is an alternate footprint
for U3 as a WLP.  Only populate one.
N 42500 52000 43100 52000 4
{
T 42400 52000 5 10 1 1 0 7 1
netname=\_I2S_SHUTDOWN\_
}
N 48400 52300 47800 52300 4
{
T 48500 52300 5 10 1 1 0 1 1
netname=GATE_A
}
N 48400 52000 47800 52000 4
{
T 48500 52000 5 10 1 1 0 1 1
netname=GATE_B
}
N 47800 47200 48600 47200 4
{
T 48700 47200 5 10 1 1 0 1 1
netname=WEST_OCC
}
N 47800 46900 48600 46900 4
{
T 48700 46900 5 10 1 1 0 1 1
netname=EAST_OCC
}
C 42800 44100 1 270 0 capacitor-1.sym
{
T 43500 43900 5 10 0 1 270 0 1
device=CAPACITOR
T 43100 43800 5 10 1 1 0 0 1
refdes=C9
T 43700 43900 5 10 0 0 270 0 1
symversion=0.1
T 43300 43500 5 10 1 1 0 0 1
value=0.1uF
T 42800 44100 5 10 0 0 0 0 1
footprint=0805
T 43300 43300 5 10 1 1 0 0 1
description=16V
}
C 42900 42900 1 0 0 gnd-1.sym
N 47800 47500 48600 47500 4
{
T 48700 47500 5 10 1 1 0 1 1
netname=FORCE_ON
}
C 55000 53900 1 90 1 pot-bourns.sym
{
T 54100 53100 5 10 0 0 90 6 1
device=VARIABLE_RESISTOR
T 55000 53900 5 10 0 0 180 2 1
footprint=TC33_trimmer
T 55200 53300 5 10 1 1 0 3 1
refdes=R3
T 54700 53800 5 10 1 1 180 0 1
value=10k
}
C 54700 54000 1 0 0 3.3V-plus-1.sym
N 54900 53900 54900 54000 4
C 54800 52600 1 0 0 gnd-1.sym
N 54900 52900 54900 53000 4
T 55300 53700 9 10 1 0 0 0 1
BELL VOLUME
N 54400 53400 53800 53400 4
{
T 53700 53400 5 10 1 1 0 7 1
netname=BELL_VOL
}
N 43100 53200 42500 53200 4
{
T 42400 53200 5 10 1 1 0 7 1
netname=BELL_VOL
}
N 55200 56100 54600 56100 4
{
T 54500 56100 5 10 1 1 0 7 1
netname=DEP_LOCK
}
N 47800 48400 48600 48400 4
{
T 48700 48400 5 10 1 1 0 1 1
netname=DEP_LOCK
}
C 40300 39900 1 0 0 74hc4049-pwr-1.sym
{
T 40600 40800 5 10 1 1 0 0 1
refdes=U6
T 40500 41350 5 10 0 0 0 0 1
footprint=SO16
T 40800 40500 5 10 1 1 0 0 1
device=74HC4049
}
C 34900 41700 1 0 0 74hc4049-1.sym
{
T 36600 42000 5 10 0 0 0 0 1
device=74HC4049
T 36600 42200 5 10 0 0 0 0 1
footprint=SO16
T 35600 42400 5 10 1 1 0 0 1
refdes=U6
T 34900 41700 5 10 0 0 0 0 1
slot=1
}
N 32900 42200 34900 42200 4
N 33900 42400 33900 42200 4
C 36000 41300 1 0 0 74hc4049-1.sym
{
T 37700 41600 5 10 0 0 0 0 1
device=74HC4049
T 37700 41800 5 10 0 0 0 0 1
footprint=SO16
T 36700 42000 5 10 1 1 0 0 1
refdes=U6
T 36000 41300 5 10 0 0 0 0 1
slot=2
}
N 32900 41800 36000 41800 4
C 34000 38700 1 90 0 res-pack4-1.sym
{
T 34000 38700 5 10 0 0 90 0 1
slot=3
T 33700 39300 5 10 1 1 90 0 1
refdes=R4
T 33700 39700 5 10 1 1 90 0 1
value=100k
T 34000 38700 5 10 0 0 0 0 1
footprint=RPACK4-1206
}
C 34600 38700 1 90 0 res-pack4-1.sym
{
T 34600 38700 5 10 0 0 90 0 1
slot=4
T 34300 39300 5 10 1 1 90 0 1
refdes=R4
T 34300 39700 5 10 1 1 90 0 1
value=100k
T 34600 38700 5 10 0 0 0 0 1
footprint=RPACK4-1206
}
N 34500 38500 34500 39100 4
N 33900 40000 33900 40200 4
N 33900 40200 34500 40200 4
N 34500 40000 34500 40200 4
C 34300 40200 1 0 0 3.3V-plus-1.sym
N 33900 39100 33900 38900 4
C 36000 38000 1 0 0 74hc4049-1.sym
{
T 37700 38300 5 10 0 0 0 0 1
device=74HC4049
T 37700 38500 5 10 0 0 0 0 1
footprint=SO16
T 36700 38700 5 10 1 1 0 0 1
refdes=U6
T 36000 38000 5 10 0 0 0 0 1
slot=4
}
C 34900 38400 1 0 0 74hc4049-1.sym
{
T 35600 39100 5 10 1 1 0 0 1
refdes=U6
T 36600 38700 5 10 0 0 0 0 1
device=74HC4049
T 36600 38900 5 10 0 0 0 0 1
footprint=SO16
T 34900 38400 5 10 0 0 0 0 1
slot=3
}
N 32900 38900 34900 38900 4
N 32900 38500 36000 38500 4
C 53800 45100 1 0 0 74hc4049-1.sym
{
T 55500 45400 5 10 0 0 0 0 1
device=74HC4049
T 55500 45600 5 10 0 0 0 0 1
footprint=SO16
T 54500 45800 5 10 1 1 0 0 1
refdes=U6
T 53800 45100 5 10 0 0 0 0 1
slot=5
}
C 53800 44100 1 0 0 74hc4049-1.sym
{
T 55500 44400 5 10 0 0 0 0 1
device=74HC4049
T 55500 44600 5 10 0 0 0 0 1
footprint=SO16
T 54500 44800 5 10 1 1 0 0 1
refdes=U6
T 53800 44100 5 10 0 0 0 0 1
slot=6
}
N 53400 46000 53400 45600 4
N 53400 45600 53800 45600 4
N 56400 45600 55000 45600 4
N 53400 45000 53400 44600 4
N 53400 44600 53800 44600 4
N 55000 44600 56400 44600 4
C 40300 41400 1 0 0 3.3V-plus-1.sym
C 40400 39100 1 0 0 gnd-1.sym
C 39100 40900 1 270 0 capacitor-1.sym
{
T 39800 40700 5 10 0 1 270 0 1
device=CAPACITOR
T 39400 40600 5 10 1 1 0 0 1
refdes=C8
T 40000 40700 5 10 0 0 270 0 1
symversion=0.1
T 39600 40300 5 10 1 1 0 0 1
value=0.1uF
T 39100 40900 5 10 0 0 0 0 1
footprint=0805
T 39600 40100 5 10 1 1 0 0 1
description=16V
}
N 40500 41400 40500 41000 4
N 40500 39400 40500 39800 4
N 39300 40000 39300 39700 4
N 39300 39700 40500 39700 4
N 39300 40900 39300 41200 4
N 39300 41200 40500 41200 4
T 32000 41800 9 10 1 0 0 7 1
AUX IN
N 37200 41800 37500 41800 4
{
T 37700 41800 5 10 1 1 0 1 1
netname=AUX_IN
}
N 47800 47800 48600 47800 4
{
T 48700 47800 5 10 1 1 0 1 1
netname=AUX_IN
}
