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
C 32200 55600 1 0 1 termblk2-1.sym
{
T 31200 56250 5 10 0 0 0 6 1
device=TERMBLK2
T 31800 56500 5 10 1 1 0 6 1
refdes=J1
T 32200 55600 5 10 0 0 0 0 1
footprint=TERMBLK2_200MIL
}
C 32100 53800 1 0 0 gnd-1.sym
N 32200 55800 32200 54100 4
N 32900 56200 32200 56200 4
N 33800 56200 36100 56200 4
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
N 36300 52400 36700 52400 4
N 36700 52400 36700 49300 4
N 36300 51600 36700 51600 4
N 36300 50100 36700 50100 4
N 36300 49300 36700 49300 4
C 36600 49000 1 0 0 gnd-1.sym
N 36300 51200 37100 51200 4
{
T 37200 51200 5 10 1 1 0 1 1
netname=LPWM_RIGHT
}
N 36300 49700 37100 49700 4
{
T 37200 49700 5 10 1 1 0 1 1
netname=LPWM_CONST
}
T 32000 52000 9 10 1 0 0 7 1
LEFT LIGHT
T 32000 51600 9 10 1 0 0 7 1
RIGHT LIGHT
T 32000 49700 9 10 1 0 0 7 1
CONST LIGHT
N 36100 43600 37200 43600 4
{
T 37300 43600 5 10 1 1 0 1 1
netname=ISLAND_OCC
}
C 34000 43400 1 90 0 res-pack4-1.sym
{
T 34000 43400 5 10 0 0 90 0 1
slot=3
T 33700 44000 5 10 1 1 90 0 1
refdes=R2
T 33700 44400 5 10 1 1 90 0 1
value=10k
T 34000 43400 5 10 0 0 0 0 1
footprint=RPACK4-1206
}
C 29700 35200 0 0 0 title-bordered-A1.sym
T 32100 44400 9 10 1 0 0 7 1
WEST APPROACH OCC
T 32100 44000 9 10 1 0 0 7 1
EAST APPROACH OCC
N 36100 45800 37200 45800 4
{
T 37400 45800 5 10 1 1 0 1 1
netname=WEST_OCC
}
N 37200 45400 37500 45400 4
{
T 37700 45400 5 10 1 1 0 1 1
netname=EAST_OCC
}
N 33900 44700 33900 44900 4
C 56400 44200 1 0 0 lv8548mc-1.sym
{
T 58000 44250 5 10 1 1 0 5 1
device=LV8548MC
T 56900 46450 5 10 1 1 0 6 1
refdes=U5
T 56400 44200 5 10 0 0 0 0 1
footprint=LV8548MC
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
refdes=C8
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
C 33900 57000 1 0 0 12V-plus-1.sym
N 34100 57000 34100 56200 4
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
T 32800 49100 5 10 0 0 0 0 1
footprint=TERMBLK3_200MIL
}
N 53100 45600 56400 45600 4
{
T 52900 45600 5 10 1 1 0 7 1
netname=GATE_A
}
N 53000 44600 56400 44600 4
{
T 52900 44600 5 10 1 1 0 7 1
netname=GATE_B
}
T 60600 45800 9 10 1 0 0 0 1
PRIMARY GATES
T 60600 44700 9 10 1 0 0 0 1
4-QUAD GATES
T 32000 49300 9 10 1 0 0 7 1
\_ACTIVE\_
C 44500 41300 1 0 0 gnd-1.sym
N 44200 41600 46500 41600 4
C 44300 41900 1 90 0 res-pack4-1.sym
{
T 44300 41900 5 10 0 0 90 0 1
slot=3
T 44000 42500 5 10 1 1 90 0 1
refdes=R1
T 44000 42900 5 10 1 1 90 0 1
value=330
T 44300 41900 5 10 0 0 0 0 1
footprint=RPACK4-1206
}
C 45100 41900 1 90 0 res-pack4-1.sym
{
T 45100 41900 5 10 0 0 90 0 1
slot=2
T 44800 42500 5 10 1 1 90 0 1
refdes=R1
T 44800 42900 5 10 1 1 90 0 1
value=330
T 45100 41900 5 10 0 0 0 0 1
footprint=RPACK4-1206
}
C 44000 43200 1 270 1 led-3.sym
{
T 43850 43450 5 10 1 1 0 6 1
device=RED
T 43850 43650 5 10 1 1 0 6 1
refdes=D3
T 44000 43200 5 10 0 0 270 0 1
footprint=0805
}
C 44800 43200 1 270 1 led-3.sym
{
T 45850 43450 5 10 1 1 0 6 1
device=RED
T 45750 43650 5 10 1 1 0 6 1
refdes=D4
T 44800 43200 5 10 0 0 270 0 1
footprint=0805
}
N 44200 44100 44200 44800 4
N 45000 44100 45000 44400 4
N 36300 52000 37100 52000 4
{
T 37200 52000 5 10 1 1 0 1 1
netname=LPWM_LEFT
}
N 36300 48900 37100 48900 4
{
T 37200 48900 5 10 1 1 0 1 1
netname=ACT_OUT
}
N 45000 44400 45800 44400 4
{
T 45900 44400 5 10 1 1 0 1 1
netname=LPWM_RIGHT
}
N 44200 44800 45000 44800 4
{
T 45100 44800 5 10 1 1 0 1 1
netname=LPWM_LEFT
}
C 41100 43000 1 0 0 74hc4049-pwr-1.sym
{
T 41400 43900 5 10 1 1 0 0 1
refdes=U6
T 41300 44450 5 10 0 0 0 0 1
footprint=SO16
T 41600 43600 5 10 1 1 0 0 1
device=74HC4049
}
C 34900 43100 1 0 0 74hc4049-1.sym
{
T 36600 43400 5 10 0 0 0 0 1
device=74HC4049
T 36600 43600 5 10 0 0 0 0 1
footprint=SO16
T 35600 43800 5 10 1 1 0 0 1
refdes=U6
T 34900 43100 5 10 0 0 0 0 1
slot=3
}
N 32900 43600 34900 43600 4
N 33900 43800 33900 43600 4
C 34000 45600 1 90 0 res-pack4-1.sym
{
T 34000 45600 5 10 0 0 90 0 1
slot=1
T 33700 46200 5 10 1 1 90 0 1
refdes=R2
T 33700 46600 5 10 1 1 90 0 1
value=10k
T 34000 45600 5 10 0 0 0 0 1
footprint=RPACK4-1206
}
C 34600 45600 1 90 0 res-pack4-1.sym
{
T 34600 45600 5 10 0 0 90 0 1
slot=2
T 34300 46200 5 10 1 1 90 0 1
refdes=R2
T 34300 46600 5 10 1 1 90 0 1
value=10k
T 34600 45600 5 10 0 0 0 0 1
footprint=RPACK4-1206
}
N 34500 45400 34500 46000 4
N 33900 46900 33900 47100 4
N 33900 47100 34500 47100 4
N 34500 46900 34500 47100 4
N 33900 46000 33900 45800 4
C 36000 44900 1 0 0 74hc4049-1.sym
{
T 37700 45200 5 10 0 0 0 0 1
device=74HC4049
T 37700 45400 5 10 0 0 0 0 1
footprint=SO16
T 36700 45600 5 10 1 1 0 0 1
refdes=U6
T 36000 44900 5 10 0 0 0 0 1
slot=2
}
C 34900 45300 1 0 0 74hc4049-1.sym
{
T 35600 46000 5 10 1 1 0 0 1
refdes=U6
T 36600 45600 5 10 0 0 0 0 1
device=74HC4049
T 36600 45800 5 10 0 0 0 0 1
footprint=SO16
T 34900 45300 5 10 0 0 0 0 1
slot=1
}
N 32900 45800 34900 45800 4
N 33300 45400 36000 45400 4
C 41200 42200 1 0 0 gnd-1.sym
C 39900 44000 1 270 0 capacitor-1.sym
{
T 40600 43800 5 10 0 1 270 0 1
device=CAPACITOR
T 40200 43700 5 10 1 1 0 0 1
refdes=C7
T 40800 43800 5 10 0 0 270 0 1
symversion=0.1
T 40400 43400 5 10 1 1 0 0 1
value=0.1uF
T 39900 44000 5 10 0 0 0 0 1
footprint=0805
T 40400 43200 5 10 1 1 0 0 1
description=16V
}
N 41300 44500 41300 44100 4
N 41300 42500 41300 42900 4
N 40100 43100 40100 42800 4
N 40100 42800 41300 42800 4
N 40100 44000 40100 44300 4
N 40100 44300 41300 44300 4
C 30600 36100 1 0 0 hole-1.sym
{
T 30600 36100 5 10 0 1 0 0 1
device=HOLE
T 30600 36100 5 10 0 0 0 0 1
footprint=STANDOFF_HEX_n4
T 30800 36700 5 10 1 1 0 4 1
refdes=H1
}
C 32100 36100 1 0 0 hole-1.sym
{
T 32100 36100 5 10 0 1 0 0 1
device=HOLE
T 32100 36100 5 10 0 0 0 0 1
footprint=STANDOFF_HEX_n4
T 32300 36700 5 10 1 1 0 4 1
refdes=H4
}
C 31600 36100 1 0 0 hole-1.sym
{
T 31600 36100 5 10 0 1 0 0 1
device=HOLE
T 31600 36100 5 10 0 0 0 0 1
footprint=STANDOFF_HEX_n4
T 31800 36700 5 10 1 1 0 4 1
refdes=H3
}
C 31100 36100 1 0 0 hole-1.sym
{
T 31100 36100 5 10 0 1 0 0 1
device=HOLE
T 31100 36100 5 10 0 0 0 0 1
footprint=STANDOFF_HEX_n4
T 31300 36700 5 10 1 1 0 4 1
refdes=H2
}
N 35000 54800 35000 54500 4
N 35000 55700 35000 56200 4
C 40000 56000 1 270 0 capacitor-1.sym
{
T 40700 55800 5 10 0 1 270 0 1
device=CAPACITOR
T 40900 55800 5 10 0 0 270 0 1
symversion=0.1
T 40000 56000 5 10 0 0 0 0 1
footprint=0805
T 40300 55700 5 10 1 1 0 0 1
refdes=C2
T 40300 55400 5 10 1 1 0 2 1
value=22uF
T 40300 55200 5 10 1 1 0 2 1
comment=6.3V
}
N 40200 56000 40200 56200 4
C 34800 55700 1 270 0 Cap_H-2.sym
{
T 36300 55700 5 10 0 0 270 0 1
device=Capacitor
T 34800 55700 5 10 0 1 0 0 1
footprint=cap-elec-Panasonic-FK--D6.30-H5.80-mm
T 35100 55400 5 10 1 1 0 0 1
refdes=C1
T 34300 55400 5 10 1 1 0 2 1
value=68uF
}
C 36100 54800 1 0 0 ap6320x-1.sym
{
T 36400 57450 5 10 0 0 0 0 1
footprint=SOT26
T 39500 55900 5 10 0 1 0 6 1
footprint=SOT26
T 37400 56700 5 10 1 1 0 6 1
device=AP63205
T 37400 56900 5 10 1 1 0 6 1
refdes=U1
}
N 40200 55100 40200 54500 4
C 35400 55700 1 270 0 capacitor-1.sym
{
T 36100 55500 5 10 0 1 270 0 1
device=CAPACITOR
T 36300 55500 5 10 0 0 270 0 1
symversion=0.1
T 35400 55700 5 10 0 0 0 0 1
footprint=0805
T 35700 55400 5 10 1 1 0 0 1
refdes=C4
T 35700 55100 5 10 1 1 0 2 1
value=10uF
}
N 35600 54800 35600 54500 4
N 35600 55700 35600 56200 4
N 36100 55800 36000 55800 4
N 36000 55800 36000 56200 4
N 36900 54800 36900 54500 4
C 38600 56100 1 0 0 inductor-1.sym
{
T 38800 56600 5 10 0 0 0 0 1
device=INDUCTOR
T 38800 56800 5 10 0 0 0 0 1
symversion=0.1
T 38600 56100 5 10 0 0 0 0 1
footprint=MSS6132
T 39050 56350 5 10 1 1 0 3 1
refdes=L1
T 39050 55900 5 10 1 1 0 5 1
model=MSS6132-472
T 39050 56100 5 10 1 1 0 5 1
value=4.7uH
}
N 37700 55400 39800 55400 4
N 39800 55400 39800 56200 4
C 38100 57100 1 270 0 capacitor-1.sym
{
T 38800 56900 5 10 0 1 270 0 1
device=CAPACITOR
T 39000 56900 5 10 0 0 270 0 1
symversion=0.1
T 38100 57100 5 10 0 0 0 0 1
footprint=0805
T 38400 57000 5 10 1 1 0 0 1
refdes=C3
T 38400 56800 5 10 1 1 0 0 1
value=0.1uF
}
N 37700 55800 38300 55800 4
N 38300 55800 38300 56200 4
N 38300 56200 38600 56200 4
N 38300 57100 37900 57100 4
N 37900 57100 37900 56200 4
N 37900 56200 37700 56200 4
C 40700 56000 1 270 0 capacitor-1.sym
{
T 41400 55800 5 10 0 1 270 0 1
device=CAPACITOR
T 41600 55800 5 10 0 0 270 0 1
symversion=0.1
T 40700 56000 5 10 0 0 0 0 1
footprint=0805
T 41000 55700 5 10 1 1 0 0 1
refdes=C2A
T 41000 55400 5 10 1 1 0 2 1
value=22uF
T 41000 55200 5 10 1 1 0 2 1
comment=6.3V
}
N 40900 55100 40900 54500 4
N 40900 56000 40900 56200 4
N 32200 54500 41300 54500 4
N 39500 56200 42600 56200 4
C 42400 56200 1 0 0 5V-plus-1.sym
C 43900 46900 1 0 0 mega328-tqfp32.sym
{
T 44400 46900 5 10 1 1 0 6 1
refdes=U2
T 44200 53700 5 10 0 0 0 0 1
device=ATMega328-TQFP32
T 44200 53900 5 10 0 0 0 0 1
footprint=TQFP32_7
}
C 46200 46200 1 0 0 gnd-1.sym
N 46100 46900 46100 46500 4
N 46100 46500 46500 46500 4
N 46500 46900 46500 46500 4
C 48900 51400 1 0 0 gnd-1.sym
N 48700 51900 49000 51900 4
N 49000 51900 49000 51700 4
C 45900 55100 1 0 0 5V-plus-1.sym
N 46100 55100 46100 53600 4
N 46100 53600 46500 53600 4
C 47200 54800 1 270 0 capacitor-1.sym
{
T 47900 54600 5 10 0 1 270 0 1
device=CAPACITOR
T 48100 54600 5 10 0 0 270 0 1
symversion=0.1
T 47200 54800 5 10 0 0 0 0 1
footprint=0805
T 47000 54500 5 10 1 1 0 0 1
refdes=C5
T 46800 54200 5 10 1 1 0 2 1
value=0.1uF
}
C 48200 53600 1 0 0 gnd-1.sym
C 48100 54800 1 270 0 capacitor-1.sym
{
T 48800 54600 5 10 0 1 270 0 1
device=CAPACITOR
T 49000 54600 5 10 0 0 270 0 1
symversion=0.1
T 48100 54800 5 10 0 0 0 0 1
footprint=0805
T 47900 54500 5 10 1 1 0 0 1
refdes=C6
T 47700 54200 5 10 1 1 0 2 1
value=0.1uF
}
N 47400 53900 48300 53900 4
N 48300 54800 46100 54800 4
C 34300 47100 1 0 0 5V-plus-1.sym
C 41100 44500 1 0 0 5V-plus-1.sym
C 37600 49300 1 0 1 ncv8402-1.sym
{
T 36200 52800 5 10 1 1 0 6 1
device=NCV8402
T 34400 52800 5 10 1 1 0 0 1
refdes=U3
T 37600 49300 5 10 0 0 0 0 1
footprint=SO8
}
C 37600 47000 1 0 1 ncv8402-1.sym
{
T 36200 50500 5 10 1 1 0 6 1
device=NCV8402
T 34400 50500 5 10 1 1 0 0 1
refdes=U4
T 37600 47000 5 10 0 0 0 0 1
footprint=SO8
}
N 33800 48000 33800 49300 4
N 33800 48900 34200 48900 4
N 33800 51600 33800 51200 4
N 33800 51200 34200 51200 4
N 32800 51600 34200 51600 4
N 33100 50100 34200 50100 4
N 33800 50100 33800 49700 4
N 32800 52000 34200 52000 4
N 33800 52400 33800 52000 4
N 34200 52400 33800 52400 4
C 42400 55000 1 270 1 led-3.sym
{
T 43750 55250 5 10 1 1 0 6 1
device=GREEN
T 43350 55450 5 10 1 1 0 6 1
refdes=D2
T 42400 55000 5 10 0 0 270 0 1
footprint=0805
}
N 42600 56200 42600 55900 4
C 40900 54400 1 0 0 res-pack4-1.sym
{
T 40900 54400 5 10 0 0 0 0 1
slot=1
T 41500 54700 5 10 1 1 0 0 1
refdes=R1
T 41900 54700 5 10 1 1 0 0 1
value=330
T 40900 54400 5 10 0 0 270 0 1
footprint=RPACK4-1206
}
N 42600 55000 42600 54500 4
N 42600 54500 42200 54500 4
C 52400 53300 1 0 0 avrprog-1.sym
{
T 52400 54900 5 10 0 1 0 0 1
device=AVRPROG
T 53000 54600 5 10 1 1 0 0 1
refdes=J8
T 52400 53300 5 10 0 0 0 0 1
footprint=JUMPER3x2-SMT
}
C 53900 54800 1 0 0 5V-plus-1.sym
N 54100 54800 54100 54300 4
N 54100 54300 53800 54300 4
C 54100 52900 1 0 0 gnd-1.sym
N 53800 53500 54200 53500 4
N 54200 53500 54200 53200 4
N 53800 53900 54600 53900 4
{
T 54700 53900 5 10 1 1 0 1 1
netname=MOSI
}
N 52400 54300 51600 54300 4
{
T 51500 54300 5 10 1 1 0 7 1
netname=MISO
}
N 43900 51600 43100 51600 4
{
T 43000 51600 5 10 1 1 0 7 1
netname=MISO
}
N 43900 51900 43100 51900 4
{
T 43000 51900 5 10 1 1 0 7 1
netname=MOSI
}
N 52400 53900 51600 53900 4
{
T 51500 53900 5 10 1 1 0 7 1
netname=SCK
}
N 43900 51300 43100 51300 4
{
T 43000 51300 5 10 1 1 0 7 1
netname=SCK
}
C 35600 47500 1 0 0 74hc4049-1.sym
{
T 37300 47800 5 10 0 0 0 0 1
device=74HC4049
T 37300 48000 5 10 0 0 0 0 1
footprint=SO16
T 36300 48200 5 10 1 1 0 0 1
refdes=U6
T 35600 47500 5 10 0 0 0 0 1
slot=4
}
N 36800 48000 37100 48000 4
{
T 37300 48000 5 10 1 1 0 1 1
netname=ACT_IN
}
N 33600 48000 35600 48000 4
C 34000 48100 1 180 0 res-pack4-1.sym
{
T 34000 48100 5 10 0 0 180 0 1
slot=4
T 33400 47800 5 10 1 1 180 0 1
refdes=R2
T 33000 47800 5 10 1 1 180 0 1
value=10k
T 34000 48100 5 10 0 0 90 0 1
footprint=RPACK4-1206
}
N 32300 48000 32300 48200 4
C 32100 48200 1 0 0 5V-plus-1.sym
N 32300 48000 32700 48000 4
C 33700 44900 1 0 0 5V-plus-1.sym
T 32100 43600 9 10 1 0 0 7 1
ISLAND OCC
N 48700 49500 49500 49500 4
{
T 49600 49500 5 10 1 1 0 1 1
netname=ACT_OUT
}
N 48700 49200 49500 49200 4
{
T 49600 49200 5 10 1 1 0 1 1
netname=ACT_IN
}
N 48700 48900 49500 48900 4
{
T 49600 48900 5 10 1 1 0 1 1
netname=WEST_OCC
}
N 48700 48600 49500 48600 4
{
T 49600 48600 5 10 1 1 0 1 1
netname=EAST_OCC
}
N 48700 48300 49500 48300 4
{
T 49600 48300 5 10 1 1 0 1 1
netname=ISLAND_OCC
}
N 43900 52800 43100 52800 4
{
T 43000 52800 5 10 1 1 0 7 1
netname=LPWM_CONST
}
N 52400 53500 51600 53500 4
{
T 51500 53500 5 10 1 1 0 7 1
netname=\_RESET\_
}
N 48700 47700 52100 47700 4
{
T 52200 47700 5 10 1 1 0 1 1
netname=\_RESET\_
}
C 49400 47700 1 270 0 capacitor-1.sym
{
T 50100 47500 5 10 0 1 270 0 1
device=CAPACITOR
T 49700 47400 5 10 1 1 0 0 1
refdes=C12
T 50300 47500 5 10 0 0 270 0 1
symversion=0.1
T 49900 47100 5 10 1 1 0 0 1
value=0.1uF
T 49400 47700 5 10 0 0 0 0 1
footprint=0805
T 49900 46900 5 10 1 1 0 0 1
description=1uF
}
C 49500 46500 1 0 0 gnd-1.sym
C 51900 48000 1 90 0 resistor-1.sym
{
T 51500 48300 5 10 0 0 90 0 1
device=RESISTOR
T 51600 48200 5 10 1 1 90 0 1
refdes=R3
T 51900 48000 5 10 0 0 90 0 1
footprint=0805
T 51600 48500 5 10 1 1 90 0 1
value=10k
}
N 51800 48000 51800 47700 4
C 51600 49200 1 0 0 5V-plus-1.sym
N 51800 49200 51800 48900 4
C 32900 42500 1 0 0 5V-plus-1.sym
N 33100 41700 33100 42500 4
N 33100 42100 32900 42100 4
T 32100 42100 9 10 1 0 0 7 1
SENSOR +5V
C 48800 52800 1 0 0 5V-plus-1.sym
N 49000 52800 49000 52200 4
N 49000 52200 48700 52200 4
C 60200 52600 1 180 0 qwiic-1.sym
{
T 59200 51950 5 10 0 0 180 0 1
device=QWIIC
T 59900 50750 5 10 1 1 180 3 1
refdes=J9
T 59400 50400 5 10 0 1 180 0 1
footprint=SM04B-SRSS
}
C 58900 50400 1 0 0 gnd-1.sym
N 59300 51000 59000 51000 4
N 59000 51000 59000 50700 4
C 57800 52700 1 0 0 5V-plus-1.sym
N 58000 52700 58000 51400 4
N 58000 51400 59300 51400 4
N 59300 51800 59000 51800 4
{
T 58900 51800 5 10 1 1 0 7 1
netname=\_BELL\_
}
C 33100 39800 1 0 0 gnd-1.sym
N 32900 40300 33200 40300 4
N 33200 40100 33200 40700 4
C 32900 40100 1 0 1 termblk2-1.sym
{
T 31900 40750 5 10 0 0 0 6 1
device=TERMBLK2
T 32900 40100 5 10 0 0 0 0 1
footprint=TERMBLK2_200MIL
T 32500 41000 5 10 1 1 0 6 1
refdes=J10
}
T 32100 40300 9 10 1 0 0 7 1
GND
N 32900 40700 33200 40700 4
C 32900 43400 1 0 1 termblk3-1.sym
{
T 31900 44050 5 10 0 0 0 6 1
device=HEADER3
T 32500 44700 5 10 1 1 0 6 1
refdes=J4
T 32900 43400 5 10 0 0 0 0 1
footprint=TERMBLK3_200MIL
}
N 32900 44000 33300 44000 4
N 33300 44000 33300 45400 4
N 32900 45800 32900 44400 4
C 32900 41500 1 0 1 termblk2-1.sym
{
T 31900 42150 5 10 0 0 0 6 1
device=TERMBLK2
T 32900 41500 5 10 0 0 0 0 1
footprint=TERMBLK2_200MIL
T 32500 42400 5 10 1 1 0 6 1
refdes=J5
}
N 32900 41700 33100 41700 4
T 32100 41700 9 10 1 0 0 7 1
SENSOR +5V
T 32100 40700 9 10 1 0 0 7 1
GND
C 32800 52200 1 180 0 termblk2-1.sym
{
T 31800 51550 5 10 0 0 180 0 1
device=TERMBLK2
T 32400 51300 5 10 1 1 180 0 1
refdes=J2
T 32800 52200 5 10 0 0 180 6 1
footprint=TERMBLK2_200MIL
}
N 33800 49700 34200 49700 4
N 32800 49700 33500 49700 4
N 33500 49700 33500 49300 4
N 33500 49300 34200 49300 4
N 33100 50100 33100 49300 4
N 33100 49300 32800 49300 4
C 59700 45200 1 180 1 termblk2-1.sym
{
T 60700 44550 5 10 0 0 180 6 1
device=TERMBLK2
T 60100 44300 5 10 1 1 180 6 1
refdes=J6
T 59700 45200 5 10 0 0 180 0 1
footprint=TERMBLK2_200MIL
}
C 59700 46200 1 180 1 termblk2-1.sym
{
T 60700 45550 5 10 0 0 180 6 1
device=TERMBLK2
T 60100 46400 5 10 1 1 180 6 1
refdes=J7
T 59700 46200 5 10 0 0 180 0 1
footprint=TERMBLK2_200MIL
}
C 46300 42800 1 270 1 led-3.sym
{
T 47650 43050 5 10 1 1 0 6 1
device=AMBER
T 47250 43250 5 10 1 1 0 6 1
refdes=D5
T 46300 42800 5 10 0 0 270 0 1
footprint=0805
}
N 46500 43700 46500 44000 4
N 46500 44000 47300 44000 4
{
T 47400 44000 5 10 1 1 0 1 1
netname=ACT_OUT
}
N 44200 41600 44200 42300 4
N 45000 42300 45000 41600 4
C 46600 41400 1 90 0 res-pack4-1.sym
{
T 46600 41400 5 10 0 0 90 0 1
slot=4
T 46300 42000 5 10 1 1 90 0 1
refdes=R1
T 46300 42400 5 10 1 1 90 0 1
value=330
T 46600 41400 5 10 0 0 0 0 1
footprint=RPACK4-1206
}
N 46500 41600 46500 41800 4
N 46500 42800 46500 42700 4
N 56400 45000 55000 45000 4
C 53800 44500 1 0 0 74hc4049-1.sym
{
T 55500 44800 5 10 0 0 0 0 1
device=74HC4049
T 55500 45000 5 10 0 0 0 0 1
footprint=SO16
T 54500 45200 5 10 1 1 0 0 1
refdes=U6
T 53800 44500 5 10 0 0 0 0 1
slot=5
}
N 53400 45000 53800 45000 4
N 55000 46000 56400 46000 4
C 53800 45500 1 0 0 74hc4049-1.sym
{
T 55500 45800 5 10 0 0 0 0 1
device=74HC4049
T 55500 46000 5 10 0 0 0 0 1
footprint=SO16
T 54500 46200 5 10 1 1 0 0 1
refdes=U6
T 53800 45500 5 10 0 0 0 0 1
slot=6
}
N 53400 46000 53800 46000 4
N 53400 46000 53400 45600 4
N 53400 45000 53400 44600 4
N 43900 52200 43100 52200 4
{
T 43000 52200 5 10 1 1 0 7 1
netname=LPWM_LEFT
}
N 43900 52500 43100 52500 4
{
T 43000 52500 5 10 1 1 0 7 1
netname=LPWM_RIGHT
}
C 39000 47100 1 0 0 gnd-1.sym
N 39500 49800 39100 49800 4
N 39100 49800 39100 47400 4
N 39500 47700 39100 47700 4
N 39500 48000 39100 48000 4
N 39500 48300 39100 48300 4
N 39500 48600 39100 48600 4
N 39500 48900 39100 48900 4
N 39500 49200 39100 49200 4
N 39500 49500 39100 49500 4
C 39500 50200 1 180 1 switch-dip8-1.sym
{
T 40900 47625 5 8 0 0 180 6 1
device=219-8MST
T 39500 50200 5 10 0 0 180 6 1
footprint=DIPSW16
T 39800 47450 5 10 1 1 180 6 1
refdes=SW1
}
N 40800 49800 41100 49800 4
{
T 41200 49800 5 10 1 1 0 1 1
netname=SW8
}
N 40800 49500 41100 49500 4
{
T 41200 49500 5 10 1 1 0 1 1
netname=SW7
}
N 40800 49200 41100 49200 4
{
T 41200 49200 5 10 1 1 0 1 1
netname=SW6
}
N 40800 48900 41100 48900 4
{
T 41200 48900 5 10 1 1 0 1 1
netname=SW5
}
N 40800 48600 41100 48600 4
{
T 41200 48600 5 10 1 1 0 1 1
netname=SW4
}
N 40800 48300 41100 48300 4
{
T 41200 48300 5 10 1 1 0 1 1
netname=SW3
}
N 40800 48000 41100 48000 4
{
T 41200 48000 5 10 1 1 0 1 1
netname=SW2
}
N 40800 47700 41100 47700 4
{
T 41200 47700 5 10 1 1 0 1 1
netname=SW1
}
N 48700 48000 49000 48000 4
{
T 49100 48000 5 10 1 1 0 1 1
netname=SW4
}
N 43900 48600 43600 48600 4
{
T 43500 48600 5 10 1 1 0 7 1
netname=\_BELL\_
}
N 43900 48900 43600 48900 4
{
T 43500 48900 5 10 1 1 0 7 1
netname=SW8
}
N 43900 49200 43600 49200 4
{
T 43500 49200 5 10 1 1 0 7 1
netname=SW7
}
N 43900 49500 43600 49500 4
{
T 43500 49500 5 10 1 1 0 7 1
netname=SW6
}
N 43900 49800 43600 49800 4
{
T 43500 49800 5 10 1 1 0 7 1
netname=SW5
}
N 43900 50700 43100 50700 4
{
T 43000 50700 5 10 1 1 0 7 1
netname=GATE_A
}
N 43900 51000 43100 51000 4
{
T 43000 51000 5 10 1 1 0 7 1
netname=GATE_B
}
N 43900 48300 43600 48300 4
{
T 43500 48300 5 10 1 1 0 7 1
netname=SW1
}
N 43900 48000 43600 48000 4
{
T 43500 48000 5 10 1 1 0 7 1
netname=SW2
}
N 43900 47700 43600 47700 4
{
T 43500 47700 5 10 1 1 0 7 1
netname=SW3
}
