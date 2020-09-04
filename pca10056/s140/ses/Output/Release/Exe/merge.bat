cls
@ECHO OFF
setlocal EnableDelayedExpansion

ECHO Long-Range without GPS = 0
ECHO Long-Range with GPS = 1
ECHO Ultra Long-Range without GPS = 2
ECHO Ultra Long-Range with GPS = 3

set type=
set /p type=Enter a value for the type of tag:  

if %type%==0 (mergehex -m s140_nrf52_7.2.0_softdevice.hex ble_long_range_beacon.hex -o ble_long_range_beacon_without_gps.hex) 
if %type%==1 (mergehex -m s140_nrf52_7.2.0_softdevice.hex ble_long_range_beacon.hex -o ble_long_range_beacon_with_gps.hex) 
if %type%==2 (mergehex -m s140_nrf52_7.2.0_softdevice.hex ble_long_range_beacon.hex -o ble_ultra_long_range_beacon_without_gps.hex) 
if %type%==3 (mergehex -m s140_nrf52_7.2.0_softdevice.hex ble_long_range_beacon.hex -o ble_ultra_long_range_beacon_with_gps.hex) 
pause