@echo off
setlocal

echo Changing to avrdude directory...
cd %appdata%

cd ..\Local\Arduino15\packages\arduino\tools\avrdude\6.3.0-arduino17\bin


:: Prompt the user for the COM port number
set /p COMPORT=Enter COM port number (e.g., 3 for COM3): 

:: Construct the full COM port name
set COMNAME=com%COMPORT%

:: Display the selected COM port
echo Writing fuses to %COMNAME%...
avrdude.exe -p m328p -b 19200 -c avrisp -C ..\etc\avrdude.conf -P %COMNAME% -U lfuse:w:0xFF:m -U hfuse:w:0xD6:m -U efuse:w:0xFD:m 


echo.
echo.
echo Verifying fuses on %COMNAME%...
avrdude.exe -p m328p -b 19200 -c avrisp -C ..\etc\avrdude.conf -P %COMNAME% -U lfuse:r:lowfuse:h -U hfuse:r:highfuse:h -U efuse:r:exfuse:h
echo Fuse values SHOULD be: 0xff, 0xd6, 0xfd.
echo.
echo Fuse values read from the chip:
type lowfuse
type highfuse
type exfuse

echo.
echo.
echo Cleaning up temporary files...
del lowfuse highfuse exfuse



:: Pause to keep the window open
pause
