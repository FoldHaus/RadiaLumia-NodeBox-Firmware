@echo off

title FoldHaus Mass programmer

set FIRMWARE=.pioenvs\pro16MHzatmega328\firmware.hex
set ROOT=%~dp0..

set SETUPHEX=%ROOT%\\%FIRMWARE:~0,-3%setup.hex
set MAINHEX=%ROOT%\\%FIRMWARE:~0,-3%main.hex

SET PLATFORMIO_BUILD_FLAGS='-DFirstRunAndPinSpotTest=true'
platformio.exe run --silent
move %ROOT%/%FIRMWARE% %SETUPHEX%

SET PLATFORMIO_BUILD_FLAGS='-DFirstRunAndPinSpotTest=false'
platformio.exe run --silent
move %ROOT%/%FIRMWARE% %MAINHEX%

fc /b %MAINHEX% %SETUPHEX% > nul
if errorlevel 1 (
    echo Let's get this programming party started!
) else (
    echo Built files are the same. This is wrong
    exit
)

SET PROG="avrdude" -pm328p -carduino -PCOM8 -b57600

:StartLoop

%PROG% -lnul || goto StartLoop

echo Programming new device with setup firmware and PinSpot test

%PROG% -e -Uflash:w:"%SETUPHEX%":a || goto StartLoop

REM Sleep for 10 seconds to test LED.
REM Use timeout so that any key press will start next step in case of failure.
timeout 10 > nul

echo Programming main run firmware
%PROG% -D -U flash:w:"%MAINHEX%":a || goto StartLoop


echo Done. Waiting for device to be unplugged.

sleep 4

:DoneLoop
%PROG% -lnul && goto DoneLoop

echo Device unplugged. Waiting for next device.

sleep 1

goto StartLoop