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

echo Press any key when connected
timeout -1 > nul

echo Programming test & setup firmware

%PROG% -e -Uflash:w:"%SETUPHEX%":a || goto ErrorTryAgain

REM Sleep for 10 seconds to test LED.
REM Use timeout so that any key press will start next step in case of failure.
timeout 10 > nul

echo Programming main firmware
%PROG% -D -U flash:w:"%MAINHEX%":a || goto ErrorTryAgain


echo Done.

sleep 4

goto StartLoop

:ErrorTryAgain

echo Error. Please try again.

goto StartLoop
