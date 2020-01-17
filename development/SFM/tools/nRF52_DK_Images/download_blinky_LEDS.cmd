@echo off
..\usr_bin\nrfjprog.exe --reset --program blinky_LEDS/blinky_pca10040_s132.hex -f nrf52 --chiperase --verify
if not "%ERRORLEVEL%" == "0" (
echo Flash Write Error
pause
)
