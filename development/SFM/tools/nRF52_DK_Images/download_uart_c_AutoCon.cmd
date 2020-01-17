@echo off
..\usr_bin\nrfjprog.exe --reset --program uart_c_AutoCon/ble_app_uart_c.hex -f nrf52 --chiperase --verify
if not "%ERRORLEVEL%" == "0" (
echo Flash Write Error
pause
)
