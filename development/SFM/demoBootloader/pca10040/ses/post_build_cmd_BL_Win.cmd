@echo off
set SFM_DEFAULT_TARGET_DIR=./Output/Release/Exe
set SFM_DEFAULT_TARGET_NAME=SFM_demoBL_s132_pca10040
set SFM_TARGET_CONFIG_FILE_NAME=..\..\..\common\config\cfg_config_defines.h
set SFM_CP_TOOLS=..\..\..\tools\usr_bin\cp.exe
set SFM_RM_TOOLS=..\..\..\tools\usr_bin\rm.exe

set SFM_TARGET_DIR=""
set SFM_TARGET_NAME=""

if not "%1"=="" (
    set SFM_TARGET_DIR=%1
) else (
    set SFM_TARGET_DIR=%SFM_DEFAULT_TARGET_DIR%
)

if not "%2"=="" (
    set SFM_TARGET_NAME=%2
) else (
    set SFM_TARGET_NAME=%SFM_DEFAULT_TARGET_NAME%
)

echo Searching... Board Type Defines.
set cmd="findstr REPLACE_DEVICE_DEFINE_HERE %SFM_TARGET_CONFIG_FILE_NAME%"
FOR /F "delims=" %%i IN (' %cmd% ') DO SET CFG_LINE_INPUT=%%i
echo CFG_LINE_INPUT : %CFG_LINE_INPUT%
set CDEV_BOARD_TYPE_DEFINE=%CFG_LINE_INPUT:~49%
set DEVICE_TYPE=Default
if "%CDEV_BOARD_TYPE_DEFINE:~0,14%" == "CDEV_BOARD_EVB" set DEVICE_TYPE=EVB
if "%CDEV_BOARD_TYPE_DEFINE:~0,16%" == "CDEV_BOARD_IHERE" set DEVICE_TYPE=iHere
echo CDEV_BOARD_TYPE_DEFINE in config file : %CDEV_BOARD_TYPE_DEFINE%
echo board type string : %DEVICE_TYPE%

%SFM_CP_TOOLS% -f %SFM_TARGET_DIR%/%SFM_TARGET_NAME%.hex ../../hex/sfm_bootloader_%DEVICE_TYPE%.hex
if not "%ERRORLEVEL%" == "0" (
	echo file copy Error!
    goto QUIT
)

echo Important memory symbol
%SFM_CP_TOOLS% -f %SFM_TARGET_DIR%/%SFM_TARGET_NAME%.map tmp_%SFM_TARGET_NAME%.map

set cmd="findstr __text_size__ tmp_%SFM_TARGET_NAME%.map"
FOR /F "delims=" %%i IN (' %cmd% ') DO SET CFG_LINE_INPUT=%%i
echo %CFG_LINE_INPUT%

set cmd="findstr __rodata_size__ tmp_%SFM_TARGET_NAME%.map"
FOR /F "delims=" %%i IN (' %cmd% ') DO SET CFG_LINE_INPUT=%%i
echo %CFG_LINE_INPUT%

set cmd="findstr __data_size__ tmp_%SFM_TARGET_NAME%.map"
FOR /F "delims=" %%i IN (' %cmd% ') DO SET CFG_LINE_INPUT=%%i
echo %CFG_LINE_INPUT%

set cmd="findstr __bss_size__ tmp_%SFM_TARGET_NAME%.map"
FOR /F "delims=" %%i IN (' %cmd% ') DO SET CFG_LINE_INPUT=%%i
echo %CFG_LINE_INPUT%

%SFM_RM_TOOLS% -f tmp_%SFM_TARGET_NAME%.map

:QUIT

