@echo off

set SFM_DEFAULT_TARGET_DIR=./Output/Release/Exe
set SFM_DEFAULT_TARGET_NAME=SFM_demoApp_pca10040_s132
set SFM_TARGET_CONFIG_FILE_NAME=..\..\..\..\common\config\cfg_config_defines.h
set SFM_NRFUTIL_TOOLS=..\..\..\..\tools\SFM_PC-nrfutil\bin\nrfutil_wisol\nrfutil_wisol.exe
set SFM_HEXMERGE_TOOLS=..\..\..\..\tools\usr_bin\mergehex.exe
set SFM_CP_TOOLS=..\..\..\..\tools\usr_bin\cp.exe
set SFM_RM_TOOLS=..\..\..\..\tools\usr_bin\rm.exe
set SFM_MKDIR_TOOLS=..\..\..\..\tools\usr_bin\mkdir.exe
set SFM_MV_TOOLS=..\..\..\..\tools\usr_bin\mv.exe
set SFM_ECHO_TOOLS=..\..\..\..\tools\usr_bin\echo.exe
set SFM_SOFTDEVICE_CODE=0xB7
set SFM_BL_SETTING_VER=2

set SFM_TARGET_DIR=""
set SFM_TARGET_NAME=""
set SFM_APP_VER=""
set SFM_BL_VER=""

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

copy /y ..\..\..\..\..\..\components\softdevice\s132\hex\s132_nrf52_6.1.1_softdevice.hex .\Output\%SFM_TARGET_NAME%_SD.hex
set SFM_INOUT_SOFT_DEVICE_HEX_FILE_NAME=./Output/%SFM_TARGET_NAME%_SD.hex
set SFM_INOUT_BL_FILE_NAME=""
set SFM_INOUT_SEC_KEY_FILE_NAME=""
set SFM_OUTPUT_APP_FILE_NAME=./Output/%SFM_TARGET_NAME%_APP.hex
set SFM_OUTPUT_BL_FILE_NAME=./Output/%SFM_TARGET_NAME%_BL.hex
set SFM_OUTPUT_SETTING_FILE_NAME=./Output/%SFM_TARGET_NAME%_Setting.hex
set SFM_OUTPUT_SD_SETTING_FILE_NAME=./Output/%SFM_TARGET_NAME%_mreged_SD_Setting.hex
set SFM_OUTPUT_APP_SETTING_FILE_NAME=./Output/%SFM_TARGET_NAME%_mreged_APP_Setting.hex
set SFM_OUTPUT_SD_BL_FILE_NAME=./Output/%SFM_TARGET_NAME%_mreged_SD_BL.hex
set SFM_OUTPUT_MERGED_ALL_FILE_NAME=./Output/%SFM_TARGET_NAME%_mreged_ALL.hex
set SFM_OUTPUT_DFU_PKG_FILE_NAME=./Output/%SFM_TARGET_NAME%_DFU_PKG.zip
set SFM_OUTPUT_TARGET_BINARY_TOP_PATH=../../../../binary
set SFM_OUTPUT_TARGET_BINARY_PREFIX_NAME=""

%SFM_CP_TOOLS% -f %SFM_TARGET_DIR%/%SFM_TARGET_NAME%.hex %SFM_OUTPUT_APP_FILE_NAME%
if not "%ERRORLEVEL%" == "0" (
	echo APP Hex file copy Error!
    goto QUIT
)

echo Searching... APP SW version.
set cmd="findstr CDEV_SW_VER_MAJOR %SFM_TARGET_CONFIG_FILE_NAME%"
FOR /F "delims=" %%i IN (' %cmd% ') DO SET CFG_LINE_INPUT=%%i
set SW_VER_MAJOR=%CFG_LINE_INPUT:~27,1%
set cmd="findstr CDEV_SW_VER_MINOR %SFM_TARGET_CONFIG_FILE_NAME%"
FOR /F "delims=" %%i IN (' %cmd% ') DO SET CFG_LINE_INPUT=%%i
set SW_VER_MINOR=%CFG_LINE_INPUT:~27,2%
set SFM_APP_VER=%SW_VER_MAJOR%%SW_VER_MINOR%
echo APP version : %SFM_APP_VER%

echo Searching... BL SW version.
set cmd="findstr CDEV_BL_VER %SFM_TARGET_CONFIG_FILE_NAME%"
FOR /F "delims=" %%i IN (' %cmd% ') DO SET CFG_LINE_INPUT=%%i
set SFM_BL_VER=%CFG_LINE_INPUT:~21,1%
echo BL version : %SFM_BL_VER%

echo Searching... Model Name.
set SFM_MODEL_NAME=Unknown
set cmd="findstr CDEV_MODEL_NAME %SFM_TARGET_CONFIG_FILE_NAME%"
FOR /F "delims=" %%i IN (' %cmd% ') DO SET CFG_LINE_INPUT=%%i
set SFM_MODEL_NAME=%CFG_LINE_INPUT:~25,6%
echo MODEL NAME : %SFM_MODEL_NAME%

echo Searching... Board Type Defines.
set cmd="findstr REPLACE_DEVICE_DEFINE_HERE %SFM_TARGET_CONFIG_FILE_NAME%"
FOR /F "delims=" %%i IN (' %cmd% ') DO SET CFG_LINE_INPUT=%%i
echo CFG_LINE_INPUT : %CFG_LINE_INPUT%
set CDEV_BOARD_TYPE_DEFINE=%CFG_LINE_INPUT:~49%
set SFM_DEVICE_TYPE=Default
if "%CDEV_BOARD_TYPE_DEFINE:~0,14%" == "CDEV_BOARD_EVB" set SFM_DEVICE_TYPE=EVB
if "%CDEV_BOARD_TYPE_DEFINE:~0,16%" == "CDEV_BOARD_IHERE" set SFM_DEVICE_TYPE=iHere
echo CDEV_BOARD_TYPE_DEFINE in config file : %CDEV_BOARD_TYPE_DEFINE%
echo board type string : %SFM_DEVICE_TYPE%

set SFM_INOUT_BL_FILE_NAME=../../../../demoBootloader/hex/sfm_bootloader_%SFM_DEVICE_TYPE%.hex
echo input bootloader file name : %SFM_INOUT_BL_FILE_NAME%
set SFM_INOUT_SEC_KEY_FILE_NAME=../../../../demoBootloader/keys/%SFM_DEVICE_TYPE%_private.pem
echo input secure key file name : %SFM_INOUT_SEC_KEY_FILE_NAME%

%SFM_CP_TOOLS% -f %SFM_INOUT_BL_FILE_NAME% %SFM_OUTPUT_BL_FILE_NAME%
if not "%ERRORLEVEL%" == "0" (
	echo Bootloader Hex file copy Error!
    goto QUIT
)

echo make setting file : For more information, please visit https://github.com/NordicSemiconductor/pc-nrfutil
echo Setting : %SFM_OUTPUT_SETTING_FILE_NAME%
%SFM_NRFUTIL_TOOLS% settings generate --family NRF52 --application %SFM_OUTPUT_APP_FILE_NAME% --application-version %SFM_APP_VER% --bootloader-version %SFM_BL_VER% --bl-settings-version %SFM_BL_SETTING_VER% %SFM_OUTPUT_SETTING_FILE_NAME%
echo Setting file name : %SFM_OUTPUT_SETTING_FILE_NAME%

echo make merged file [softdevice+setting]
%SFM_HEXMERGE_TOOLS% -m %SFM_INOUT_SOFT_DEVICE_HEX_FILE_NAME% %SFM_OUTPUT_SETTING_FILE_NAME% -o %SFM_OUTPUT_SD_SETTING_FILE_NAME%
echo SD+Seting file name : %SFM_OUTPUT_SD_SETTING_FILE_NAME%

echo make merged file [app+setting]
%SFM_HEXMERGE_TOOLS% -m %SFM_OUTPUT_APP_FILE_NAME% %SFM_OUTPUT_SETTING_FILE_NAME% -o %SFM_OUTPUT_APP_SETTING_FILE_NAME%
echo APP+Seting file name : %SFM_OUTPUT_APP_SETTING_FILE_NAME%

echo make merged file [softdevice+bootloader]
%SFM_HEXMERGE_TOOLS% -m %SFM_INOUT_SOFT_DEVICE_HEX_FILE_NAME% %SFM_OUTPUT_BL_FILE_NAME% -o %SFM_OUTPUT_SD_BL_FILE_NAME%
echo SD+BL file name : %SFM_OUTPUT_SD_BL_FILE_NAME%

echo make merged file [ALL]
%SFM_HEXMERGE_TOOLS% -m %SFM_OUTPUT_APP_SETTING_FILE_NAME% %SFM_OUTPUT_SD_BL_FILE_NAME% -o %SFM_OUTPUT_MERGED_ALL_FILE_NAME%
echo Merged ALL file name : %SFM_OUTPUT_MERGED_ALL_FILE_NAME%

echo make DFU Package file : For more information, please visit https://github.com/NordicSemiconductor/pc-nrfutil
%SFM_NRFUTIL_TOOLS% pkg generate --hw-version 52 --sd-req %SFM_SOFTDEVICE_CODE% --application-version %SFM_APP_VER% --application %SFM_OUTPUT_APP_FILE_NAME% --key-file %SFM_INOUT_SEC_KEY_FILE_NAME% %SFM_OUTPUT_DFU_PKG_FILE_NAME%
echo Package file name : %SFM_OUTPUT_DFU_PKG_FILE_NAME%

set SFM_OUTPUT_TARGET_BINARY_PREFIX_NAME=%SFM_MODEL_NAME%_%SFM_DEVICE_TYPE%_AV%SFM_APP_VER%_BV%SFM_BL_VER%
echo copy target binary [prefix : %SFM_OUTPUT_TARGET_BINARY_PREFIX_NAME%]

echo make output dir
%SFM_RM_TOOLS% -rf %SFM_OUTPUT_TARGET_BINARY_TOP_PATH%/bin/
%SFM_RM_TOOLS% -rf %SFM_OUTPUT_TARGET_BINARY_TOP_PATH%/output_files/
%SFM_RM_TOOLS% -rf %SFM_OUTPUT_TARGET_BINARY_TOP_PATH%/DFU_packages/
%SFM_RM_TOOLS% -rf %SFM_OUTPUT_TARGET_BINARY_TOP_PATH%/factory_download_images/
%SFM_RM_TOOLS% -rf %SFM_OUTPUT_TARGET_BINARY_TOP_PATH%/download_APP.cmd
%SFM_RM_TOOLS% -rf %SFM_OUTPUT_TARGET_BINARY_TOP_PATH%/download_Factory.cmd
%SFM_MKDIR_TOOLS%  %SFM_OUTPUT_TARGET_BINARY_TOP_PATH%/bin/
%SFM_MKDIR_TOOLS%  %SFM_OUTPUT_TARGET_BINARY_TOP_PATH%/output_files/
%SFM_MKDIR_TOOLS%  %SFM_OUTPUT_TARGET_BINARY_TOP_PATH%/DFU_packages/
%SFM_MKDIR_TOOLS%  %SFM_OUTPUT_TARGET_BINARY_TOP_PATH%/factory_download_images/

echo copy output binaries
%SFM_CP_TOOLS% -f %SFM_TARGET_DIR%/%SFM_TARGET_NAME%.hex %SFM_OUTPUT_TARGET_BINARY_TOP_PATH%/output_files/%SFM_OUTPUT_TARGET_BINARY_PREFIX_NAME%_APP.hex
%SFM_CP_TOOLS% -f %SFM_TARGET_DIR%/%SFM_TARGET_NAME%.elf %SFM_OUTPUT_TARGET_BINARY_TOP_PATH%/output_files/%SFM_OUTPUT_TARGET_BINARY_PREFIX_NAME%_APP.elf
%SFM_CP_TOOLS% -f %SFM_TARGET_DIR%/%SFM_TARGET_NAME%.map %SFM_OUTPUT_TARGET_BINARY_TOP_PATH%/output_files/%SFM_OUTPUT_TARGET_BINARY_PREFIX_NAME%_APP.map
%SFM_CP_TOOLS% -f %SFM_OUTPUT_SETTING_FILE_NAME% %SFM_OUTPUT_TARGET_BINARY_TOP_PATH%/output_files/%SFM_OUTPUT_TARGET_BINARY_PREFIX_NAME%_Setting.hex
%SFM_CP_TOOLS% -f %SFM_OUTPUT_BL_FILE_NAME% %SFM_OUTPUT_TARGET_BINARY_TOP_PATH%/output_files/%SFM_OUTPUT_TARGET_BINARY_PREFIX_NAME%_BL.hex
%SFM_CP_TOOLS% -f %SFM_INOUT_SOFT_DEVICE_HEX_FILE_NAME% %SFM_OUTPUT_TARGET_BINARY_TOP_PATH%/output_files/%SFM_OUTPUT_TARGET_BINARY_PREFIX_NAME%_SD.hex
%SFM_CP_TOOLS% -f %SFM_OUTPUT_APP_SETTING_FILE_NAME% %SFM_OUTPUT_TARGET_BINARY_TOP_PATH%/output_files/%SFM_OUTPUT_TARGET_BINARY_PREFIX_NAME%_mreged_APP_Setting.hex
%SFM_CP_TOOLS% -f %SFM_OUTPUT_MERGED_ALL_FILE_NAME% %SFM_OUTPUT_TARGET_BINARY_TOP_PATH%/output_files/%SFM_OUTPUT_TARGET_BINARY_PREFIX_NAME%_mreged_ALL.hex

echo copy DFU Package
%SFM_CP_TOOLS% -f %SFM_OUTPUT_DFU_PKG_FILE_NAME% %SFM_OUTPUT_TARGET_BINARY_TOP_PATH%/DFU_packages/%SFM_OUTPUT_TARGET_BINARY_PREFIX_NAME%_DFU.zip

echo copy Factory Download Hex
%SFM_CP_TOOLS% -f %SFM_OUTPUT_MERGED_ALL_FILE_NAME% %SFM_OUTPUT_TARGET_BINARY_TOP_PATH%/factory_download_images/%SFM_OUTPUT_TARGET_BINARY_PREFIX_NAME%_factory.hex

echo copy tools
%SFM_CP_TOOLS% -f ../../../../tools/usr_bin/jlinkarm_nrf52_nrfjprog.dll %SFM_OUTPUT_TARGET_BINARY_TOP_PATH%/factory_download_images/
%SFM_CP_TOOLS% -f ../../../../tools/usr_bin/nrfjprog.dll %SFM_OUTPUT_TARGET_BINARY_TOP_PATH%/factory_download_images/
%SFM_CP_TOOLS% -f ../../../../tools/usr_bin/nrfjprog.exe %SFM_OUTPUT_TARGET_BINARY_TOP_PATH%/factory_download_images/
%SFM_CP_TOOLS% -f ../../../../tools/usr_bin/nrfjprog.ini %SFM_OUTPUT_TARGET_BINARY_TOP_PATH%/factory_download_images/
%SFM_CP_TOOLS% -f ../../../../tools/usr_bin/jlinkarm_nrf52_nrfjprog.dll %SFM_OUTPUT_TARGET_BINARY_TOP_PATH%/bin/
%SFM_CP_TOOLS% -f ../../../../tools/usr_bin/nrfjprog.dll %SFM_OUTPUT_TARGET_BINARY_TOP_PATH%/bin/
%SFM_CP_TOOLS% -f ../../../../tools/usr_bin/nrfjprog.exe %SFM_OUTPUT_TARGET_BINARY_TOP_PATH%/bin/
%SFM_CP_TOOLS% -f ../../../../tools/usr_bin/nrfjprog.ini %SFM_OUTPUT_TARGET_BINARY_TOP_PATH%/bin/

echo make download cmd files
echo make factory_write.cmd
%SFM_ECHO_TOOLS% @echo off> %SFM_OUTPUT_TARGET_BINARY_TOP_PATH%/factory_download_images/%SFM_MODEL_NAME%_factory_write.cmd
echo nrfjprog --reset --program %SFM_OUTPUT_TARGET_BINARY_PREFIX_NAME%_factory.hex -f nrf52 --chiperase --verify>> %SFM_OUTPUT_TARGET_BINARY_TOP_PATH%/factory_download_images/%SFM_MODEL_NAME%_factory_write.cmd
echo if not "%%ERRORLEVEL%%" == "0" (>> %SFM_OUTPUT_TARGET_BINARY_TOP_PATH%/factory_download_images/%SFM_MODEL_NAME%_factory_write.cmd
echo echo Factory Write Error>> %SFM_OUTPUT_TARGET_BINARY_TOP_PATH%/factory_download_images/%SFM_MODEL_NAME%_factory_write.cmd
echo )>> %SFM_OUTPUT_TARGET_BINARY_TOP_PATH%/factory_download_images/%SFM_MODEL_NAME%_factory_write.cmd

echo make download_APP.cmd
echo @echo off> %SFM_OUTPUT_TARGET_BINARY_TOP_PATH%/download_APP.cmd
echo bin\nrfjprog.exe --reset --program output_files/%SFM_OUTPUT_TARGET_BINARY_PREFIX_NAME%_mreged_APP_Setting.hex -f nrf52 --sectorerase --verify>> %SFM_OUTPUT_TARGET_BINARY_TOP_PATH%/download_APP.cmd
echo if not "%%ERRORLEVEL%%" == "0" (>> %SFM_OUTPUT_TARGET_BINARY_TOP_PATH%/download_APP.cmd
echo echo Flash Write Error>> %SFM_OUTPUT_TARGET_BINARY_TOP_PATH%/download_APP.cmd
echo pause>> %SFM_OUTPUT_TARGET_BINARY_TOP_PATH%/download_APP.cmd
echo )>> %SFM_OUTPUT_TARGET_BINARY_TOP_PATH%/download_APP.cmd

echo make download_Factory.cmd
echo @echo off> %SFM_OUTPUT_TARGET_BINARY_TOP_PATH%/download_Factory.cmd
echo bin\nrfjprog.exe --reset --program output_files/%SFM_OUTPUT_TARGET_BINARY_PREFIX_NAME%_mreged_ALL.hex -f nrf52 --chiperase --verify>> %SFM_OUTPUT_TARGET_BINARY_TOP_PATH%/download_Factory.cmd
echo if not "%%ERRORLEVEL%%" == "0" (>> %SFM_OUTPUT_TARGET_BINARY_TOP_PATH%/download_Factory.cmd
echo echo Flash Write Error>> %SFM_OUTPUT_TARGET_BINARY_TOP_PATH%/download_Factory.cmd
echo pause>> %SFM_OUTPUT_TARGET_BINARY_TOP_PATH%/download_Factory.cmd
echo )>> %SFM_OUTPUT_TARGET_BINARY_TOP_PATH%/download_Factory.cmd

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

