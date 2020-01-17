@echo off

..\tools\usr_bin\sed.exe -i '/\#define CDEV_MODEL_NAME/c\#define CDEV_MODEL_NAME \"SFM20R\"   \/\/MODEL NAME SIZE IS 6BYTE' ../common/config/cfg_config_defines.h
..\tools\usr_bin\sed.exe -i '/\#define CDEV_MODULE_TYPE/c\#define CDEV_MODULE_TYPE                        CDEV_MODULE_SFM20R' ../common/config/cfg_config_defines.h
..\tools\usr_bin\sed.exe -i '/\#define CDEV_BOARD_TYPE/c\#define CDEV_BOARD_TYPE                          CDEV_BOARD_IHERE  \/\/REPLACE_DEVICE_DEFINE_HERE' ../common/config/cfg_config_defines.h
..\tools\usr_bin\unix2dos.exe ../common/config/cfg_config_defines.h
:end

