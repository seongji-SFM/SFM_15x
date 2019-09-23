mkdir WIFI_Release_SFM20R
mkdir WIFI_Release_SFM20R\bin_spi_at

copy /Y esptool.exe WIFI_Release_SFM20R\

copy /Y flash_write_R1_CE.cmd WIFI_Release_SFM20R\
copy /Y flash_write_R2_FCC.cmd WIFI_Release_SFM20R\
copy /Y flash_write_R3_TELEC.cmd WIFI_Release_SFM20R\
copy /Y flash_write_R4_FCC.cmd WIFI_Release_SFM20R\

copy /Y bin_spi_at\boot_v1.6.bin WIFI_Release_SFM20R\bin_spi_at\
copy /Y bin_spi_at\blank_multiboot_0x81000.bin WIFI_Release_SFM20R\bin_spi_at\
copy /Y bin_spi_at\ESP8266_SDK_INIT_PARAM_26M_20170421.bin WIFI_Release_SFM20R\bin_spi_at\

copy /Y bin_spi_at\esp_init_data_default_V0302_CE.bin WIFI_Release_SFM20R\bin_spi_at\
copy /Y bin_spi_at\SPT_AT_ModemSLP_GPIO_VER_FLASH_CE_301117.bin WIFI_Release_SFM20R\bin_spi_at\

copy /Y bin_spi_at\esp_init_data_default_V0402_FCC_R2.bin WIFI_Release_SFM20R\bin_spi_at\
copy /Y bin_spi_at\SPT_AT_ModemSLP_GPIO_VER_FLASH_FCC_301117.bin WIFI_Release_SFM20R\bin_spi_at\

copy /Y bin_spi_at\esp_init_data_default_V0502_TELEC.bin WIFI_Release_SFM20R\bin_spi_at\
copy /Y bin_spi_at\SPT_AT_ModemSLP_GPIO_VER_FLASH_TELEC_301117.bin WIFI_Release_SFM20R\bin_spi_at\

copy /Y bin_spi_at\esp_init_data_default_V0602_FCC_R4.bin WIFI_Release_SFM20R\bin_spi_at\
copy /Y bin_spi_at\SPT_AT_ModemSLP_GPIO_VER_FLASH_FCC_301117.bin WIFI_Release_SFM20R\bin_spi_at\