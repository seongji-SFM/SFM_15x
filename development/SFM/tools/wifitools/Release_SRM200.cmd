mkdir WIFI_Release_SRM200
mkdir WIFI_Release_SRM200\bin_spi_at

copy /Y esptool.exe WIFI_Release_SRM200\

copy /Y flash_write_SRM_WIFI.cmd WIFI_Release_SRM200\

copy /Y bin_spi_at\boot_v1.6.bin WIFI_Release_SRM200\bin_spi_at\
copy /Y bin_spi_at\blank_multiboot_0x81000.bin WIFI_Release_SRM200\bin_spi_at\
copy /Y bin_spi_at\ESP8266_SDK_INIT_PARAM_26M_20170421.bin WIFI_Release_SRM200\bin_spi_at\

copy /Y bin_spi_at\esp_init_data_default_V0701_SRM200.bin WIFI_Release_SRM200\bin_spi_at\
copy /Y bin_spi_at\user1.1024.new.2_20190820.bin WIFI_Release_SRM200\bin_spi_at\
