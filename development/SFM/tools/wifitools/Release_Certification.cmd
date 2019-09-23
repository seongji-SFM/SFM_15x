mkdir WIFI_Certification
mkdir WIFI_Certification\Certification_Test

copy /Y esptool.exe WIFI_Certification\
copy /Y Certification_Test\* WIFI_Certification\Certification_Test\

copy /Y Certification_Test_write_CE_Adaptivity.cmd WIFI_Certification\
copy /Y Certification_Test_write_RF_TEST.cmd WIFI_Certification\
copy /Y Certification_Test_write_TELEC.cmd WIFI_Certification\

