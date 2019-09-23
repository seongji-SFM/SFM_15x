mkdir RF_TEST_SRM200A
copy /Y ..\..\..\documentation\manual\RF_regulatory_certification_guide_SRM200_EVB_kor.doc RF_TEST_SRM200A\

mkdir RF_TEST_SRM200A\CommonTools
copy /Y ..\..\teraTerm\teraterm-4.102.zip RF_TEST_SRM200A\CommonTools\

mkdir RF_TEST_SRM200A\SIGFOX
copy /Y ..\..\EXE_SRM200A_Tool_v03.zip RF_TEST_SRM200A\SIGFOX\

mkdir RF_TEST_SRM200A\WIFI
mkdir RF_TEST_SRM200A\WIFI\Certification_Test
copy /Y ..\..\wifitools\esptool.exe WIFI\
copy /Y ..\..\wifitools\Certification_Test\* RF_TEST_SRM200A\WIFI\Certification_Test\
copy /Y ..\..\wifitools\Certification_Test_write_CE_Adaptivity.cmd RF_TEST_SRM200A\WIFI\
copy /Y ..\..\wifitools\Certification_Test_write_RF_TEST.cmd RF_TEST_SRM200A\WIFI\
copy /Y ..\..\wifitools\Certification_Test_write_TELEC.cmd RF_TEST_SRM200A\WIFI\

mkdir RF_TEST_SRM200A\BLE
copy /Y ..\..\nRF_Go_Studio\nrfgostudiowin641212installer.zip RF_TEST_SRM200A\BLE\

mkdir RF_TEST_SRM200A\GPS
copy /Y ..\..\u-center\ublox-GNSS_u-center_windows_v8.23.zip RF_TEST_SRM200A\GPS\
copy /Y ..\..\u-center\u-center_UserGuide_(UBX-13005250).pdf RF_TEST_SRM200A\GPS\

