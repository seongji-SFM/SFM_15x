@echo off
RD /Q /S development\SFM\demoApp\pca10040\s132\ses\Output
DEL /Q development\SFM\demoApp\pca10040\s132\ses\*.jlink
DEL /Q development\SFM\demoApp\pca10040\s132\ses\*.emSession

RD /Q /S development\SFM\demoBootloader\pca10040\ses\Output
DEL /Q development\SFM\demoBootloader\pca10040\ses\*.jlink
DEL /Q development\SFM\demoBootloader\pca10040\ses\*.emSession

RD /Q /S development\SFM\tools\SFM_PC-nrfutil\pc-nrfutil\build
RD /Q /S development\SFM\tools\SFM_PC-nrfutil\pc-nrfutil\dist

RD /Q /S development\SFM\binary\bin
RD /Q /S development\SFM\binary\output_files
RD /Q /S development\SFM\binary\DFU_packages
RD /Q /S development\SFM\binary\factory_download_images
DEL /Q development\SFM\binary\download_APP.cmd
DEL /Q development\SFM\binary\download_Factory.cmd
DEL /Q development\SFM\binary\git_commits_*

RD /Q /S development\SFM\tools\wifitools\WIFI_Release_SFM20R
RD /Q /S development\SFM\tools\wifitools\WIFI_Release_SRM200
RD /Q /S development\SFM\tools\wifitools\WIFI_Certification

RD /Q /S development\SFM\tools\RF_Test\SRM200_EVB\RF_TEST_SRM200A