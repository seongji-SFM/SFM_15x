### based example
examples/dfu/secure_bootloader/pca10040_ble

### emProject files (segger embedded studio)
development/SFM/demoBootloader/pca10040/ses/SFM_demoBL_s132_pca10040.emProject

### how to build with SES(segger embedded studio)
click Build->Build SFM_demoAPP_pca10040_s132 (or push F7)

### how to update secure KEY (For security, create and apply each model.) 
The guide below only supports windows 64.
If you use a different operating system, you should use python.
For more information, please visit the nordic semiconductor web page.
```
* use development\sigfox_cfg2\tools\nrftools\nrfutil.exe

nrfutil keys generate priv.pem : make key file
nrfutil keys display --key pk --format code priv.pem --out_file public_key.c : make pk file

* Add pk data to the public_key.c file (development/SFM/demoBootloader/dfu_public_key.c).
build SFM_demoBL and copy hex file to development/SFM/demoBootloader/hex

* You may need to modify the batch file that creates the DFU package. (Demo is separated by "CDEV_BOARD_TYPE")
```

### how to make update package
The guide below only supports windows 64.

### how to update with DFU ble
The mp4 file is below - development/SFM/experimental/ihere_android_app/how_to_download_over_FOTA.mp4
[![how to download over FOTA](../documentation/pics/how_to_download_over_FOTA.png)](https://youtu.be/YftrZyONju8)

### how to update with DFU Serial
The guide below only supports windows 64.

### LED in DFU
```
Serial Ctrl Connected:  Long  On / Short Off
Wait DFU Download    :  Long  On / Long  Off
Updating             :  Short On / Short Off
ERROR                :  Short On / Long  Off
```
