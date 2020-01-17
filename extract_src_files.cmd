@echo off

call remove_unnecessary_files.cmd

development\SFM\tools\usr_bin\rm.exe -rf extract_src
development\SFM\tools\usr_bin\mkdir.exe -p extract_src

development\SFM\tools\usr_bin\cp.exe license.txt extract_src/license.txt
development\SFM\tools\usr_bin\cp.exe README.md extract_src/README.md
development\SFM\tools\usr_bin\cp.exe remove_unnecessary_files.cmd extract_src/remove_unnecessary_files.cmd
development\SFM\tools\usr_bin\cp.exe sdk_version.txt extract_src/sdk_version.txt

development\SFM\tools\usr_bin\cp.exe -r components extract_src/components
development\SFM\tools\usr_bin\mkdir.exe -p extract_src/config/nrf52832
development\SFM\tools\usr_bin\cp.exe -r config/nrf52832 extract_src/config
development\SFM\tools\usr_bin\cp.exe -r external extract_src/external
development\SFM\tools\usr_bin\cp.exe -r external_tools extract_src/external_tools
development\SFM\tools\usr_bin\cp.exe -r integration extract_src/integration
development\SFM\tools\usr_bin\cp.exe -r modules extract_src/modules

development\SFM\tools\usr_bin\mkdir.exe -p extract_src/development/SFM/
development\SFM\tools\usr_bin\cp.exe -r development/SFM/binary extract_src/development/SFM/
development\SFM\tools\usr_bin\cp.exe -r development/SFM/common extract_src/development/SFM/
development\SFM\tools\usr_bin\cp.exe -r development/SFM/demoApp extract_src/development/SFM/
development\SFM\tools\usr_bin\cp.exe -r development/SFM/demoApp_Simple extract_src/development/SFM/
development\SFM\tools\usr_bin\cp.exe -r development/SFM/demoBootloader extract_src/development/SFM/

development\SFM\tools\usr_bin\mkdir.exe -p extract_src/development/SFM/tools/SFM_PC-nrfutil
development\SFM\tools\usr_bin\cp.exe -r development/SFM/tools/SFM_PC-nrfutil/bin extract_src/development/SFM/tools/SFM_PC-nrfutil/
development\SFM\tools\usr_bin\cp.exe -r development/SFM/tools/usr_bin extract_src/development/SFM/tools/

pause
