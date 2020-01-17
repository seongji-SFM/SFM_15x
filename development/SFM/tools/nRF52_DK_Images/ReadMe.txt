DK보드용 바이너리들

download_blinky_LEDS.cmd
  -> LED만 주기적으로 깜박임
download_uart_c_AutoCon.cmd
  -> BLE NUS를 Uart로 Bypass시켜준다.
      * 예) GPS Bypass
        단말 : CMB 입력하여 GPS to NUS모드로 전환
        DK보드 : uart_c_AutoCon\make_ble_app_uart_c_hex.cmd 를 편집하여 Auto Connect MAC Address를 변경
                   예)set MAC_TO_CONNECT_NUS=F07FEB6BAE30
                   make_ble_app_uart_c_hex.cmd를 실행하여 바이너리 생성.
                   download_uart_c_AutoCon.cmd를 실행하여 DK보드에 다운로드.
        기능 : DK보드에 연결된 Virtual Uart Port로 GPS를 제어할 수 있다.
        build필요시 : examples\ble_central\ble_app_uart_c기반으로 개발 -> src_in_ble_app_uart_c.zip의 main.c와 sdk_config.h를 merge한다.
                         MAC을 변경하는 배치파일을 수정한다. (0xa1a2a3a4a5a6를 Target mac으로 변경. Intel Hex 한라인 혹은 두라인을 수정해야한다.)
                           make_ble_app_uart_c_hex.cmd.old1 -> 한라인을 수정하는 base cmd
                           make_ble_app_uart_c_hex.cmd.old1 -> 두라인을 수정하는 base cmd