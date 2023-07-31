# ESP32_Interface
Facilitate learning the Z80 Retro! hardware and software.

## Feature list:
ESP32-s3 based board to provide WiFi enabled features.
* RS232 connection to main Z80-Retro console via cable for telnet console access
* RS232 connection to Z80-Retro Aux com port via cable for future WiFi enabled serial download.
* Z80 bus programmer using MCP23S17 SPI IO expander devices.
  - facilitates faster Z80 bus access at 10Mhz SPI rate. 
    - Serial access still relatively slow but 25x I2C.
  - Program Flash without the need for a Raspberry Pi.
  - Might be possible to program areas of SRAM memory e.g. future WiFi file download.
* Off-board SPI using header connection to future board(s) such as FPGA video card.
  - programming FPGA plus additional SPI-FPGA debug features if needed.
* One byte, 8 bit, message register implemented in logic for handshaking with Z80.
  - If the 8 bit message port interface to/from the Z80 Retro! is not needed five IC's can be no stuff, saving cost.
  - uses IORQ addreess line 0xA4.
* USB OTG is present in hardware.
* SD card slot for future possiblities.
