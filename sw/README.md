# ESP32_Interface Software
Facilitate learning the Z80 Retro! hardware and software.

## Information:
The ESP32-S3 SW is in a platformio project format. If using VS Code there is a platformio plugin. Upon opening the project folder the project should self configure.
* To program the board use the JTAG port on the module.
  - First upload the filesystem.
  - Then upload the code.
* On first boot the module enters WiFi host AP mode.
  - Connect local computer to WiFi network "Z80-programmer" with password "Z80-Retro!"
  - IP address is 192.168.4.1
  - Open a webpage at this address to set the WiFi parameters for WiFi client mode in the local network environment.
  - Power cycle.
  - On the local WiFi router find the new IP address that the DHCP server assigned.
* Connect local computer to local WiFi network
  - Open webage and enter the IP address assigned by DHCP server to the Z80-Programmer! 
