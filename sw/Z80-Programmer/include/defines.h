//****************************************************************************
//
//  Global defines for the project
//
//****************************************************************************
#ifndef __DEFINES_H__
#define __DEFINES_H__


//****************************************************************************
//
//  ESP module specific defines
//
//****************************************************************************

// Defines for ESP32 based boards.  Uncomment the one in use
// Note : platformio.ini needs to be modified to specify board as well
// Version needs to be set for V1.0 ESP32_S3_DEVKITC hardware boards to enable 
// swapping the LED port
#define ESP32_S3_DEVKITC
#define ESP32_S3_DEVKITC_VERSION (1)
//#define ESP32_ESP_WROOM_32_DEVKIT

// Defines for ESP2866 based boards.  Uncomment the one in use
// Note : platformio.ini needs to be modified to specify board as well
// Note : ESP2866_12E is a ESP2866 Node MCU ESP-12E
// #define ESP2866_NODE_MCU_12E
// #define ESP2866_WEMOS_D1_MINI

#if ( defined( ESP32_S3_DEVKITC ) || defined( ESP32_ESP_WROOM_32_DEVKIT ) )
    #define ESP32_TYPE_MCU
#else 
    #define ESP2866_TYPE_MCU
#endif

#define WTM_PROG_I2C


#ifdef ESP32_TYPE_MCU
    #ifdef ESP32_S3_DEVKITC
        #define TXD0 (43)
        #define RXD0 (44)
        #define TXD1 (1)
        #define RXD1 (2)
        #define TXD2 (40)
        #define RXD2 (41)

        #undef WTM_PROG_I2C
        #define WTM_PROG_SPI
        #define SCL (36)
        #define SDA (37)

        /* ESP32_S3_DEVKITC use LED on GPIO 48 (V1) or 38 (V1.1)*/
        // so some pin swap needed per version */
        // DP Todo - In future see if we can remove need to hardcode
        // Note - This is an RGB LED so while RGB_CTRL is connected to 
        // GPIO38/48 it is controlled on/off via GPIO LED_BUILTIN (97) */
        #if ESP32_S3_DEVKITC_VERSION == 1
            //ESP32_S3_DEVKITC V1
            #define LED_PIN        (LED_BUILTIN)
            // #define LED_PIN        (48)
            #define WTM_RGB_PIN    (RGB_BUILTIN)
            #define ESP_DATA_DIR_R (38)
        #else // ESP32_S3_DEVKITC V1.1
            #define LED_PIN        (LED_BUILTIN)
            // #define LED_PIN        (38)
            #define WTM_RGB_PIN    (RGB_BUILTIN)
            #define ESP_DATA_DIR_R (48)
        #endif /*  ESP32_S3_DEVKITC_VERSION == 1 */

        #define WTM_SPI_HOST_SD     (SPI2_HOST)
        #define WTM_SPI_SD_MOSI     (47)
        #define WTM_SPI_SD_MISO     (45)
        #define WTM_SPI_SD_CLK      (9)
        #define WTM_SPI_SD_CS0      (21)
        #define WTM_SPI_IO_EXP_CS1  (35)
        #define WTM_SPI_SD_CD       (8)
        // Work around for Rev 0 board where SD bus is
        // shared with other devices with no isolation
        // of MOSI.
        // Set to 1 if Rev 0 board.
        #define WTM_SPI_WORKAROUND  (0)
        
        // According to the datasheet these pins are FSPI = SPI2_HOST
        // However SPI SD crashes if AUX delared after SD bus.
        // No crash if set to SPI3_HOST
        #define WTM_SPI_HOST_AUX    (SPI3_HOST)
        #define WTM_SPI_AUX_MOSI    (11)
        #define WTM_SPI_AUX_MISO    (13)
        #define WTM_SPI_AUX_CLK     (12)
        #define WTM_SPI_AUX_CS0     (10)
        #define WTM_SPI_AUX_CS1     (14)
        #define WTM_AUX_RESET_N     (39)
        #define WTM_AUX_DONE        (3)

        #define WTM_SPI_SD_FREQ       (10000)  // 10MHz - SD card 20MHz max in MMC SPI mode.
        #define WTM_SPI_IO_EXP_FREQ   (10000)  // 10MHz - MCP23S17 10MHz max.
        #define WTM_SPI_AUX_FREQ      (10000)    // 400kHz
        // #define WTM_SPI_SD_FREQ       (400)    // 400kHz
        // #define WTM_SPI_IO_EXP_FREQ   (400)    // 400kHz
        // #define WTM_SPI_AUX_FREQ      (400)    // 400kHz

        #define Z80_DATA_0     (18)
        #define Z80_DATA_1     (17)
        #define Z80_DATA_2     (16)
        #define Z80_DATA_3     (15)
        #define Z80_DATA_4      (7)
        #define Z80_DATA_5      (6)
        #define Z80_DATA_6      (5)
        #define Z80_DATA_7      (4)
        #define Z80_DATA_DIR_W (38)
        #define Z80_INT_N      (42)

        #define Z80_IORQ_A0_RD (37)
        #define Z80_IORQ_A0_WR (36)
        
        #define USE_ON_BOARD_LED 

    #else /* Not ESP32_S3_DEVKITC */
        #define TXD0 (1)
        #define RXD0 (3)
        #define TXD1 (27)
        #define RXD1 (14)
        #define TXD2 (17)
        #define RXD2 (16)

        #define SCL (22)
        #define SDA (21)

        #define LED_PIN     (2)
        #define USE_ON_BOARD_LED 

        #define WTM_SD_MISO  (0)
        #define WTM_SD_MOSI  (0)
        #define WTM_SD_CLK   (0)
        #define WTM_SD_CS    (0)
        #define WTM_SD_CD    (0)
        #define WTM_SD_HOST  (0) 
    #endif /* ESP32_S3_DEVKITC_1 */

#else
    #define WTM_SD_MISO  (0)
    #define WTM_SD_MOSI  (0)
    #define WTM_SD_CLK   (0)
    #define WTM_SD_CS    (0)
    #define WTM_SD_CD    (0)
    #define WTM_SD_HOST  (0) 
#endif /* ESP32_TYPE_MCU */


#ifdef  ESP2866_NODE_MCU_12E
#define LED_PIN     2
// #define SCL         5 // Pin D1 / GPIO 5 
// #define SDA         4 // Pin D2 / GPIO 4
#endif

#ifdef ESP2866_WEMOS_D1_MINI
// Wemos onboard LED is muxed with TX1 so can't use both. If using Serial1 don't use on board LED.
#ifdef USE_ON_BOARD_LED
#define LED_PIN     2
#endif /* USE_ON_BOARD_LED */
//#define SCL (5) // Pin D1 / GPIO 5 
//#define SDA (4) // Pin D2 / GPIO 4
#endif /* ESP2866_WEMOS_D1_MINI */


//****************************************************************************
//
//  Bere are the defines common to all ESP modules 
//
//****************************************************************************

// Determine serial port buffer size and baud rate.
// Mapping to UART ports is done globally in Z80-programmer.cpp (main)
#define CONSOLE_BAUD          wtmSerialX::baud115200
#define CONSOLE_RX_BUF_SIZE   1024
#define LOGGER_BAUD           wtmSerialX::baud115200
#define LOGGER_RX_BUF_SIZE    1024
#define Z80_BAUD              wtmSerialX::baud115200
#define Z80_RX_BUF_SIZE       1024

// LED settings
#define LED_ON      0x1
#define LED_OFF     0x0
#define WTM_RGB_BRIGHTNESS (16)

// Telnet defines.
// Keeps buffer sizes manageable
#define STACK_PROTECTOR  512 // bytes
// Note: Temporary buffer size may be double this number. Change carefully.
#define TELNET_MAX_TX_BUF_SIZE 1024 
// Buffer size setting for optimal WiFi transfer time/size
#define TELNET_AVAILABLE_FOR_WRITE 127
// Setting for how many clients should be able to telnet into the server.
#define MAX_TELNET_CLIENTS 2
#define TELNET_PORT_NUM_CONSOLE 2323
#define TELNET_PORT_NUM_Z80 23

// Web server and WiFi settings
// WiFi access point mode is used if Station settings for netowrk WiFi not present
// Access point settings are hard coded
// We use the ESP default 192.168.4.1/24 address/mask for AP mode
#define WEB_SERVER_PORT 80
#define WTM_AP_SSID "Z80-programmer"
#define WTM_AP_PSWD "Z80-Retro!"

// Default filenames for the Z80 and FPGA flash images
// The name MUST start with /
#define WTM_Z80_FILENAME "/z80retro.bin"
#define WTM_FPGA_FILENAME "/FLEADiP.bin"
// Structure that holds the WiFI Station mode SSID name and password in EEPROM.
typedef struct wifiSettings {
    char magic[6];
    char ssid[33];
    char password[65];
} WiFiSettings;

// enable DBG messages in dbg.h to logger rather than stdout
#define WTM_ENABLE_DBG

// redirect stdout to logger if set to 1. 
// May be helpful in cases where ESP error messages are printed to the console
// such as debugging espressif sd card code.
#define WTM_REDIRECT_STD_OUT (0)

// Uncomment to debug telnet section of SerialX::sxWrite
// #define SX_TEL_RX_DEBUG

// Uncomment to debug telnet section of SerialX::sxReadBytes, sxParseTelnetControlChars
// #define SX_TEL_TX_DEBUG

// Uncomment to debug xmodem transfer
// #define XMDEBUG

// Uncomment to debug WiFi startup
// Note : this will show the WiFi network SSID and password if saved in EEPROM
// #define WIFI_DBG_PRINTS

// Uncomment to turn on debug prints for the SPI bus init and deinit
#define WTM_DBG_SPI_INIT

// Uncomment to turn on debug prints for the SPI bus IO Expander devices
// #define WTM_DBG_IO_EXPANDER

// Uncomment to turn on debug prints for the aux SPI bus.
// #define WTM_DBG_SPI_FPGA
// Uncomment to turn on adress prints for erasing and reading FPGA Flash
// #define WTM_DBG_SPI_FPGA_2

// Uncomment this for board bring-up test to test automatic reading and writing to the Z80.
// Need to run esp32rw.com on the z80 for this code to run.
// Make sure to comment after board bring-up test.
// #define WTM_TEST_Z80_MESSAGE_INTERFACE

#endif  /* __DEFINES_H__ */