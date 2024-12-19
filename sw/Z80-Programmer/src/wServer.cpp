/*************************************************************************
 * 
 *    Z80-Retro! ESP Programmer
 *    Code & ESP Programmer Board by Denno Wiggle
 *    Z80-Retro project by John Winans
 *     - https://github.com/Z80-Retro
 * 
 *    The web interface for this project was inspired by code written by 
 *    user bitfixer for the romulator project, some parts of which have been 
 *    reused and re-written.
 *     - https://github.com/bitfixer/bf-romulator
 * 
 *************************************************************************
 *
 *    This library is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU Lesser General Public
 *    License as published by the Free Software Foundation; either
 *    version 2.1 of the License, or (at your option) any later version.
 *
 *    This library is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *    Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with this library; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301
 *    USA
 *
 *****************************************************************************/
#include "wtmSerialx.h"
#include "webServer.h"
#include "defines.h"

#ifdef ESP32_TYPE_MCU
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#endif /* ESP32_TYPE_MCU */

#ifdef ESP2866_TYPE_MCU
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>
#endif /* ESP2866_TYPE_MCU */

#include <Arduino.h>
#include <EEPROM.h>
#include <LittleFS.h>
#include "wtmProgrammer.h"
#include "dbg.h"

//****************************************************************************
//    Global variable declaration
//****************************************************************************

// Declare externally defined global variables
extern wtmSerialX* console;
extern wtmSerialX* logger;

// WiFi station mode setting
const char *ssid = WTM_AP_SSID;
const char *password = WTM_AP_PSWD;

// Global variable to hold WiFi station mode setting
WiFiSettings wifiApSetting;

// Holds boolean for access point mode or station mode. 
// Station mode is for normal use where connection is made to a network WiFi router
// Access point mode is used for setting the username and password required for network access
// so that web pages and telnet can be hosted.
bool accessPointMode;

// Used when we upload files over the web interface.
File fsUploadFile;

// Define which port to use for the web server
#ifdef ESP32_TYPE_MCU
WebServer server(WEB_SERVER_PORT);
#else  /* ESP2866_TYPE_MCU */
ESP8266WebServer server(WEB_SERVER_PORT);
#endif /* ESP32_TYPE_MCU */

// Reference external globally declared programmer class that is used
// for programming Z80-Retro Flash using files that have been uploaded
// via the web portal
extern wtmProgrammer flashProgrammer;

// Global variable to hold the currect LED state (on/off)
extern uint8_t ledState;

// Variable used to hold a time variable so that we can flash the LED
// if WiFi not connected so the user has a visual indication of WiFi state
int _lastMillis;

//****************************************************************************
// Debug code that can be used to debug web communication issues with client
//****************************************************************************
void reportServerArgs()
{
    bool printAllHeaders = false;
    // Check if server arguments ere received
    if (server.args() > 0 ) 
    {
        console->printf("Arguments received\r\n");

        // Loop and display all client-server arguments received
        String argumentName, clientResponse;
        for ( uint8_t i = 0; i < server.args(); i++ ) {
            console->printf("%s", server.argName(i).c_str());
            argumentName = server.argName(i);
            if (server.argName(i) == "user_input") {
                console->printf(" Input received was: %s\r\n", server.arg(i).c_str());
                clientResponse = server.arg(i);
            }
        }
    } else {
        static int i = 0;
        console->printf("%d request checks performed\r\n", ++i);
    }
    if (printAllHeaders)
    {
        console->printf("server.hostHeader() = %s\r\n", server.hostHeader());
    }
}


//****************************************************************************
// Clear the WiFi settings held in EEPROM
//****************************************************************************
void clearWifiSettings()
{
    WiFiSettings newSettings;
    memset((void*)&newSettings, 0, sizeof(WiFiSettings));
    EEPROM.put(0, newSettings);
    EEPROM.commit();

    console->printf("WiFi settings cleared.\r\n");
}

//****************************************************************************
// Display WiFi status and IP address
//****************************************************************************
void displayWifiSettings()
{
    if (WiFi.status() == WL_CONNECTED)
    {
        if (WiFi.getMode() == WIFI_AP) 
        {
            console->printf("WiFi in Access Point mode, IP address is %s\r\n", WiFi.softAPIP().toString().c_str());
        }
        else
        {
            console->printf("WiFi connected to %s, IP address %s\r\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
        }
    }
    else
    {
        console->printf("WiFi not connected\r\n");
    }
}

//****************************************************************************
// handlePortal() sets up the root web page 
// 1. In Access Point mode to handle setting of the users network WiFi SSID and password.
// 2. In STA mode to serve the project root web page.
//****************************************************************************
void handlePortal() {
    if (accessPointMode) 
    {
        if (server.method() == HTTP_POST) {
            strncpy(wifiApSetting.ssid,     server.arg("ssid").c_str(),     sizeof(wifiApSetting.ssid) );
            strncpy(wifiApSetting.password, server.arg("password").c_str(), sizeof(wifiApSetting.password) );
            strcpy(wifiApSetting.magic, "BFROM");
            wifiApSetting.ssid[server.arg("ssid").length()] = wifiApSetting.password[server.arg("password").length()] = '\0';
            EEPROM.put(0, wifiApSetting);
            EEPROM.commit();
            File fp = LittleFS.open("/wifiSetupComplete.html", "r");
            server.streamFile(fp, "text/html");
        } else {
            File fp = LittleFS.open("/wifiSetup.html", "r");
            server.streamFile(fp, "text/html");

        }
    }
    else
    {
        console->printf("The main webpage has been requested.\r\n");
        File fp = LittleFS.open("/z80retro.html", "r");
        server.streamFile(fp, "text/html");
        
        fp.close();
        console->printf("Webpage sent\r\n");
    }
}

//****************************************************************************
// handleFileUpload() is used to upload a Z80 binary image from the web client
// and store it in the local filesystem with filename z80retro.bin
//****************************************************************************
void handleFileUpload(bool isZ80Flash)
{
    int fileSize = 0;
    String fileName;

    if (server.args())
    {
        String confStr = server.arg(0);
        fileSize = atoi(confStr.c_str());
    }
    uint64_t flashSize = isZ80Flash ? Z80_FLASH_SIZE : FPGA_FLASH_SIZE;
    bool boolFileTooLarge = (fileSize > flashSize);

    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START)
    {
        if (boolFileTooLarge)
        {
            console->printf("File is too large. Please send a file of %d kByte maximum.\r\n", flashSize / 1024);
            server.send(200, "text/html", "500: File too large!");
        } else {
            // use fixed filename for upload
            fileName = isZ80Flash? WTM_Z80_FILENAME : WTM_FPGA_FILENAME;
            console->printf("handleFileUpload Name: %s\r\n", fileName.c_str()); 
            fsUploadFile = LittleFS.open(fileName, "w");            // Open the file for writing in LittleFS (create if it doesn't exist)
            fileName = String();
        }
    } 
    else if (upload.status == UPLOAD_FILE_WRITE)
    {
        if (boolFileTooLarge) {
            console->printf("Abort file upload\r\n");
            server.send(200, "text/html", "500: File too large!");
        } else if (fsUploadFile) {
            if ( (upload.totalSize + upload.currentSize) > flashSize )
            {
                server.send(200, "text/html", "500: File too large!");
                boolFileTooLarge = true;
            } else {
                // console->printf("Writing %d bytes out of %d bytes total.\r\n", upload.currentSize, upload.totalSize);
                fsUploadFile.write(upload.buf, upload.currentSize); // Write the received bytes to the file
            }
        }
    } 
    else if (upload.status == UPLOAD_FILE_END) 
    {
        if ( (fsUploadFile) && (!boolFileTooLarge) )
        {   
            // If the file was successfully created close the file and print
            // the file size to the console
            fsUploadFile.close();
            console->printf("handleFileUpload Size: %d bytes\r\n", upload.totalSize); 
            // start programming using the uploaded file
            flashProgrammer.setProgrammingFromWeb(true);
            if (isZ80Flash)
            {
                flashProgrammer.setFlashType(flashProgrammer.Z80_FLASH);
            } else {
                flashProgrammer.setFlashType(flashProgrammer.FPGA_FLASH);
            }
            flashProgrammer.beginProgramming((char *)(fileName.c_str()));
            server.send(200, "text/html", "done uploading!");
        } else {
            server.send(500, "text/plain", "500: couldn't create file");
        }
    }
}

void handleZ80FileUpload()
{
    handleFileUpload(true);
}

void handleFpgaFileUpload()
{
    handleFileUpload(false);
}

//****************************************************************************
// handleProgress() is used to send the file programming progress to the 
// web client for client display
//****************************************************************************
void handleProgress() {
    char progressStr[32];
    int progressPct = flashProgrammer.getProgrammingPercentage();
    sprintf(progressStr, "%d", progressPct);
    server.send(200, "text/html", progressStr);
}

//****************************************************************************
// handleZ80Reset() is used to command the reset process of the Z80-Retro 
// main board and communicate back to the web client upon completion
//****************************************************************************
void handleZ80Reset() {
    flashProgrammer.resetZ80();
    server.send(200, "text/html", "Z80 Retro Computer reset completed.");
}

//****************************************************************************
// handleEspReset() is used to reset the ESP MCU (this board). A message
// is sent to the web client before reset has started as this code will 
// restart with reset. The webclient should auto reconnect after ESP restart
// and the WiFi webserver has restarted.
//****************************************************************************
void handleEspReset() {
    server.send(200, "text/html", "ESP Wifi Module reset in progress. <br/>Page will reload after reset.");
    delay(100);
    ESP.restart();
}

//****************************************************************************
// handleZ80js() is used to send /z80retro.js javascript code to the web
// client upon request. It is this webclient javascriopt code that 
// communicates user requested actions with this server.
//****************************************************************************
void handleZ80js()
{
    File fp = LittleFS.open("/z80retro.js", "r");
    server.streamFile(fp, "application/javascript");

    fp.close();
}

//****************************************************************************
// handleCss() sends /wtm.css stylesheet to the web client upon request. This
// sets the webpages to use a standard WTM color and layout scheme common
// with other WTM programs. 
//****************************************************************************
void handleCss()
{
    File fp = LittleFS.open("/wtm.css", "r");
    server.streamFile(fp, "text/css");

    fp.close();
}

//****************************************************************************
// handleGetBinImage() sends the locally (server) stored /z80retro.bin file 
// to the web client upon request. This is the file that the user may have 
// uploaded in the past, or the standard default image that comes with 
// ESP Programmer.
// Todo : In future it would be nice to have a function to read the Flash
//        on the Z80 Retro main board and store that image as /z80retro.bin
//****************************************************************************
void handleGetBinImage() 
{
    File fp = LittleFS.open(WTM_Z80_FILENAME, "r");
    char buffer[100];
    String fileName = WTM_Z80_FILENAME;
    sprintf(buffer, "attachment; filename=\"%s\"", fileName.substring(1, fileName.length()));
    String strBuffer = buffer;
    server.sendHeader("content-disposition", strBuffer);
    server.streamFile(fp, "application/octet-stream");
    fp.close();
}

//****************************************************************************
// handleFavicon() sends the WTM logo image to the web client as /favicon.ico
//****************************************************************************
void handleFavicon()
{
    File fp = LittleFS.open("/favicon.ico", "r");
    server.streamFile(fp, "image/ico");
    
    fp.close();
}

//****************************************************************************
// validateWifiSettings() validates the WiFi settings. If not valid then
// the settings are cleared.
//****************************************************************************
void validateWifiSettings(WiFiSettings* settings)
{
    // verify we have valid wifi settings
    if (settings->magic[5] != 0)
    {
        memset(settings, 0, sizeof(WiFiSettings));
    }

    if (strcmp("BFROM", settings->magic) != 0)
    {
        memset(settings, 0, sizeof(WiFiSettings));
    }
}

//****************************************************************************
// startServer() is the main function is this code. It sets up and starts the
// webserver and links HTTP GET requests with the functions that handle those
// requests.
//****************************************************************************
void startServer()
{
#ifdef WIFI_DBG_PRINTS
    bool dbgPrints = true;
#else
    bool dbgPrints = false;
#endif

#ifdef USE_ON_BOARD_LED
    ledState = LED_OFF;
    #ifdef WTM_RGB_PIN
        neopixelWrite(WTM_RGB_PIN, 0, WTM_RGB_BRIGHTNESS, 0); // Green ON
    #else
        digitalWrite(LED_PIN, ledState);
    #endif
#endif

    // Get the WiFi settings from the EEPROM (usually an area in ESP Flash memory)
    // and validate them. 
    console->printf("Getting wifi data from EEPROM\r\n");
    EEPROM.get(0, wifiApSetting);
    validateWifiSettings(&wifiApSetting);

    // If the WiFi SSID has not been set then SSID name and password to connect to
    // an external network router in STA station mode is not known. In this case
    // the server will need to start in Access Point mode
    if (wifiApSetting.ssid[0] == 0)
    {
        accessPointMode = true;
    } else {
        accessPointMode = false;
    }

    // Congifure no sleep to improve WiFi response time. Sleep puts the WiFi 
    // module in low power mode at the expense of poor response times. So
    // we need to set no sleep to get a responsive server. This is important
    // when we come to telnet console / z80uart command line mode.
    WiFi.setSleep(WIFI_PS_NONE);

    // If not in Access Point mode set up the WiFi server in station mode. This is the mode
    // that connects to the users WiFi network router for user network access to
    // this program.
    if (accessPointMode == false)
    {
        uint8_t tries = 0;

        if (dbgPrints) console->printf("WiFi.mode(WIFI_STA)\r\n");
        WiFi.mode(WIFI_STA);
        delay(500);
        
        console->printf("Connecting to Wifi SSID = %s.", wifiApSetting.ssid);
        if (dbgPrints) 
        {
            console->printf("\r\n\nConnecting to Wifi SSID = %s, password = %s\r\n", wifiApSetting.ssid, wifiApSetting.password);
            delay(500);
        }
        WiFi.begin(wifiApSetting.ssid, wifiApSetting.password);
        if (dbgPrints) delay(500);

        while ( (WiFi.status() != WL_CONNECTED) && (accessPointMode == false) ) 
        {
            console->printf(".");
            if (tries++ > 20 || wifiApSetting.ssid[0] == 0) 
            {
                accessPointMode = true;
                console->printf("Timeout\r\n");
            }

#ifdef USE_ON_BOARD_LED
            if (ledState == LED_OFF)
            {
                ledState = LED_ON;
            } else {
                ledState = LED_OFF;
            }

    #ifdef WTM_RGB_PIN
            neopixelWrite(WTM_RGB_PIN, 0, 0, (ledState == LED_OFF)? 0 : WTM_RGB_BRIGHTNESS); // Blue ON/OFF
            // Need a delay after neopixelWrite otherwise the next neopixelWrite happens soon after and the ESP32 locks up.
            // We need a delay anyway, so no added delay by placiing here.
            delay(500);
    #else
            digitalWrite(LED_PIN, ledState);
    #endif
#endif
        }
    }

    // If we are in Access Point mode it most likely means we do not have the credentials 
    // to configure WiFi network access, so we need to set up as an access point
    // so the user can configure credentials for WiFi network access.
    if (accessPointMode) 
    {
        console->printf("Starting access point.\r\n");
        if (dbgPrints)
        { 
            console->printf("    status: %s\r\n", String(WiFi.status()) );
            console->printf("WiFi.begin())\r\n");
            WiFi.begin();
            console->printf("WiFi.mode(WIFI_AP)\r\n");
        }

        WiFi.mode(WIFI_MODE_AP);

        if (dbgPrints)
        { 
            console->printf("    status: %s\r\n", String(WiFi.status()) );
            console->printf("WiFi.softAP(ssid=%s, password=%s)\r\n\n\n\n", ssid, password);
        }

        // We use the ESP default 192.168.4.1/24 address/mask for Access Point mode
        WiFi.softAP(ssid, password);
        console->printf("Access Point mode, ip address is %s\r\n", WiFi.softAPIP().toString().c_str());

        if (dbgPrints) console->printf("    status: %s\r\n", String(WiFi.status()) );

    } else {

        // In STA station mode = connected
#ifdef USE_ON_BOARD_LED
        // in station mode connected we show the LED as a solid ON.
        ledState = LED_ON;
    #ifdef WTM_RGB_PIN
        neopixelWrite(WTM_RGB_PIN, 0, 0, (ledState == LED_OFF)? 0 : WTM_RGB_BRIGHTNESS); // Blue ON
    #else
        digitalWrite(LED_PIN, ledState);
    #endif
#endif
        console->printf(" Connected.\r\nIP address = %s\r\n", WiFi.localIP().toString().c_str());
    }

    // Here we set up client-server communication commands and start the web server
    server.on("/",  handlePortal);
    server.on("/uploadZ80", HTTP_POST, [](){server.send(200);}, handleZ80FileUpload);
    server.on("/uploadFpga", HTTP_POST, [](){server.send(200);}, handleFpgaFileUpload);
    server.on("/progress", handleProgress);
    server.on("/resetZ80", handleZ80Reset);
    server.on("/resetEsp", handleEspReset);
    server.on("/wtm.css", handleCss);
    server.on("/z80retro.js", handleZ80js);
    server.on("/getBinImage",handleGetBinImage);
    server.on("/favicon.ico", handleFavicon);
    server.begin();
    if (dbgPrints) delay(500);
    console->printf("HTTP server started.\r\n");

    _lastMillis = millis();
}

        
//****************************************************************************
// The purpose of function handleClient() is to add an LED flashing feature
// in Access Point mode before the call to server.handleClient()
//****************************************************************************
void handleClient()
{
    if (accessPointMode)
    {
        int nowMillis = millis();
        if (nowMillis - _lastMillis >= 500)
        {
#ifdef USE_ON_BOARD_LED
            if (ledState == LED_OFF)
            {
                ledState = LED_ON;
            }
            else
            {
                ledState = LED_OFF;
            }
    #ifdef WTM_RGB_PIN
        neopixelWrite(WTM_RGB_PIN, 0,  (ledState == LED_OFF)? 0 : WTM_RGB_BRIGHTNESS, 0); // Green ON/OFF
    #else
        digitalWrite(LED_PIN, ledState);
    #endif
#endif
            _lastMillis += 500;
        }
    }

    server.handleClient();
}