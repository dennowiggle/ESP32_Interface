/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-microsd-card-arduino/
  
  This sketch was mofidied from: Examples > SD(esp32) > SD_Test
*/
#include <stdio.h>      /* printf, scanf, NULL */
#include <stdlib.h>     /* malloc, free, rand */

#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "driver/spi_master.h"
// DP
#include "defines.h"
#include "sd_diskio.h"
#include "driver/sdspi_host.h"
#include "driver/sdmmc_defs.h"
#include "sdmmc_common.h"
#include "wtm_sdmmc_init.h"
// #include "sdmmc_cmd.h"
#include "wtmSerialx.h"
#include "dbg.h"
#include "flash.h"
#include "wtmSpi.h"

// DP
#define SCK     WTM_SPI_SD_CLK
#define MISO    WTM_SPI_SD_MISO
#define MOSI    WTM_SPI_SD_MOSI
#define CS      WTM_SPI_SD_CS0
#define SD_FREQ WTM_SPI_SD_FREQ

extern wtmSerialX* console;

SPIClass spi = SPIClass(WTM_SPI_HOST_SD);

void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\r\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.printf("Failed to open directory \"%s\"\r\n", dirname);
        return;
    }
    if(!root.isDirectory()){
        Serial.printf("Not a directory \"%s\"\r\n", dirname);
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
              listDir(fs, file.name(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

void createDir(fs::FS &fs, const char * path){
    Serial.printf("Creating Dir: %s\r\n", path);
    if(fs.mkdir(path)){
        Serial.println("Dir created");
    } else {
        Serial.println("mkdir failed");
    }
}

void removeDir(fs::FS &fs, const char * path){
    Serial.printf("Removing Dir: %s\r\n", path);
    if(fs.rmdir(path)){
        Serial.println("Dir removed");
    } else {
        Serial.println("rmdir failed");
    }
}

void readFile(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\r\n", path);

    File file = fs.open(path);
    if(!file){
        Serial.println("Failed to open file for reading");
        return;
    }

    Serial.print("Read from file: ");
    while(file.available()){
        Serial.write(file.read());
    }
    Serial.printf("\r\n");
    file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\r\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("File written");
    } else {
        Serial.println("Write failed");
    }
    file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Appending to file: %s\r\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("Failed to open file for appending");
        return;
    }
    if(file.print(message)){
        Serial.println("Message appended");
    } else {
        Serial.println("Append failed");
    }
    file.close();
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
    Serial.printf("Renaming file %s to %s\r\n", path1, path2);
    if (fs.rename(path1, path2)) {
        Serial.println("File renamed");
    } else {
        Serial.println("Rename failed");
    }
}

void deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s\r\n", path);
    if(fs.remove(path)){
        Serial.println("File deleted");
    } else {
        Serial.println("Delete failed");
    }
}

void testFileIO(fs::FS &fs, const char * path){
    File file = fs.open(path);
    static uint8_t buf[512];
    size_t len = 0;
    uint32_t start = millis();
    uint32_t end = start;
    if(file){
        len = file.size();
        size_t flen = len;
        start = millis();
        while(len){
            size_t toRead = len;
            if(toRead > 512){
                toRead = 512;
            }
            file.read(buf, toRead);
            len -= toRead;
        }
        end = millis() - start;
        Serial.printf("%u bytes read for %u ms\r\n", flen, end);
        file.close();
    } else {
        Serial.println("Failed to open file for reading");
    }


    file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }

    size_t i;
    start = millis();
    for(i=0; i<2048; i++){
        file.write(buf, 512);
    }
    end = millis() - start;
    Serial.printf("%u bytes written for %u ms\r\n", 2048 * 512, end);
    file.close();
}

void printCardInfo(uint8_t cardType)
{
    if(cardType == CARD_NONE){
        Serial.println("No SD card attached");
        return;
    }

    Serial.print("SD Card Type: ");
    if(cardType == CARD_MMC){
        Serial.println("MMC");
    } else if(cardType == CARD_SD){
        Serial.println("SDSC");
    } else if(cardType == CARD_SDHC){
        Serial.println("SDHC");
    } else {
        Serial.println("UNKNOWN");
    }    
}

// Override sdmmc_card_print_info() function so we can redirect the output
void wtm_sdmmc_card_print_info(wtmSerialX* port, const sdmmc_card_t* card)
{
    bool print_scr = false;
    bool print_csd = false;
    const char* type;
    port->printf("Name: %s\r\n", card->cid.name);
    if (card->is_sdio) {
        type = "SDIO";
        print_scr = true;
        print_csd = true;
    } else if (card->is_mmc) {
        type = "MMC";
        print_csd = true;
    } else {
        type = (card->ocr & SD_OCR_SDHC_CAP) ? "SDHC/SDXC" : "SDSC";
        print_csd = true;
    }
    port->printf("Type: %s\r\n", type);
    if (card->max_freq_khz < 1000) {
        port->printf("Speed: %d kHz\r\n", card->max_freq_khz);
    } else {
        port->printf("Speed: %d MHz%s\r\n", card->max_freq_khz / 1000,
                card->is_ddr ? ", DDR" : "");
    }
    port->printf("Size: %lluMB\r\n", ((uint64_t) card->csd.capacity) * card->csd.sector_size / (1024 * 1024));

    if (print_csd) {
        port->printf("CSD: ver=%d, sector_size=%d, capacity=%d read_bl_len=%d\r\n",
                (card->is_mmc ? card->csd.csd_ver : card->csd.csd_ver + 1),
                card->csd.sector_size, card->csd.capacity, card->csd.read_block_len);
        if (card->is_mmc) {
            port->printf("EXT CSD: bus_width=%d\r\n", (1 << card->log_bus_width));
        } else if (!card->is_sdio){ // make sure card is SD
            port->printf("SSR: bus_width=%d\r\n", (card->scr.bus_width ? 4 : 1));
        }
    }
    if (print_scr) {
        port->printf("SCR: sd_spec=%d, bus_width=%d\r\n", card->scr.sd_spec, card->scr.bus_width);
    }
}

void sd_app_main()
{
    Serial.begin(115200);
    spi.begin(SCK, MISO, MOSI, CS);
    uint8_t cardType;

    // DP - debug code
    bool boolDebug = false;
    bool boolDebug2 = true;

    if (boolDebug)
    {
        uint8_t pdrv = 0xFF;

        spi.begin(SCK, MISO, MOSI, CS);

        pdrv = sdcard_init(CS, &spi, SD_FREQ * 1000);
        Serial.printf("pdrv (pdrv = sdcard_init) = 0x%02x\r\n", pdrv);
        if (pdrv != ESP_OK)
        {
            Serial.printf("sdcard_init Failed : 0x%02x\r\n", pdrv);
        } else {
            Serial.printf("sdcard_init success : 0x%02x\r\n", pdrv);
        }

        uint8_t cardType = SD.cardType();

        if(cardType == CARD_NONE){
            Serial.println("No SD card attached");
            // return;
        }

        Serial.print("SD Card Type: ");
        if(cardType == CARD_MMC){
            Serial.println("MMC");
        } else if(cardType == CARD_SD){
            Serial.println("SDSC");
        } else if(cardType == CARD_SDHC){
            Serial.println("SDHC");
        } else {
            Serial.println("UNKNOWN");
        }

        uint64_t cardSize = SD.cardSize() / (1024 * 1024);
        Serial.printf("SD Card Size: %lluMB\r\n", cardSize);

        pdrv = sdcard_init(CS, &spi, SD_FREQ * 1000);
        Serial.printf("pdrv (pdrv = sdcard_init) = 0x%02x\r\n", pdrv);
        if (pdrv != ESP_OK)
        {
            Serial.printf("sdcard_init Failed : 0x%02x\r\n", pdrv);
        } else {
            Serial.printf("sdcard_init success : 0x%02x\r\n", pdrv);
        }

        uint8_t ret = sdcard_uninit(pdrv);
        if (ret != ESP_OK)
        {
            Serial.printf("sdcard_uninit Failed : 0x%02x\r\n", ret);
            return;
        } else {
            Serial.printf("sdcard_uninit success : 0x%02x\r\n", ret);
        }

        spi.end();

        return;
    }

    if (!SD.begin(CS,spi,SD_FREQ)) {
        Serial.println("Card Mount Failed");
        return;
    }
    
    cardType = SD.cardType();

    if(cardType == CARD_NONE){
        Serial.println("No SD card attached");
        return;
    }

    Serial.print("SD Card Type: ");
    if(cardType == CARD_MMC){
        Serial.println("MMC");
    } else if(cardType == CARD_SD){
        Serial.println("SDSC");
    } else if(cardType == CARD_SDHC){
        Serial.println("SDHC");
    } else {
        Serial.println("UNKNOWN");
    }

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\r\n", cardSize);

    listDir(SD, "/", 0);
    createDir(SD, "/mydir");
    listDir(SD, "/", 0);
    removeDir(SD, "/mydir");
    listDir(SD, "/", 2);
    writeFile(SD, "/hello.txt", "Hello ");
    appendFile(SD, "/hello.txt", "World!\n");
    readFile(SD, "/hello.txt");
    deleteFile(SD, "/foo.txt");
    renameFile(SD, "/hello.txt", "/foo.txt");
    readFile(SD, "/foo.txt");
    testFileIO(SD, "/test.txt");
    Serial.printf("Total space: %lluMB\r\n", SD.totalBytes() / (1024 * 1024));
    Serial.printf("Used space: %lluMB\r\n", SD.usedBytes() / (1024 * 1024));
}

