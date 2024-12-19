#include "sdmmc_common.h"
#include "wtm_sdmmc_init.h"
#include "sdmmc_cmd.h"
#include "sd_diskio.h"
//#include "driver/sdspi_host.h"
//#include "driver/sdmmc_defs.h"
#include "wtmSerialx.h"

extern wtmSerialX* console;

static const char* TAG = "sdmmc_init";


#define SDMMC_INIT_STEP(condition, function) \
    do { \
        if ((condition)) { \
            esp_err_t err = (function)(card); \
            if (err != ESP_OK) { \
                ESP_LOGD(TAG, "%s: %s returned 0x%x", __func__, #function, err); \
                return err; \
            } \
        } \
    } while(0);

esp_err_t wtm_sdmmc_fix_host_flags(sdmmc_card_t* card)
{
    const uint32_t width_1bit = SDMMC_HOST_FLAG_1BIT;
    const uint32_t width_4bit = SDMMC_HOST_FLAG_4BIT;
    const uint32_t width_8bit = SDMMC_HOST_FLAG_8BIT;
    const uint32_t width_mask = width_1bit | width_4bit | width_8bit;

    int slot_bit_width = card->host.get_bus_width(card->host.slot);
    if (slot_bit_width == 1 &&
            (card->host.flags & (width_4bit | width_8bit))) {
        card->host.flags &= ~width_mask;
        card->host.flags |= width_1bit;
    } else if (slot_bit_width == 4 && (card->host.flags & width_8bit)) {
        if ((card->host.flags & width_4bit) == 0) {
            ESP_LOGW(TAG, "slot width set to 4, but host flags don't have 4 line mode enabled; using 1 line mode");
            card->host.flags &= ~width_mask;
            card->host.flags |= width_1bit;
        } else {
            card->host.flags &= ~width_mask;
            card->host.flags |= width_4bit;
        }
    }
    return ESP_OK;
}

esp_err_t wtm_sdmmc_send_cmd(sdmmc_card_t* card, sdmmc_command_t* cmd)
{
    if (card->host.command_timeout_ms != 0) {
        cmd->timeout_ms = card->host.command_timeout_ms;
    } else if (cmd->timeout_ms == 0) {
        cmd->timeout_ms = SDMMC_DEFAULT_CMD_TIMEOUT_MS;
    }

    int slot = card->host.slot;
    console->printf("sending cmd slot=%d op=%d arg=%x flags=%x data=%p blklen=%d datalen=%d timeout=%d\r\n",
            slot, cmd->opcode, cmd->arg, cmd->flags, cmd->data, cmd->blklen, cmd->datalen, cmd->timeout_ms);
    // ESP_LOGV(TAG, "sending cmd slot=%d op=%d arg=%x flags=%x data=%p blklen=%d datalen=%d timeout=%d",
    //         slot, cmd->opcode, cmd->arg, cmd->flags, cmd->data, cmd->blklen, cmd->datalen, cmd->timeout_ms);
    esp_err_t err = (*card->host.do_transaction)(slot, cmd);
    if (err != 0) {
        console->printf("cmd=%d, sdmmc_req_run returned 0x%x\r\n", cmd->opcode, err);
        // ESP_LOGD(TAG, "cmd=%d, sdmmc_req_run returned 0x%x", cmd->opcode, err);
        return err;
    }
    int state = MMC_R1_CURRENT_STATE(cmd->response);
    console->printf("cmd response %08x %08x %08x %08x err=0x%x state=%d\r\n",
               cmd->response[0],
               cmd->response[1],
               cmd->response[2],
               cmd->response[3],
               cmd->error,
               state);
    // ESP_LOGV(TAG, "cmd response %08x %08x %08x %08x err=0x%x state=%d",
    //            cmd->response[0],
    //            cmd->response[1],
    //            cmd->response[2],
    //            cmd->response[3],
    //            cmd->error,
    //            state);
    return cmd->error;
}

esp_err_t wtm_sdmmc_io_rw_direct(sdmmc_card_t* card, int func,
        uint32_t reg, uint32_t arg, uint8_t *byte)
{
    esp_err_t err;
    sdmmc_command_t cmd = {
        .opcode = uint32_t(SD_IO_RW_DIRECT),
        .arg = 0,
        .flags = SCF_CMD_AC | SCF_RSP_R5,
    };

    arg |= (func & SD_ARG_CMD52_FUNC_MASK) << SD_ARG_CMD52_FUNC_SHIFT;
    arg |= (reg & SD_ARG_CMD52_REG_MASK) << SD_ARG_CMD52_REG_SHIFT;
    arg |= (*byte & SD_ARG_CMD52_DATA_MASK) << SD_ARG_CMD52_DATA_SHIFT;
    cmd.arg = arg;

    err = wtm_sdmmc_send_cmd(card, &cmd);
    if (err != ESP_OK) {
        ESP_LOGV(TAG, "%s: sdmmc_send_cmd returned 0x%x", __func__, err);
        return err;
    }

    *byte = SD_R5_DATA(cmd.response);

    return ESP_OK;
}

esp_err_t wtm_sdmmc_io_reset(sdmmc_card_t* card)
{
    uint8_t sdio_reset = CCCR_CTL_RES;
    esp_err_t err = wtm_sdmmc_io_rw_direct(card, 0, SD_IO_CCCR_CTL, SD_ARG_CMD52_WRITE, &sdio_reset);
    if (err == ESP_ERR_TIMEOUT || (host_is_spi(card) && err == ESP_ERR_NOT_SUPPORTED)) {
        /* Non-IO cards are allowed to time out (in SD mode) or
         * return "invalid command" error (in SPI mode).
         */
    } else if (err == ESP_ERR_NOT_FOUND) {
        ESP_LOGD(TAG, "%s: card not present", __func__);
        return err;
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s: unexpected return: 0x%x", __func__, err );
        return err;
    }
    return ESP_OK;
}

void wtmDecodeError(esp_err_t errCode)
{
    switch(errCode)
    {
        case ESP_OK:
            console->printf("ESP_OK = Success\r\n");
            break;
        case ESP_FAIL:
            console->printf("ESP_FAIL = Failure\r\n");
            break;
        case ESP_ERR_NO_MEM:
            console->printf("ESP_ERR_NO_MEM = Out of memory\r\n");
            break;
        case ESP_ERR_INVALID_ARG:
            console->printf("ESP_ERR_INVALID_ARG = Invalid argument\r\n");
            break;
        case ESP_ERR_INVALID_STATE:
            console->printf("ESP_ERR_INVALID_STATE = Invalid state\r\n");
            break;
        case ESP_ERR_INVALID_SIZE:
            console->printf("ESP_ERR_INVALID_SIZE = Invalid size\r\n");
            break;
        case ESP_ERR_NOT_FOUND:
            console->printf("ESP_ERR_NOT_FOUND = Requested resource not found\r\n");
            break;
        case ESP_ERR_NOT_SUPPORTED:
            console->printf("ESP_ERR_NOT_SUPPORTED = peration or feature not supported\r\n");
            break;
        case ESP_ERR_TIMEOUT:
            console->printf("ESP_ERR_TIMEOUT = Operation timed out\r\n");
            break;
        case ESP_ERR_INVALID_RESPONSE:
            console->printf("ESP_ERR_INVALID_RESPONSE = Received response was invalid\r\n");
            break;
        case ESP_ERR_INVALID_CRC:
            console->printf("ESP_ERR_INVALID_CRC = CRC or checksum was invalid\r\n");
            break;
        case ESP_ERR_INVALID_VERSION:
            console->printf("ESP_ERR_INVALID_VERSION = Version was invalid\r\n");
            break;
        case ESP_ERR_INVALID_MAC:
            console->printf("ESP_ERR_INVALID_MAC = MAC address was invalid\r\n");
            break;
        case ESP_ERR_NOT_FINISHED:
            console->printf("ESP_ERR_NOT_FINISHED = There are items remained to retrieve\r\n");
            break;
        default:
            console->printf("Detail not available for error code = 0x%04x\r\n", errCode);
            break;
    }
}

esp_err_t wtm_sdmmc_send_cmd_send_if_cond(sdmmc_card_t* card, uint32_t ocr)
{
    const uint8_t pattern = 0xaa; /* any pattern will do here */
    sdmmc_command_t cmd = {
        .opcode = SD_SEND_IF_COND,
        .arg = uint32_t( (((ocr & SD_OCR_VOL_MASK) != 0) << 8) | pattern),
        .flags = SCF_CMD_BCR | SCF_RSP_R7,
    };
    console->printf("   %s: sending .opcode=0x%x, .flags=0x%x", __func__, cmd.opcode, cmd.flags);
    esp_err_t err = wtm_sdmmc_send_cmd(card, &cmd);
    if (err != ESP_OK) {
        return err;
    }
    uint8_t response = cmd.response[0] & 0xff;
    console->printf("   %s: received=0x%x expected=0x%x", __func__, response, pattern);
    if (response != pattern) {
        ESP_LOGD(TAG, "%s: received=0x%x expected=0x%x", __func__, response, pattern);
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

esp_err_t wtm_sdmmc_init_sd_if_cond(sdmmc_card_t* card)
{
    /* SEND_IF_COND (CMD8) command is used to identify SDHC/SDXC cards.
     * SD v1 and non-SD cards will not respond to this command.
     */
    uint32_t host_ocr = get_host_ocr(card->host.io_voltage);
    esp_err_t err = wtm_sdmmc_send_cmd_send_if_cond(card, host_ocr);
    if (err == ESP_OK) {
        ESP_LOGD(TAG, "SDHC/SDXC card");
        host_ocr |= SD_OCR_SDHC_CAP;
    } else if (err == ESP_ERR_TIMEOUT) {
        ESP_LOGD(TAG, "CMD8 timeout; not an SD v2.00 card");
    } else if (host_is_spi(card) && err == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGD(TAG, "CMD8 rejected; not an SD v2.00 card");
    } else {
        ESP_LOGE(TAG, "%s: send_if_cond (1) returned 0x%x", __func__, err);
        return err;
    }
    card->ocr = host_ocr;
    return ESP_OK;
}

esp_err_t wtm_sdmmc_io_send_op_cond(sdmmc_card_t* card, uint32_t ocr, uint32_t *ocrp)
{
    esp_err_t err = ESP_OK;
    sdmmc_command_t cmd = {
        .opcode = SD_IO_SEND_OP_COND,
        .arg = (ocr),
        .flags = SCF_CMD_BCR | SCF_RSP_R4,
    };
    for (size_t i = 0; i < 100; i++) {
        err = wtm_sdmmc_send_cmd(card, &cmd);
        if (err != ESP_OK) {
            break;
        }
        if ((MMC_R4(cmd.response) & SD_IO_OCR_MEM_READY) ||
            ocr == 0) {
            break;
        }
        err = ESP_ERR_TIMEOUT;
        vTaskDelay(SDMMC_IO_SEND_OP_COND_DELAY_MS / portTICK_PERIOD_MS);
    }
    if (err == ESP_OK && ocrp != NULL)
        *ocrp = MMC_R4(cmd.response);

    return err;
}

esp_err_t wtm_sdmmc_io_enable_int(sdmmc_card_t* card)
{
    if (card->host.io_int_enable == NULL) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    return (*card->host.io_int_enable)(card->host.slot);
}

esp_err_t wtm_sdmmc_init_io(sdmmc_card_t* card)
{
    /* IO_SEND_OP_COND(CMD5), Determine if the card is an IO card.
     * Non-IO cards will not respond to this command.
     */
    esp_err_t err = wtm_sdmmc_io_send_op_cond(card, 0, &card->ocr);
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "%s: io_send_op_cond (1) returned 0x%x; not IO card", __func__, err);
        card->is_sdio = 0;
        card->is_mem = 1;
    } else {
        card->is_sdio = 1;

        if (card->ocr & SD_IO_OCR_MEM_PRESENT) {
            ESP_LOGD(TAG, "%s: IO-only card", __func__);
            card->is_mem = 0;
        }
        card->num_io_functions = SD_IO_OCR_NUM_FUNCTIONS(card->ocr);
        ESP_LOGD(TAG, "%s: number of IO functions: %d", __func__, card->num_io_functions);
        if (card->num_io_functions == 0) {
            card->is_sdio = 0;
        }
        uint32_t host_ocr = get_host_ocr(card->host.io_voltage);
        host_ocr &= card->ocr;
        err = wtm_sdmmc_io_send_op_cond(card, host_ocr, &card->ocr);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "%s: sdmmc_io_send_op_cond (1) returned 0x%x", __func__, err);
            return err;
        }
        err = wtm_sdmmc_io_enable_int(card);
        if (err != ESP_OK) {
            ESP_LOGD(TAG, "%s: sdmmc_enable_int failed (0x%x)", __func__, err);
        }
    }
    return ESP_OK;
}

esp_err_t wtm_sdmmc_send_cmd_crc_on_off(sdmmc_card_t* card, bool crc_enable)
{
    assert(host_is_spi(card) && "CRC_ON_OFF can only be used in SPI mode");
    sdmmc_command_t cmd = {
            .opcode = SD_CRC_ON_OFF,
            .arg = uint32_t(crc_enable ? 1 : 0),
            .flags = SCF_CMD_AC | SCF_RSP_R1
    };
    return wtm_sdmmc_send_cmd(card, &cmd);
}

esp_err_t wtm_sdmmc_init_spi_crc(sdmmc_card_t* card)
{
    /* In SD mode, CRC checks of data transfers are mandatory and performed
     * by the hardware. In SPI mode, CRC16 of data transfers is optional and
     * needs to be enabled.
     */
    assert(host_is_spi(card));
    esp_err_t err = wtm_sdmmc_send_cmd_crc_on_off(card, true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s: sdmmc_send_cmd_crc_on_off returned 0x%x", __func__, err);
        return err;
    }
    return ESP_OK;
}

esp_err_t wtm_sdmmc_send_app_cmd(sdmmc_card_t* card, sdmmc_command_t* cmd)
{
    sdmmc_command_t app_cmd = {
        .opcode = MMC_APP_CMD,
        .arg = uint32_t(MMC_ARG_RCA(card->rca)),
        .flags = SCF_CMD_AC | SCF_RSP_R1,
    };
    esp_err_t err = wtm_sdmmc_send_cmd(card, &app_cmd);
    if (err != ESP_OK) {
        return err;
    }
    // Check APP_CMD status bit (only in SD mode)
    if (!host_is_spi(card) && !(MMC_R1(app_cmd.response) & MMC_R1_APP_CMD)) {
        ESP_LOGW(TAG, "card doesn't support APP_CMD");
        return ESP_ERR_NOT_SUPPORTED;
    }
    return wtm_sdmmc_send_cmd(card, cmd);
}

esp_err_t wtm_sdmmc_send_cmd_send_op_cond(sdmmc_card_t* card, uint32_t ocr, uint32_t *ocrp)
{
    esp_err_t err;

    sdmmc_command_t cmd = {
            .opcode = SD_APP_OP_COND,
            .arg = ocr,
            .flags = SCF_CMD_BCR | SCF_RSP_R3,
    };
    int nretries = SDMMC_SEND_OP_COND_MAX_RETRIES;
    int err_cnt = SDMMC_SEND_OP_COND_MAX_ERRORS;
    for (; nretries != 0; --nretries)  {
        bzero(&cmd, sizeof cmd);
        cmd.arg = ocr;
        cmd.flags = SCF_CMD_BCR | SCF_RSP_R3;
        if (!card->is_mmc) { /* SD mode */
            cmd.opcode = SD_APP_OP_COND;
            err = wtm_sdmmc_send_app_cmd(card, &cmd);
        } else { /* MMC mode */
            cmd.arg &= ~MMC_OCR_ACCESS_MODE_MASK;
            cmd.arg |= MMC_OCR_SECTOR_MODE;
            cmd.opcode = MMC_SEND_OP_COND;
            err = wtm_sdmmc_send_cmd(card, &cmd);
        }

        if (err != ESP_OK) {
            if (--err_cnt == 0) {
                ESP_LOGD(TAG, "%s: sdmmc_send_app_cmd err=0x%x", __func__, err);
                return err;
            } else {
                ESP_LOGV(TAG, "%s: ignoring err=0x%x", __func__, err);
                continue;
            }
        }
        // In SD protocol, card sets MEM_READY bit in OCR when it is ready.
        // In SPI protocol, card clears IDLE_STATE bit in R1 response.
        if (!host_is_spi(card)) {
            if ((MMC_R3(cmd.response) & MMC_OCR_MEM_READY) ||
                ocr == 0) {
                break;
            }
        } else {
            if ((SD_SPI_R1(cmd.response) & SD_SPI_R1_IDLE_STATE) == 0) {
                break;
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    if (nretries == 0) {
        return ESP_ERR_TIMEOUT;
    }
    if (ocrp) {
        *ocrp = MMC_R3(cmd.response);
    }
    return ESP_OK;
}

esp_err_t wtm_sdmmc_send_cmd_read_ocr(sdmmc_card_t *card, uint32_t *ocrp)
{
    assert(ocrp);
    sdmmc_command_t cmd = {
        .opcode = SD_READ_OCR,
        .flags = SCF_CMD_BCR | SCF_RSP_R2
    };
    esp_err_t err = wtm_sdmmc_send_cmd(card, &cmd);
    if (err != ESP_OK) {
        return err;
    }
    *ocrp = SD_SPI_R3(cmd.response);
    return ESP_OK;
}

esp_err_t wtm_sdmmc_init_ocr(sdmmc_card_t* card)
{
    esp_err_t err;
    /* In SPI mode, READ_OCR (CMD58) command is used to figure out which voltage
     * ranges the card can support. This step is skipped since 1.8V isn't
     * supported on the ESP32.
     */

    uint32_t host_ocr = get_host_ocr(card->host.io_voltage);
    if ((card->ocr & SD_OCR_SDHC_CAP) != 0) {
        host_ocr |= SD_OCR_SDHC_CAP;
    }
    /* Send SEND_OP_COND (ACMD41) command to the card until it becomes ready. */
    err = wtm_sdmmc_send_cmd_send_op_cond(card, host_ocr, &card->ocr);

    /* If time-out, re-try send_op_cond as MMC */
    if (err == ESP_ERR_TIMEOUT && !host_is_spi(card)) {
        ESP_LOGD(TAG, "send_op_cond timeout, trying MMC");
        card->is_mmc = 1;
        err = wtm_sdmmc_send_cmd_send_op_cond(card, host_ocr, &card->ocr);
    }

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s: send_op_cond (1) returned 0x%x", __func__, err);
        return err;
    }
    if (host_is_spi(card)) {
        err = wtm_sdmmc_send_cmd_read_ocr(card, &card->ocr);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "%s: read_ocr returned 0x%x", __func__, err);
            return err;
        }
    }
    ESP_LOGD(TAG, "host_ocr=0x%x card_ocr=0x%x", host_ocr, card->ocr);

    /* Clear all voltage bits in host's OCR which the card doesn't support.
     * Don't touch CCS bit because in SPI mode cards don't report CCS in ACMD41
     * response.
     */
    host_ocr &= (card->ocr | (~SD_OCR_VOL_MASK));
    ESP_LOGD(TAG, "sdmmc_card_init: host_ocr=%08x, card_ocr=%08x", host_ocr, card->ocr);
    return ESP_OK;
}

esp_err_t wtm_sdmmc_send_cmd_go_idle_state(sdmmc_card_t* card)
{
    sdmmc_command_t cmd = {
        .opcode = MMC_GO_IDLE_STATE,
        .flags = SCF_CMD_BC | SCF_RSP_R0,
    };
    esp_err_t err = wtm_sdmmc_send_cmd(card, &cmd);
    if (host_is_spi(card)) {
        /* To enter SPI mode, CMD0 needs to be sent twice (see figure 4-1 in
         * SD Simplified spec v4.10). Some cards enter SD mode on first CMD0,
         * so don't expect the above command to succeed.
         * SCF_RSP_R1 flag below tells the lower layer to expect correct R1
         * response (in SPI mode).
         */
        (void) err;
        vTaskDelay(SDMMC_GO_IDLE_DELAY_MS / portTICK_PERIOD_MS);

        // DP - Some cards don't seem to like idle state 2 //
        // Todo - If ACMD41 doesn't work try the two idle states
        // console->printf("wtm_sdmmc_send_cmd(MMC_GO_IDLE_STATE_2);\r\n");
        // delay(500);
        // err = wtm_sdmmc_send_cmd(card, &cmd);
        cmd.flags |= SCF_RSP_R1;
        // DP - Some cards don't seem to like idle state 2 //
        // Todo - If ACMD41 doesn't work try the two idle states
        err = wtm_sdmmc_send_cmd(card, &cmd);
    }
    if (err == ESP_OK) {
        vTaskDelay(SDMMC_GO_IDLE_DELAY_MS / portTICK_PERIOD_MS);
    }
    return err;
}


esp_err_t wtm2_sdmmc_card_init(const sdmmc_host_t* config, sdmmc_card_t* card)
{
    esp_err_t ret;

    memset(card, 0, sizeof(*card));
    memcpy(&card->host, config, sizeof(*config));
    const bool is_spi = host_is_spi(card);
    const bool always = true;
    const bool io_supported = true;

    /* Reset SDIO (CMD52, RES) before re-initializing IO (CMD5). */
    console->printf("\r\n");
    console->printf("**** (CMD52) wtm_sdmmc_io_reset\r\n");
    delay(500);
    ret = wtm_sdmmc_io_reset(card);
    if (ret != ESP_OK)
    {
        console->printf("   wtm_sdmmc_io_reset Failed : 0x%04x\r\n", ret);
        wtmDecodeError(ret);
        return ret;
    } else {
        console->printf("   wtm_sdmmc_io_reset success : 0x%04x\r\n", ret);
    }

    /* send CMD0 & expect reply message = 0x01 (enter SPI mode) */
    /* GO_IDLE_STATE (CMD0) command resets the card */
    console->printf("\r\n");
    console->printf("**** CMD0 - wtm_sdmmc_send_cmd_go_idle_state\r\n");
    delay(500);
    ret = wtm_sdmmc_send_cmd_go_idle_state(card);
    if (ret != ESP_OK)
    {
        console->printf("   wtm_sdmmc_send_cmd_go_idle_state Failed : 0x%04x\r\n", ret);
        wtmDecodeError(ret);
        return ret;
    } else {
        console->printf("   wtm_sdmmc_send_cmd_go_idle_state success : 0x%04x\r\n", ret);
    }

    /* 2. send CMD8 (establish that the host uses Version 2.0 SD SPI protocol) */
    /* SEND_IF_COND (CMD8) command is used to identify SDHC/SDXC cards. */
    console->printf("\r\n");
    console->printf("**** (CMD8) used to identify SDHC/SDXC cards - wtm_sdmmc_init_sd_if_cond\r\n");
    delay(500);
    ret = wtm_sdmmc_init_sd_if_cond(card);
    if (ret != ESP_OK)
    {
        console->printf("   wtm_sdmmc_init_sd_if_cond Failed : 0x%04x\r\n", ret);
        wtmDecodeError(ret);
        return ret;
    } else {
        console->printf("   wtm_sdmmc_init_sd_if_cond success : 0x%04x\r\n", ret);
    }

    // /* IO_SEND_OP_COND(CMD5), Determine if the card is an IO card. */
    // console->printf("\r\n");
    // console->printf("**** (CMD5) Determine if the card is an IO card- wtm_sdmmc_init_io\r\n");
    // delay(500);
    // ret = wtm_sdmmc_init_io(card);
    // console->printf("   0x106 = ESP_ERR_NOT_SUPPORTED (card is an IO card)\r\n");
    // if (ret != ESP_OK)
    // {
    //     console->printf("   wtm_sdmmc_init_io Failed : 0x%04x\r\n", ret);
    //     decodeError(ret);
    //     return ret;
    // } else {
    //     console->printf("   wtm_sdmmc_init_io success : 0x%04x\r\n", ret);
    // }

    // const bool is_mem = card->is_mem;
    // const bool is_sdio = !is_mem;

    // // DP - Not really needed for this application
    // /* Enable CRC16 checks for data transfers in SPI mode */
    // console->printf("\r\n");
    // console->printf("**** (CMD59) Enable CRC16 checks for data transfers in SPI mode - wtm_sdmmc_init_spi_crc\r\n");
    // delay(500);
    // ret = wtm_sdmmc_init_spi_crc(card);
    // if (ret != ESP_OK)
    // {
    //     console->printf("   wtm_sdmmc_init_spi_crc Failed : 0x%04x\r\n", ret);
    //     decodeError(ret);
    //     return ret;
    // } else {
    //     console->printf("   wtm_sdmmc_init_spi_crc success : 0x%04x\r\n", ret);
    // }

    /* (ACMD41 + CMD58) Use SEND_OP_COND to set up card OCR */
    console->printf("\r\n");
    console->printf("**** (ACMD41 (CMD55 + 41) + CMD58 SEND_OP_COND to set up card OCR - wtm_sdmmc_init_ocr\r\n");
    delay(500);
    ret = wtm_sdmmc_init_ocr(card);
    if (ret != ESP_OK)
    {
        console->printf("   wtm_sdmmc_init_ocr Failed : 0x%04x\r\n", ret);
        wtmDecodeError(ret);
        return ret;
    } else {
        console->printf("   wtm_sdmmc_init_ocr success : 0x%04x\r\n", ret);
    }

    // const bool is_mmc = is_mem && card->is_mmc;
    // const bool is_sdmem = is_mem && !is_mmc;

    // console->printf("\r\n%s: card type is %s\r\n", __func__,
    //         is_sdio ? "SDIO" : is_mmc ? "MMC" : "SD");

    console->printf("\r\n");

    return 0;

}

