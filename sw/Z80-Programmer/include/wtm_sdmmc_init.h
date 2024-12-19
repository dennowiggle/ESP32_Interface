#ifndef __WTM_SDMMC_INIT_H__
#define __WTM_SDMMC_INIT_H__

#include "sdmmc_common.h"

esp_err_t wtm2_sdmmc_card_init(const sdmmc_host_t* config, sdmmc_card_t* card);

void wtmDecodeError(esp_err_t errCode);

#endif