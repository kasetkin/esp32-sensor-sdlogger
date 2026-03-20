#pragma once

#include <optional>
#include "esp_err.h"
#include "sd_protocol_types.h"
#include "hal/spi_types.h"

class SdCard
{
public:
    esp_err_t format();
    esp_err_t mountFilesystem();
    esp_err_t unmount();

    sdmmc_card_t *card();
private:
    std::unique_ptr<sdmmc_card_t> m_card;
    spi_host_device_t m_spiDevice = SPI_HOST_MAX;

};

esp_err_t writeFile(const std::string &path, char *data);
esp_err_t readFile(const std::string &path);