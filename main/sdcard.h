#pragma once

#include <optional>
#include "esp_err.h"
#include "sd_protocol_types.h"
#include "hal/spi_types.h"

class SdCard
{
public:
    esp_err_t format_sdcard();
    esp_err_t mount_sdcard_filesystem();
    esp_err_t unmount_sdcard();

    sdmmc_card_t *card();
private:
    std::unique_ptr<sdmmc_card_t> m_card;
    spi_host_device_t m_spi_device = SPI_HOST_MAX;

};

esp_err_t s_example_write_file(const std::string &path, char *data);
esp_err_t s_example_read_file(const std::string &path);