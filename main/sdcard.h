#pragma once

#include <optional>
#include <memory>
#include "esp_err.h"
#include "sd_protocol_types.h"
#include "hal/spi_types.h"

class SdCard
{
public:
    esp_err_t format();
    esp_err_t mountFilesystem(const std::string &mountPoint = "/sdcard");
    esp_err_t unmount();
    void printInfoToStdout();

    std::string mountPoint() const;
    sdmmc_card_t *card() const;

    /// this functions (write/read) can be outside, but it's easy to read code if they are here
    /// filename should start with "/", for example "/myfile.txt"
    esp_err_t writeFile(const std::string &path, const char *data);

    esp_err_t appendFile(const std::string &path, const char *data);

    /// filename should start with "/", for example "/myfile.txt"
    esp_err_t readFile(const std::string &path);
private:
    std::unique_ptr<sdmmc_card_t> m_card;
    spi_host_device_t m_spiDevice = SPI_HOST_MAX;
    std::string m_mountPoint = "";

};

