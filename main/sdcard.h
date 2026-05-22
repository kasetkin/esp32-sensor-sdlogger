#pragma once

#include <optional>
#include <memory>
#include <span>
#include <string_view>
#include <esp_err.h>
#include <driver/gpio.h>
#include <sd_protocol_types.h>
#include <hal/spi_types.h>

class SdCard
{
public:
    static constexpr gpio_num_t PIN_NUM_MISO = GPIO_NUM_20;
    static constexpr gpio_num_t PIN_NUM_MOSI = GPIO_NUM_18;
    static constexpr gpio_num_t PIN_NUM_CLK = GPIO_NUM_19;
    static constexpr gpio_num_t PIN_NUM_CS = GPIO_NUM_21;

    [[nodiscard("SD card format failure silently ignored")]]
    esp_err_t format();
    [[nodiscard("SD card not mounted on failure")]]
    esp_err_t mountFilesystem(const std::string &mountPoint = "/sdcard");
    [[nodiscard("unmount failure may corrupt card state")]]
    esp_err_t unmount();
    void printInfoToStdout();

    std::string mountPoint() const;
    bool cardIsMounted() const;

    /// this functions (write/read) can be outside, but it's easy to read code if they are here
    /// filename should start with "/", for example "/myfile.txt"
    [[nodiscard("write failure silently drops data")]]
    esp_err_t writeFile(const std::string &path, std::span<const std::byte> data);
    [[nodiscard("write failure silently drops data")]]
    esp_err_t writeFile(const std::string &path, std::string_view data);

    [[nodiscard("append failure silently drops data")]]
    esp_err_t appendFile(const std::string &path, std::span<const std::byte> data);
    [[nodiscard("append failure silently drops data")]]
    esp_err_t appendFile(const std::string &path, std::string_view data);

    /// filename should start with "/", for example "/myfile.txt"
    // esp_err_t readFileExample(const std::string &path);

    SdCard() = default;
    SdCard(const SdCard &) = delete("SdCard owns an SPI bus handle and sdmmc_card_t* — copying aliases hardware resources");
    SdCard &operator=(const SdCard &) = delete("SdCard owns an SPI bus handle and sdmmc_card_t* — copying aliases hardware resources");

private:
    sdmmc_card_t *m_card = nullptr;
    spi_host_device_t m_spiDevice = SPI_HOST_MAX;
    std::string m_mountPoint = "";

};

