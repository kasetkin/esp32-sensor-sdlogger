#include <memory>
#include <span>
#include <esp_vfs_fat.h>
#include <sdmmc_cmd.h>

#include "sdcard.h"

static const char *TAGSD = "sdcard-log";

esp_err_t SdCard::writeFile(const std::string &path, std::span<const std::byte> data)
{
    const std::string pathWithMount = m_mountPoint + path;
    ESP_LOGI(TAGSD, "Opening file %s", path.c_str());
    FILE *f = fopen(pathWithMount.c_str(), "w");
    if (f == NULL) {
        ESP_LOGE(TAGSD, "Failed to open file for writing");
        return ESP_FAIL;
    }

    const size_t written = fwrite(data.data(), 1, data.size(), f);
    fclose(f);
    if (written != data.size()) {
        ESP_LOGE(TAGSD, "Can not write to the file %s", path.c_str());
        return ESP_FAIL;
    }

    ESP_LOGI(TAGSD, "File written");
    return ESP_OK;
}

esp_err_t SdCard::writeFile(const std::string &path, std::string_view data)
{
    return writeFile(path, std::as_bytes(std::span{data}));
}

esp_err_t SdCard::appendFile(const std::string &path, std::span<const std::byte> data)
{
    const std::string pathWithMount = m_mountPoint + path;
    ESP_LOGI(TAGSD, "Opening file %s", path.c_str());
    FILE *f = fopen(pathWithMount.c_str(), "a");
    if (f == NULL) {
        ESP_LOGE(TAGSD, "Failed to open file for appending");
        return ESP_FAIL;
    }
    const size_t written = fwrite(data.data(), 1, data.size(), f);
    fclose(f);
    if (written != data.size()) {
        ESP_LOGE(TAGSD, "Can not append to the file %s", path.c_str());
        return ESP_FAIL;
    }
    ESP_LOGI(TAGSD, "File appended");
    return ESP_OK;
}

esp_err_t SdCard::appendFile(const std::string &path, std::string_view data)
{
    return appendFile(path, std::as_bytes(std::span{data}));
}

// esp_err_t SdCard::readFileExample(const std::string &path)
// {
//     const std::string pathWithMount = m_mountPoint + path;
//     ESP_LOGI(TAGSD, "Reading file %s", path);
//     FILE *f = fopen(pathWithMount.c_str(), "r");
//     if (f == NULL) {
//         ESP_LOGE(TAGSD, "Failed to open file for reading");
//         return ESP_FAIL;
//     }
//
//     constexpr size_t EXAMPLE_MAX_CHAR_SIZE = 64;
//     char line[EXAMPLE_MAX_CHAR_SIZE];
//     fgets(line, sizeof(line), f);
//     fclose(f);
//
//     // strip newline
//     char *pos = strchr(line, '\n');
//     if (pos) {
//         *pos = '\0';
//     }
//     ESP_LOGI(TAGSD, "Read from file: '%s'", line);
//
//     return ESP_OK;
// }

esp_err_t SdCard::mountFilesystem(const std::string &mountPoint)
{
    esp_err_t ret;

    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
        .disk_status_check_enable = true,
        .use_one_fat = false
    };
    sdmmc_card_t *card;
    ESP_LOGI(TAGSD, "Initializing SD card");

    // Use settings defined above to initialize SD card and mount FAT filesystem.
    // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
    // Please check its source code and implement error recovery when developing
    // production applications.
    ESP_LOGI(TAGSD, "Using SPI peripheral");

    // By default, SD card frequency is initialized to SDMMC_FREQ_DEFAULT (20MHz)
    // For setting a specific frequency, use host.max_freq_khz (range 400kHz - 20MHz for SDSPI)
    // Example: for fixed frequency of 10MHz, use host.max_freq_khz = 10000;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .data2_io_num = -1,
        .data3_io_num = -1,
        .data4_io_num = -1,
        .data5_io_num = -1,
        .data6_io_num = -1,
        .data7_io_num = -1,
        .data_io_default_level = false,
        .max_transfer_sz = 4000,
        .flags = 0,                         /// not so sure
        .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
        .intr_flags = 0
    };

    const auto spiDevice = static_cast<spi_host_device_t>(host.slot);
    ret = spi_bus_initialize(spiDevice, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAGSD, "Failed to initialize bus.");
        return ESP_FAIL;
    }

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = spiDevice;

    ESP_LOGI(TAGSD, "Mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount(mountPoint.c_str(), &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAGSD, "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAGSD, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }

        spi_bus_free(spiDevice);
        return ESP_FAIL;
    }
    ESP_LOGI(TAGSD, "Filesystem mounted");

    m_card = card;
    m_spiDevice = spiDevice;
    m_mountPoint = mountPoint;
    return ESP_OK;
}

esp_err_t SdCard::unmount()
{
    // All done, unmount partition and disable SPI peripheral
    esp_vfs_fat_sdcard_unmount(m_mountPoint.c_str(), m_card);
    m_card = nullptr;
    m_mountPoint = "";
    ESP_LOGI(TAGSD, "Card unmounted");

    //deinitialize the bus after all devices are removed
    spi_bus_free(m_spiDevice);
    m_spiDevice = SPI_HOST_MAX;

    return ESP_OK;
}

void SdCard::printInfoToStdout()
{
    // Card has been initialized, print its properties
    if (m_card)
        sdmmc_card_print_info(stdout, m_card);
}

bool SdCard::cardIsMounted() const
{
    return m_card != nullptr;
}

std::string SdCard::mountPoint() const
{
    return m_mountPoint;
}

esp_err_t SdCard::format()
{
    // Format FATFS
    esp_err_t ret = esp_vfs_fat_sdcard_format(m_mountPoint.c_str(), m_card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAGSD, "Failed to format FATFS (%s)", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    // if (stat(file_foo, &st) == 0) {
    //     ESP_LOGI(TAGSD, "file still exists");
    //     return;
    // } else {
    //     ESP_LOGI(TAGSD, "file doesn't exist, formatting done");
    // }
    return ESP_OK;
}