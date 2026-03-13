#include <memory>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#include "sdcard.h"

//! \todo remove
#define EXAMPLE_MAX_CHAR_SIZE    64

#define PIN_NUM_MISO  GPIO_NUM_20
#define PIN_NUM_MOSI  GPIO_NUM_18
#define PIN_NUM_CLK   GPIO_NUM_19
#define PIN_NUM_CS    GPIO_NUM_21

static std::string MOUNT_POINT = "/sdcard";
static const char *TAGSD = "example";

esp_err_t s_example_write_file(const std::string &path, char *data)
{
    const std::string pathWithMount = MOUNT_POINT + path;
    ESP_LOGI(TAGSD, "Opening file %s", path);
    FILE *f = fopen(pathWithMount.c_str(), "w");
    if (f == NULL) {
        ESP_LOGE(TAGSD, "Failed to open file for writing");
        return ESP_FAIL;
    }
    fprintf(f, data);
    fclose(f);
    ESP_LOGI(TAGSD, "File written");

    return ESP_OK;
}

esp_err_t s_example_read_file(const std::string &path)
{
    const std::string pathWithMount = MOUNT_POINT + path;
    ESP_LOGI(TAGSD, "Reading file %s", path);
    FILE *f = fopen(pathWithMount.c_str(), "r");
    if (f == NULL) {
        ESP_LOGE(TAGSD, "Failed to open file for reading");
        return ESP_FAIL;
    }
    char line[EXAMPLE_MAX_CHAR_SIZE];
    fgets(line, sizeof(line), f);
    fclose(f);

    // strip newline
    char *pos = strchr(line, '\n');
    if (pos) {
        *pos = '\0';
    }
    ESP_LOGI(TAGSD, "Read from file: '%s'", line);

    return ESP_OK;
}

esp_err_t SdCard::mount_sdcard_filesystem()
{
    esp_err_t ret;

    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
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

    const auto spi_device = static_cast<spi_host_device_t>(host.slot);
    ret = spi_bus_initialize(spi_device, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAGSD, "Failed to initialize bus.");
        return ESP_FAIL;
    }

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = spi_device;

    ESP_LOGI(TAGSD, "Mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT.c_str(), &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAGSD, "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAGSD, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return ESP_FAIL;
    }
    ESP_LOGI(TAGSD, "Filesystem mounted");

    m_card = std::unique_ptr<sdmmc_card_t>(card);
    m_spi_device = spi_device;
    return ESP_OK;
}

esp_err_t SdCard::unmount_sdcard()
{
    // All done, unmount partition and disable SPI peripheral
    esp_vfs_fat_sdcard_unmount(MOUNT_POINT.c_str(), m_card.get());
    m_card.reset();
    ESP_LOGI(TAGSD, "Card unmounted");

    //deinitialize the bus after all devices are removed
    spi_bus_free(m_spi_device);
    m_spi_device = SPI_HOST_MAX;
    return ESP_OK;
}

sdmmc_card_t *SdCard::card()
{
    if (m_card) 
        return m_card.get();
    else
        return nullptr;
}

esp_err_t SdCard::format_sdcard()
{
    // Format FATFS
    esp_err_t ret = esp_vfs_fat_sdcard_format(MOUNT_POINT.c_str(), m_card.get());
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