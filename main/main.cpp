#include <memory>
#include <string>
#include "esp_err.h"
#include "esp_log.h"

#include <sys/unistd.h>
#include <sys/stat.h>
#include "sdmmc_cmd.h"

#include "main.h"
#include "sdcard.h"



static const char *TAG = "example";
#define EXAMPLE_MAX_CHAR_SIZE    64


extern "C" void app_main(void)
{
    esp_err_t ret;

    SdCard my_sdcard;
    ret = my_sdcard.mountFilesystem();
    if (ret != ESP_OK) 
        return;

    const auto card = my_sdcard.card();
    // const auto &card = 
    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);

    // Use POSIX and C standard library functions to work with files.

    // First create a file.
    const char *file_hello = "/hello.txt";
    char data[EXAMPLE_MAX_CHAR_SIZE];
    snprintf(data, EXAMPLE_MAX_CHAR_SIZE, "%s %s!\n", "Hello", card->cid.name);
    ret = my_sdcard.writeFile(file_hello, data);
    if (ret != ESP_OK) {
        return;
    }

    const char *file_foo = "/foo.txt";

    // Check if destination file exists before renaming
    struct stat st;
    if (stat(file_foo, &st) == 0) {
        // Delete it if it exists
        unlink(file_foo);
    }

    // Rename original file
    ESP_LOGI(TAG, "Renaming file %s to %s", file_hello, file_foo);
    if (rename(file_hello, file_foo) != 0) {
        ESP_LOGE(TAG, "Rename failed");
        return;
    }

    ret = my_sdcard.readFile(file_foo);
    if (ret != ESP_OK) {
        return;
    }

    const std::string file_nihao = "/nihao.txt";
    memset(data, 0, EXAMPLE_MAX_CHAR_SIZE);
    snprintf(data, EXAMPLE_MAX_CHAR_SIZE, "%s %s!\n", "Nihao", card->cid.name);
    ret = my_sdcard.writeFile(file_nihao, data);
    if (ret != ESP_OK) {
        return;
    }

    //Open file for reading
    ret = my_sdcard.readFile(file_nihao);
    if (ret != ESP_OK) {
        return;
    }

    my_sdcard.unmount();
}
