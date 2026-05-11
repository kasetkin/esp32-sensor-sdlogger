#include "unity.h"
#include <cstdio>
#include <cstring>
#include "sdcard.h"

static SdCard g_sdCard;
static bool g_sdMounted = false;

static const std::string MOUNT_POINT = "/sdcard";
static const std::string TEST_FILE   = "/unity_test.txt"; // relative — SdCard methods prepend mount point internally

static void ensureMounted(void)
{
    if (!g_sdMounted) {
        const esp_err_t err = g_sdCard.mountFilesystem(MOUNT_POINT);
        TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, err,
            "SdCard::mountFilesystem() failed - check SD card wiring and insertion");
        g_sdMounted = true;
    }
}

static void test_hw_sdcard_mounts(void)
{
    ensureMounted();
    TEST_ASSERT_TRUE(g_sdCard.cardIsMounted());
}

static void test_hw_sdcard_write_returns_ok(void)
{
    ensureMounted();
    const esp_err_t err = g_sdCard.writeFile(TEST_FILE, "unity test line\n");
    TEST_ASSERT_EQUAL(ESP_OK, err);
}

static void test_hw_sdcard_write_read_back_matches(void)
{
    ensureMounted();

    const char *content = "hello from unity\n";
    TEST_ASSERT_EQUAL(ESP_OK, g_sdCard.writeFile(TEST_FILE, content));

    FILE *f = fopen((MOUNT_POINT + TEST_FILE).c_str(), "r");
    TEST_ASSERT_NOT_NULL_MESSAGE(f, "fopen failed after writeFile");

    char buf[64];
    memset(buf, 0, sizeof(buf));
    const size_t bytesRead = fread(buf, 1, sizeof(buf) - 1, f);
    fclose(f);

    TEST_ASSERT_GREATER_THAN(0, bytesRead);
    TEST_ASSERT_EQUAL_STRING(content, buf);
}

static void test_hw_sdcard_append_adds_content(void)
{
    ensureMounted();

    TEST_ASSERT_EQUAL(ESP_OK, g_sdCard.writeFile(TEST_FILE, "line1\n"));
    TEST_ASSERT_EQUAL(ESP_OK, g_sdCard.appendFile(TEST_FILE, "line2\n"));

    FILE *f = fopen((MOUNT_POINT + TEST_FILE).c_str(), "r");
    TEST_ASSERT_NOT_NULL(f);

    char buf[64];
    memset(buf, 0, sizeof(buf));
    fread(buf, 1, sizeof(buf) - 1, f);
    fclose(f);

    TEST_ASSERT_TRUE(strstr(buf, "line1\n") != nullptr);
    TEST_ASSERT_TRUE(strstr(buf, "line2\n") != nullptr);
}

static void test_hw_sdcard_cleanup(void)
{
    ensureMounted();
    remove((MOUNT_POINT + TEST_FILE).c_str());
    FILE *f = fopen((MOUNT_POINT + TEST_FILE).c_str(), "r");
    TEST_ASSERT_NULL_MESSAGE(f, "test file should not exist after remove()");
}

void run_sdcard_tests(void)
{
    RUN_TEST(test_hw_sdcard_mounts);
    RUN_TEST(test_hw_sdcard_write_returns_ok);
    RUN_TEST(test_hw_sdcard_write_read_back_matches);
    RUN_TEST(test_hw_sdcard_append_adds_content);
    RUN_TEST(test_hw_sdcard_cleanup);
}
