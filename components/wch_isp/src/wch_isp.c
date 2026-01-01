/**
 * @file wch_isp.c
 * @brief WCH ISP main API implementation
 */

#include <stdlib.h>
#include <string.h>
#include "wch_isp.h"
#include "wch_isp_protocol.h"
#include "wch_isp_flash.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "wch_isp";

// Default ISP key (30 bytes of zeros)
static const uint8_t DEFAULT_ISP_KEY[WCH_ISP_KEY_LENGTH] = {0};

// CH32V30x chip variant lookup table
typedef struct {
    uint8_t chip_id;
    const char* name;
    uint32_t flash_size;
} ch32_variant_t;

static const ch32_variant_t ch32_variants[] = {
    { 0x30, "CH32V303VCT6", 256 * 1024 },
    { 0x31, "CH32V303RCT6", 256 * 1024 },
    { 0x32, "CH32V303RBT6", 128 * 1024 },
    { 0x33, "CH32V303CBT6", 128 * 1024 },
    { 0x50, "CH32V305RBT6", 128 * 1024 },
    { 0x70, "CH32V307VCT6", 256 * 1024 },
    { 0x71, "CH32V307RCT6", 256 * 1024 },
    { 0x73, "CH32V307WCU6", 256 * 1024 },
};

static const ch32_variant_t* get_chip_variant(uint8_t chip_id)
{
    for (size_t i = 0; i < sizeof(ch32_variants) / sizeof(ch32_variants[0]); i++) {
        if (ch32_variants[i].chip_id == chip_id) {
            return &ch32_variants[i];
        }
    }
    return NULL;
}

wch_isp_err_t wch_isp_init(const wch_isp_config_t* config, wch_isp_handle_t* out_handle)
{
    if (!config || !out_handle) {
        ESP_LOGE(TAG, "Config or handle pointer is NULL");
        return WCH_ISP_ERR_INVALID_ARG;
    }

    // Allocate handle
    struct wch_isp_handle* handle = (struct wch_isp_handle*)malloc(sizeof(struct wch_isp_handle));
    if (!handle) {
        ESP_LOGE(TAG, "Failed to allocate handle");
        return WCH_ISP_ERR_NO_MEM;
    }

    memset(handle, 0, sizeof(struct wch_isp_handle));

    // Copy configuration
    handle->uart_port = config->uart_port;
    handle->tx_pin = config->tx_pin;
    handle->rx_pin = config->rx_pin;
    handle->baudrate = config->baudrate;
    handle->rst_pin = config->rst_pin;
    handle->boot0_pin = config->boot0_pin;
    handle->timeout_ms = config->timeout_ms;

    // Initialize UART
    wch_isp_err_t err = wch_uart_init(handle);
    if (err != WCH_ISP_OK) {
        ESP_LOGE(TAG, "UART init failed: %d", err);
        free(handle);
        return err;
    }

    // Configure GPIO for RST and BOOT0
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << handle->rst_pin) | (1ULL << handle->boot0_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t gpio_err = gpio_config(&io_conf);
    if (gpio_err != ESP_OK) {
        ESP_LOGE(TAG, "GPIO config failed: %d", gpio_err);
        wch_uart_deinit(handle);
        free(handle);
        return WCH_ISP_ERR_INVALID_ARG;
    }

    // Set initial GPIO states (not in bootloader)
    gpio_set_level(handle->boot0_pin, 0);
    gpio_set_level(handle->rst_pin, 1);

    handle->initialized = true;
    handle->in_bootloader = false;

    ESP_LOGI(TAG, "WCH ISP initialized");

    *out_handle = handle;
    return WCH_ISP_OK;
}

wch_isp_err_t wch_isp_deinit(wch_isp_handle_t handle)
{
    if (!handle) {
        return WCH_ISP_ERR_INVALID_ARG;
    }

    // Exit bootloader if still in it
    if (handle->in_bootloader) {
        wch_isp_exit_bootloader(handle, true);
    }

    // Deinitialize UART
    wch_uart_deinit(handle);

    // Free handle
    free(handle);

    ESP_LOGI(TAG, "WCH ISP deinitialized");

    return WCH_ISP_OK;
}

wch_isp_err_t wch_isp_enter_bootloader(wch_isp_handle_t handle)
{
    if (!handle) {
        return WCH_ISP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Entering bootloader mode");

    // Bootloader entry sequence:
    // 1. Assert BOOT0 high
    // 2. Assert RST low
    // 3. Wait 50ms
    // 4. Release RST high
    // 5. Wait 100ms
    // 6. Release BOOT0 low
    // 7. Flush UART
    // 8. Verify with IDENTIFY

    gpio_set_level(handle->boot0_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    gpio_set_level(handle->rst_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(50));

    gpio_set_level(handle->rst_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    gpio_set_level(handle->boot0_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Flush UART RX buffer
    wch_uart_flush(handle);

    // Verify bootloader mode with IDENTIFY command
    wch_isp_err_t err = wch_cmd_identify(handle, &handle->device_type, &handle->chip_id);
    if (err != WCH_ISP_OK) {
        ESP_LOGE(TAG, "Failed to identify device in bootloader mode");
        return WCH_ISP_ERR_NOT_IN_BOOTLOADER;
    }

    handle->in_bootloader = true;

    ESP_LOGI(TAG, "Bootloader mode entered successfully");

    return WCH_ISP_OK;
}

wch_isp_err_t wch_isp_exit_bootloader(wch_isp_handle_t handle, bool reset)
{
    if (!handle) {
        return WCH_ISP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Exiting bootloader mode");

    // Send ISP_END command if requested
    if (reset && handle->in_bootloader) {
        wch_cmd_isp_end(handle, 1);  // 1 = reset
    }

    // Exit bootloader sequence:
    // 1. Ensure BOOT0 is low
    // 2. Pulse RST

    gpio_set_level(handle->boot0_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(10));

    gpio_set_level(handle->rst_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(50));

    gpio_set_level(handle->rst_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    handle->in_bootloader = false;

    ESP_LOGI(TAG, "Bootloader mode exited");

    return WCH_ISP_OK;
}

wch_isp_err_t wch_isp_get_device_info(wch_isp_handle_t handle, wch_isp_device_info_t* info)
{
    if (!handle || !info) {
        return WCH_ISP_ERR_INVALID_ARG;
    }

    if (!handle->in_bootloader) {
        ESP_LOGE(TAG, "Not in bootloader mode");
        return WCH_ISP_ERR_NOT_IN_BOOTLOADER;
    }

    // Read configuration (bootloader version + UID)
    uint8_t config_data[12];
    wch_isp_err_t err = wch_cmd_read_config(handle, CFG_MASK_BTVER | CFG_MASK_UID,
                                            config_data, sizeof(config_data));
    if (err != WCH_ISP_OK) {
        ESP_LOGE(TAG, "Failed to read config");
        return err;
    }

    // Parse response format: [0x00, major, minor, 0x00, UID[8]]
    // Bytes 0-3: bootloader version [0x00, major, minor, 0x00]
    // Bytes 4-11: UID
    handle->bootloader_ver = ((uint16_t)config_data[1] << 8) | config_data[2];
    memcpy(handle->uid, &config_data[4], 8);

    // Fill device info
    info->device_type = handle->device_type;
    info->chip_id = handle->chip_id;
    memcpy(info->uid, handle->uid, 8);
    info->bootloader_ver = handle->bootloader_ver;

    // Look up chip variant by chip_id
    const ch32_variant_t* variant = get_chip_variant(handle->chip_id);
    if (variant) {
        snprintf(info->device_name, sizeof(info->device_name), "%s", variant->name);
        info->flash_size = variant->flash_size;
    } else {
        snprintf(info->device_name, sizeof(info->device_name), "Unknown CH32V30x (0x%02X)", handle->chip_id);
        info->flash_size = 0;
    }

    ESP_LOGI(TAG, "Device info: %s (Family: 0x%02X, Chip: 0x%02X), BL v%04X",
             info->device_name, info->device_type, info->chip_id, info->bootloader_ver);
    ESP_LOGI(TAG, "UID: %02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X",
             info->uid[0], info->uid[1], info->uid[2], info->uid[3],
             info->uid[4], info->uid[5], info->uid[6], info->uid[7]);

    return WCH_ISP_OK;
}

wch_isp_err_t wch_isp_erase_flash(wch_isp_handle_t handle, size_t size)
{
    if (!handle) {
        return WCH_ISP_ERR_INVALID_ARG;
    }

    if (!handle->in_bootloader) {
        ESP_LOGE(TAG, "Not in bootloader mode");
        return WCH_ISP_ERR_NOT_IN_BOOTLOADER;
    }

    return wch_flash_erase_sectors(handle, size);
}

wch_isp_err_t wch_isp_flash_firmware(wch_isp_handle_t handle,
                                      const uint8_t* data,
                                      size_t size,
                                      uint32_t address)
{
    if (!handle || !data || size == 0) {
        return WCH_ISP_ERR_INVALID_ARG;
    }

    if (!handle->in_bootloader) {
        ESP_LOGE(TAG, "Not in bootloader mode");
        return WCH_ISP_ERR_NOT_IN_BOOTLOADER;
    }

    ESP_LOGI(TAG, "Starting firmware flash: %zu bytes at 0x%08lX", size, address);

    // Step 1: Get device info (to get UID)
    wch_isp_device_info_t info;
    wch_isp_err_t err = wch_isp_get_device_info(handle, &info);
    if (err != WCH_ISP_OK) {
        ESP_LOGE(TAG, "Failed to get device info");
        return err;
    }

    // Step 2: Setup ISP key
    err = wch_cmd_isp_key(handle, DEFAULT_ISP_KEY, WCH_ISP_KEY_LENGTH);
    if (err != WCH_ISP_OK) {
        ESP_LOGE(TAG, "Failed to set ISP key");
        return err;
    }

    // Step 3: Calculate XOR key
    wch_calculate_xor_key(handle, DEFAULT_ISP_KEY, WCH_ISP_KEY_LENGTH);

    // Step 4: Erase flash
    ESP_LOGI(TAG, "Erasing flash...");
    err = wch_flash_erase_sectors(handle, size);
    if (err != WCH_ISP_OK) {
        ESP_LOGE(TAG, "Flash erase failed");
        return err;
    }

    // Step 5: Write firmware
    ESP_LOGI(TAG, "Writing firmware...");
    err = wch_flash_write_all(handle, address, data, size);
    if (err != WCH_ISP_OK) {
        ESP_LOGE(TAG, "Firmware write failed");
        return err;
    }

    ESP_LOGI(TAG, "Firmware flashed successfully");

    return WCH_ISP_OK;
}

wch_isp_err_t wch_isp_verify_firmware(wch_isp_handle_t handle,
                                       const uint8_t* data,
                                       size_t size,
                                       uint32_t address)
{
    if (!handle || !data || size == 0) {
        return WCH_ISP_ERR_INVALID_ARG;
    }

    if (!handle->in_bootloader) {
        ESP_LOGE(TAG, "Not in bootloader mode");
        return WCH_ISP_ERR_NOT_IN_BOOTLOADER;
    }

    ESP_LOGI(TAG, "Starting firmware verify: %zu bytes at 0x%08lX", size, address);

    // Step 1: Get device info (to get UID) - needed for XOR key calculation
    wch_isp_device_info_t info;
    wch_isp_err_t err = wch_isp_get_device_info(handle, &info);
    if (err != WCH_ISP_OK) {
        ESP_LOGE(TAG, "Failed to get device info");
        return err;
    }

    // Step 2: Setup ISP key
    err = wch_cmd_isp_key(handle, DEFAULT_ISP_KEY, WCH_ISP_KEY_LENGTH);
    if (err != WCH_ISP_OK) {
        ESP_LOGE(TAG, "Failed to set ISP key");
        return err;
    }

    // Step 3: Calculate XOR key (requires UID from step 1)
    wch_calculate_xor_key(handle, DEFAULT_ISP_KEY, WCH_ISP_KEY_LENGTH);

    // Verify firmware
    ESP_LOGI(TAG, "Verifying firmware...");
    err = wch_flash_verify_all(handle, address, data, size);
    if (err != WCH_ISP_OK) {
        ESP_LOGE(TAG, "Firmware verify failed");
        return err;
    }

    ESP_LOGI(TAG, "Firmware verified successfully");

    return WCH_ISP_OK;
}

wch_isp_err_t wch_isp_read_option_bytes(wch_isp_handle_t handle,
                                         uint32_t* option_bytes,
                                         size_t count)
{
    if (!handle || !option_bytes || count != 3) {
        return WCH_ISP_ERR_INVALID_ARG;
    }

    if (!handle->in_bootloader) {
        ESP_LOGE(TAG, "Not in bootloader mode");
        return WCH_ISP_ERR_NOT_IN_BOOTLOADER;
    }

    // Read option bytes (12 bytes = 3x uint32_t)
    uint8_t data[12];
    wch_isp_err_t err = wch_cmd_read_config(handle, CFG_MASK_RDPR_USER | CFG_MASK_DATA | CFG_MASK_WRPR,
                                            data, sizeof(data));
    if (err != WCH_ISP_OK) {
        return err;
    }

    // Parse as 32-bit words (little-endian)
    for (size_t i = 0; i < count; i++) {
        option_bytes[i] = data[i*4] | ((uint32_t)data[i*4+1] << 8) |
                          ((uint32_t)data[i*4+2] << 16) | ((uint32_t)data[i*4+3] << 24);
    }

    memcpy(handle->option_bytes, option_bytes, count * sizeof(uint32_t));

    ESP_LOGI(TAG, "Option bytes: 0x%08lX 0x%08lX 0x%08lX",
             option_bytes[0], option_bytes[1], option_bytes[2]);

    return WCH_ISP_OK;
}

wch_isp_err_t wch_isp_write_option_bytes(wch_isp_handle_t handle,
                                          const uint32_t* option_bytes,
                                          size_t count)
{
    if (!handle || !option_bytes || count != 3) {
        return WCH_ISP_ERR_INVALID_ARG;
    }

    if (!handle->in_bootloader) {
        ESP_LOGE(TAG, "Not in bootloader mode");
        return WCH_ISP_ERR_NOT_IN_BOOTLOADER;
    }

    // Convert to bytes (little-endian)
    uint8_t data[12];
    for (size_t i = 0; i < count; i++) {
        data[i*4+0] = (option_bytes[i] >> 0) & 0xFF;
        data[i*4+1] = (option_bytes[i] >> 8) & 0xFF;
        data[i*4+2] = (option_bytes[i] >> 16) & 0xFF;
        data[i*4+3] = (option_bytes[i] >> 24) & 0xFF;
    }

    ESP_LOGW(TAG, "Writing option bytes: 0x%08lX 0x%08lX 0x%08lX",
             option_bytes[0], option_bytes[1], option_bytes[2]);

    // Write option bytes
    wch_isp_err_t err = wch_cmd_write_config(handle,
                                              CFG_MASK_RDPR_USER | CFG_MASK_DATA | CFG_MASK_WRPR,
                                              data, sizeof(data));
    if (err != WCH_ISP_OK) {
        return err;
    }

    memcpy(handle->option_bytes, option_bytes, count * sizeof(uint32_t));

    ESP_LOGI(TAG, "Option bytes written successfully");

    return WCH_ISP_OK;
}

wch_isp_err_t wch_isp_unlock_device(wch_isp_handle_t handle)
{
    if (!handle) {
        return WCH_ISP_ERR_INVALID_ARG;
    }

    if (!handle->in_bootloader) {
        ESP_LOGE(TAG, "Not in bootloader mode");
        return WCH_ISP_ERR_NOT_IN_BOOTLOADER;
    }

    ESP_LOGW(TAG, "Unlocking device - THIS WILL ERASE FLASH!");

    // Read current option bytes
    uint32_t option_bytes[3];
    wch_isp_err_t err = wch_isp_read_option_bytes(handle, option_bytes, 3);
    if (err != WCH_ISP_OK) {
        return err;
    }

    // Set RDPR to 0xA5 (unlocked)
    // Word 0: RDPR | ~RDPR | USER | ~USER
    uint8_t rdpr = 0xA5;
    uint8_t nrdpr = ~rdpr;
    uint8_t user = (option_bytes[0] >> 16) & 0xFF;
    uint8_t nuser = ~user;

    option_bytes[0] = rdpr | ((uint32_t)nrdpr << 8) |
                      ((uint32_t)user << 16) | ((uint32_t)nuser << 24);

    // Write option bytes
    err = wch_isp_write_option_bytes(handle, option_bytes, 3);
    if (err != WCH_ISP_OK) {
        ESP_LOGE(TAG, "Failed to unlock device");
        return err;
    }

    ESP_LOGI(TAG, "Device unlocked successfully");

    return WCH_ISP_OK;
}

const char* wch_isp_err_to_string(wch_isp_err_t err)
{
    switch (err) {
        case WCH_ISP_OK:
            return "Success";
        case WCH_ISP_ERR_INVALID_ARG:
            return "Invalid argument";
        case WCH_ISP_ERR_NO_MEM:
            return "Out of memory";
        case WCH_ISP_ERR_TIMEOUT:
            return "Timeout";
        case WCH_ISP_ERR_UART:
            return "UART error";
        case WCH_ISP_ERR_PROTOCOL:
            return "Protocol error";
        case WCH_ISP_ERR_CHECKSUM:
            return "Checksum mismatch";
        case WCH_ISP_ERR_ERASE:
            return "Flash erase failed";
        case WCH_ISP_ERR_WRITE:
            return "Flash write failed";
        case WCH_ISP_ERR_VERIFY:
            return "Flash verify failed";
        case WCH_ISP_ERR_DEVICE_LOCKED:
            return "Device is read-protected";
        case WCH_ISP_ERR_NOT_IN_BOOTLOADER:
            return "Not in bootloader mode";
        default:
            return "Unknown error";
    }
}
