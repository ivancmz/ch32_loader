/**
 * @file wch_isp_flash.c
 * @brief Flash operations implementation for WCH ISP
 */

#include <string.h>
#include "wch_isp_flash.h"
#include "esp_log.h"

static const char* TAG = "wch_isp_flash";

wch_isp_err_t wch_flash_erase_sectors(struct wch_isp_handle* handle,
                                       size_t firmware_size)
{
    if (!handle) {
        return WCH_ISP_ERR_INVALID_ARG;
    }

    // Calculate sector count (1KB sectors)
    uint32_t sector_count = (firmware_size + 1023) / 1024;

    ESP_LOGI(TAG, "Erasing %lu sectors for %zu bytes", sector_count, firmware_size);

    return wch_cmd_erase(handle, sector_count);
}

wch_isp_err_t wch_flash_write_chunk(struct wch_isp_handle* handle,
                                     uint32_t address,
                                     const uint8_t* data,
                                     uint16_t len)
{
    if (!handle || !data) {
        return WCH_ISP_ERR_INVALID_ARG;
    }

    if (len > WCH_MAX_PROGRAM_DATA) {
        ESP_LOGE(TAG, "Chunk too large: %d > %d", len, WCH_MAX_PROGRAM_DATA);
        return WCH_ISP_ERR_INVALID_ARG;
    }

    // Prepare buffer for encryption
    uint8_t buffer[WCH_MAX_PROGRAM_DATA];
    memcpy(buffer, data, len);

    // XOR encrypt data
    wch_xor_encrypt(buffer, len, handle->xor_key);

    // Align to 8 bytes, pad with XOR key pattern
    uint16_t aligned_len = (len + 7) & ~7;
    for (uint16_t i = len; i < aligned_len; i++) {
        buffer[i] = handle->xor_key[i % 8];
    }

    // Write to flash
    return wch_cmd_program(handle, address, buffer, aligned_len);
}

wch_isp_err_t wch_flash_verify_chunk(struct wch_isp_handle* handle,
                                      uint32_t address,
                                      const uint8_t* data,
                                      uint16_t len)
{
    if (!handle || !data) {
        return WCH_ISP_ERR_INVALID_ARG;
    }

    if (len > WCH_MAX_PROGRAM_DATA) {
        ESP_LOGE(TAG, "Chunk too large: %d > %d", len, WCH_MAX_PROGRAM_DATA);
        return WCH_ISP_ERR_INVALID_ARG;
    }

    // Prepare buffer for encryption
    uint8_t buffer[WCH_MAX_PROGRAM_DATA];
    memcpy(buffer, data, len);

    // XOR encrypt data
    wch_xor_encrypt(buffer, len, handle->xor_key);

    // Align to 8 bytes, pad with XOR key pattern
    uint16_t aligned_len = (len + 7) & ~7;
    for (uint16_t i = len; i < aligned_len; i++) {
        buffer[i] = handle->xor_key[i % 8];
    }

    // Verify flash contents
    return wch_cmd_verify(handle, address, buffer, aligned_len);
}

wch_isp_err_t wch_flash_write_all(struct wch_isp_handle* handle,
                                   uint32_t address,
                                   const uint8_t* data,
                                   size_t size)
{
    if (!handle || !data) {
        return WCH_ISP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Writing %zu bytes to 0x%08lX", size, address);

    const uint8_t* ptr = data;
    size_t remaining = size;
    uint32_t current_addr = address;
    size_t total_written = 0;

    while (remaining > 0) {
        // Calculate chunk size (max 56 bytes)
        uint16_t chunk_size = (remaining > WCH_MAX_PROGRAM_DATA) ?
                               WCH_MAX_PROGRAM_DATA : remaining;

        // Write chunk
        wch_isp_err_t err = wch_flash_write_chunk(handle, current_addr,
                                                   ptr, chunk_size);
        if (err != WCH_ISP_OK) {
            ESP_LOGE(TAG, "Write failed at 0x%08lX", current_addr);
            return err;
        }

        // Update progress
        ptr += chunk_size;
        current_addr += chunk_size;
        remaining -= chunk_size;
        total_written += chunk_size;

        // Log progress every 1KB
        if ((total_written % 1024) == 0 || remaining == 0) {
            ESP_LOGI(TAG, "Progress: %zu / %zu bytes (%d%%)",
                     total_written, size, (int)(total_written * 100 / size));
        }
    }

    // Send final zero-length write to signal completion
    ESP_LOGD(TAG, "Sending final zero-length write");
    wch_isp_err_t err = wch_cmd_program(handle, current_addr, NULL, 0);
    if (err != WCH_ISP_OK) {
        ESP_LOGE(TAG, "Final write failed");
        return err;
    }

    ESP_LOGI(TAG, "Write complete: %zu bytes", size);

    return WCH_ISP_OK;
}

wch_isp_err_t wch_flash_verify_all(struct wch_isp_handle* handle,
                                    uint32_t address,
                                    const uint8_t* data,
                                    size_t size)
{
    if (!handle || !data) {
        return WCH_ISP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Verifying %zu bytes at 0x%08lX", size, address);

    const uint8_t* ptr = data;
    size_t remaining = size;
    uint32_t current_addr = address;
    size_t total_verified = 0;

    while (remaining > 0) {
        // Calculate chunk size (max 56 bytes)
        uint16_t chunk_size = (remaining > WCH_MAX_PROGRAM_DATA) ?
                               WCH_MAX_PROGRAM_DATA : remaining;

        // Verify chunk
        wch_isp_err_t err = wch_flash_verify_chunk(handle, current_addr,
                                                    ptr, chunk_size);
        if (err != WCH_ISP_OK) {
            ESP_LOGE(TAG, "Verify failed at 0x%08lX", current_addr);
            return err;
        }

        // Update progress
        ptr += chunk_size;
        current_addr += chunk_size;
        remaining -= chunk_size;
        total_verified += chunk_size;

        // Log progress every 1KB
        if ((total_verified % 1024) == 0 || remaining == 0) {
            ESP_LOGI(TAG, "Progress: %zu / %zu bytes (%d%%)",
                     total_verified, size, (int)(total_verified * 100 / size));
        }
    }

    // Send final zero-length verify to signal completion
    ESP_LOGD(TAG, "Sending final zero-length verify");
    wch_isp_err_t err = wch_cmd_verify(handle, current_addr, NULL, 0);
    if (err != WCH_ISP_OK) {
        ESP_LOGE(TAG, "Final verify failed");
        return err;
    }

    ESP_LOGI(TAG, "Verify complete: %zu bytes", size);

    return WCH_ISP_OK;
}
