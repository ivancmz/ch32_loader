/**
 * @file wch_isp_protocol.c
 * @brief WCH ISP protocol implementation
 */

#include <string.h>
#include "wch_isp_protocol.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "wch_isp_protocol";

// Magic string for IDENTIFY command
static const char IDENTIFY_MAGIC[] = "\x00\x00MCU ISP & WCH.CN";
#define IDENTIFY_MAGIC_LEN 18

wch_isp_err_t wch_uart_init(struct wch_isp_handle* handle)
{
    if (!handle) {
        return WCH_ISP_ERR_INVALID_ARG;
    }

    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = handle->baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_param_config(handle->uart_port, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed: %d", err);
        return WCH_ISP_ERR_UART;
    }

    err = uart_set_pin(handle->uart_port, handle->tx_pin, handle->rx_pin,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed: %d", err);
        return WCH_ISP_ERR_UART;
    }

    // Install UART driver
    err = uart_driver_install(handle->uart_port, 256, 256, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %d", err);
        return WCH_ISP_ERR_UART;
    }

    ESP_LOGI(TAG, "UART initialized: port=%d, baud=%lu, TX=%d, RX=%d",
             handle->uart_port, handle->baudrate, handle->tx_pin, handle->rx_pin);

    return WCH_ISP_OK;
}

wch_isp_err_t wch_uart_deinit(struct wch_isp_handle* handle)
{
    if (!handle) {
        return WCH_ISP_ERR_INVALID_ARG;
    }

    esp_err_t err = uart_driver_delete(handle->uart_port);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART driver delete failed: %d", err);
        return WCH_ISP_ERR_UART;
    }

    return WCH_ISP_OK;
}

wch_isp_err_t wch_uart_flush(struct wch_isp_handle* handle)
{
    if (!handle) {
        return WCH_ISP_ERR_INVALID_ARG;
    }

    esp_err_t err = uart_flush_input(handle->uart_port);
    if (err != ESP_OK) {
        return WCH_ISP_ERR_UART;
    }

    return WCH_ISP_OK;
}

wch_isp_err_t wch_protocol_send_packet(struct wch_isp_handle* handle,
                                        uint8_t cmd,
                                        const uint8_t* data,
                                        uint16_t len)
{
    if (!handle) {
        return WCH_ISP_ERR_INVALID_ARG;
    }

    if (len > WCH_MAX_DATA_SIZE) {
        ESP_LOGE(TAG, "Data too large: %d > %d", len, WCH_MAX_DATA_SIZE);
        return WCH_ISP_ERR_INVALID_ARG;
    }

    uint8_t* buf = handle->tx_buffer;

    // Build packet: [0x57 0xAB | cmd | len_low | len_high | data | checksum]
    buf[0] = 0x57;
    buf[1] = 0xAB;
    buf[2] = cmd;
    buf[3] = (len >> 0) & 0xFF;
    buf[4] = (len >> 8) & 0xFF;

    // Copy data
    if (len > 0 && data != NULL) {
        memcpy(&buf[5], data, len);
    }

    // Calculate checksum (sum from cmd through data)
    uint8_t checksum = 0;
    for (int i = 2; i < (len + 5); i++) {
        checksum += buf[i];
    }
    buf[len + 5] = checksum;

    // Send packet
    int written = uart_write_bytes(handle->uart_port, buf, len + 6);
    if (written != (len + 6)) {
        ESP_LOGE(TAG, "UART write failed: wrote %d, expected %d", written, len + 6);
        return WCH_ISP_ERR_UART;
    }

    ESP_LOGD(TAG, "Sent packet: cmd=0x%02X, len=%d", cmd, len);

    return WCH_ISP_OK;
}

wch_isp_err_t wch_protocol_recv_packet(struct wch_isp_handle* handle,
                                        uint8_t expected_cmd,
                                        uint8_t* data,
                                        uint16_t max_len,
                                        uint16_t* actual_len)
{
    if (!handle) {
        return WCH_ISP_ERR_INVALID_ARG;
    }

    uint8_t* buf = handle->rx_buffer;

    // Read header (6 bytes): [0x55 0xAA | cmd | 0x00 | len_low | len_high]
    int read_len = uart_read_bytes(handle->uart_port, buf, 6,
                                    pdMS_TO_TICKS(handle->timeout_ms));
    if (read_len != 6) {
        ESP_LOGE(TAG, "Timeout reading header: got %d bytes", read_len);
        return WCH_ISP_ERR_TIMEOUT;
    }

    // Validate header
    if (buf[0] != 0x55 || buf[1] != 0xAA) {
        ESP_LOGE(TAG, "Invalid packet header: 0x%02X 0x%02X", buf[0], buf[1]);
        return WCH_ISP_ERR_PROTOCOL;
    }

    if (buf[2] != expected_cmd) {
        ESP_LOGE(TAG, "Unexpected command: got 0x%02X, expected 0x%02X",
                 buf[2], expected_cmd);
        return WCH_ISP_ERR_PROTOCOL;
    }

    // Get data length
    uint16_t data_len = buf[4] | ((uint16_t)buf[5] << 8);
    if (data_len > 60) {
        ESP_LOGE(TAG, "Invalid data length: %d", data_len);
        return WCH_ISP_ERR_PROTOCOL;
    }

    // Read data + checksum
    read_len = uart_read_bytes(handle->uart_port, &buf[6], data_len + 1,
                               pdMS_TO_TICKS(handle->timeout_ms));
    if (read_len != (data_len + 1)) {
        ESP_LOGE(TAG, "Timeout reading data: got %d bytes, expected %d",
                 read_len, data_len + 1);
        return WCH_ISP_ERR_TIMEOUT;
    }

    // Verify checksum
    uint8_t checksum = 0;
    for (int i = 2; i < (data_len + 6); i++) {
        checksum += buf[i];
    }
    if (buf[data_len + 6] != checksum) {
        ESP_LOGE(TAG, "Checksum mismatch: got 0x%02X, calculated 0x%02X",
                 buf[data_len + 6], checksum);
        return WCH_ISP_ERR_CHECKSUM;
    }

    // Copy data to output
    if (data != NULL && max_len > 0) {
        uint16_t copy_len = (data_len < max_len) ? data_len : max_len;
        memcpy(data, &buf[6], copy_len);
    }

    if (actual_len != NULL) {
        *actual_len = data_len;
    }

    ESP_LOGD(TAG, "Received packet: cmd=0x%02X, len=%d", expected_cmd, data_len);

    return WCH_ISP_OK;
}

wch_isp_err_t wch_cmd_identify(struct wch_isp_handle* handle,
                                uint8_t* device_type,
                                uint8_t* chip_id)
{
    if (!handle) {
        return WCH_ISP_ERR_INVALID_ARG;
    }

    // Send IDENTIFY command with magic string
    wch_isp_err_t err = wch_protocol_send_packet(handle, WCH_CMD_IDENTIFY,
                                                  (const uint8_t*)IDENTIFY_MAGIC,
                                                  IDENTIFY_MAGIC_LEN);
    if (err != WCH_ISP_OK) {
        return err;
    }

    // Receive response (2 bytes)
    uint8_t resp[2];
    uint16_t resp_len;
    err = wch_protocol_recv_packet(handle, WCH_CMD_IDENTIFY,
                                   resp, sizeof(resp), &resp_len);
    if (err != WCH_ISP_OK) {
        return err;
    }

    if (resp_len < 2) {
        ESP_LOGE(TAG, "IDENTIFY response too short: %d bytes", resp_len);
        return WCH_ISP_ERR_PROTOCOL;
    }

    // Response format: [chip_id/variant, device_type/family]
    // resp[0] = chip variant (0x30, 0x33, 0x50, 0x70, etc.)
    // resp[1] = device family (0x17 for all CH32V30x)
    if (chip_id != NULL) {
        *chip_id = resp[0];  // Chip variant
    }
    if (device_type != NULL) {
        *device_type = resp[1];  // Device family
    }

    ESP_LOGI(TAG, "Device identified: family=0x%02X, variant=0x%02X", resp[1], resp[0]);

    return WCH_ISP_OK;
}

wch_isp_err_t wch_cmd_read_config(struct wch_isp_handle* handle,
                                   uint8_t cfg_mask,
                                   uint8_t* data,
                                   uint16_t len)
{
    if (!handle || !data) {
        return WCH_ISP_ERR_INVALID_ARG;
    }

    // Send READ_CONFIG command with mask (2 bytes, little-endian)
    uint8_t mask_buf[2];
    mask_buf[0] = cfg_mask & 0xFF;
    mask_buf[1] = (cfg_mask >> 8) & 0xFF;

    wch_isp_err_t err = wch_protocol_send_packet(handle, WCH_CMD_READ_CONFIG,
                                                  mask_buf, 2);
    if (err != WCH_ISP_OK) {
        return err;
    }

    // Receive response into temporary buffer
    // Response format: [mask_echo(2B) | actual_data]
    uint8_t temp_buf[32];
    uint16_t resp_len;
    err = wch_protocol_recv_packet(handle, WCH_CMD_READ_CONFIG,
                                   temp_buf, sizeof(temp_buf), &resp_len);
    if (err != WCH_ISP_OK) {
        return err;
    }

    // Skip the first 2 bytes (mask echo) and copy actual data
    if (resp_len < 2) {
        ESP_LOGE(TAG, "Response too short: %d bytes", resp_len);
        return WCH_ISP_ERR_PROTOCOL;
    }

    uint16_t data_len = resp_len - 2;
    if (data_len > len) {
        ESP_LOGW(TAG, "Response data truncated: %d > %d", data_len, len);
        data_len = len;
    }

    memcpy(data, &temp_buf[2], data_len);

    ESP_LOGD(TAG, "Read config: mask=0x%02X, resp_len=%d, data_len=%d", cfg_mask, resp_len, data_len);

    return WCH_ISP_OK;
}

wch_isp_err_t wch_cmd_write_config(struct wch_isp_handle* handle,
                                    uint8_t cfg_mask,
                                    const uint8_t* data,
                                    uint16_t len)
{
    if (!handle || !data) {
        return WCH_ISP_ERR_INVALID_ARG;
    }

    // Prepare packet: mask (2 bytes, little-endian) + data
    uint8_t buffer[32];
    buffer[0] = cfg_mask & 0xFF;
    buffer[1] = (cfg_mask >> 8) & 0xFF;
    memcpy(&buffer[2], data, len);

    // Send WRITE_CONFIG command
    wch_isp_err_t err = wch_protocol_send_packet(handle, WCH_CMD_WRITE_CONFIG,
                                                  buffer, len + 2);
    if (err != WCH_ISP_OK) {
        return err;
    }

    // Receive response
    uint8_t resp[2];
    uint16_t resp_len;
    err = wch_protocol_recv_packet(handle, WCH_CMD_WRITE_CONFIG,
                                   resp, sizeof(resp), &resp_len);
    if (err != WCH_ISP_OK) {
        return err;
    }

    // Check status (0x00 0x00 = success)
    if (resp_len >= 2 && (resp[0] != 0 || resp[1] != 0)) {
        ESP_LOGE(TAG, "WRITE_CONFIG failed: status=0x%02X 0x%02X", resp[0], resp[1]);
        return WCH_ISP_ERR_PROTOCOL;
    }

    ESP_LOGD(TAG, "Write config: mask=0x%02X, len=%d", cfg_mask, len);

    return WCH_ISP_OK;
}

wch_isp_err_t wch_cmd_isp_key(struct wch_isp_handle* handle,
                               const uint8_t* key,
                               uint16_t key_len)
{
    if (!handle || !key) {
        return WCH_ISP_ERR_INVALID_ARG;
    }

    // Send ISP_KEY command
    wch_isp_err_t err = wch_protocol_send_packet(handle, WCH_CMD_ISP_KEY,
                                                  key, key_len);
    if (err != WCH_ISP_OK) {
        return err;
    }

    // Receive response
    uint8_t resp[2];
    uint16_t resp_len;
    err = wch_protocol_recv_packet(handle, WCH_CMD_ISP_KEY,
                                   resp, sizeof(resp), &resp_len);
    if (err != WCH_ISP_OK) {
        return err;
    }

    ESP_LOGD(TAG, "ISP key set: len=%d", key_len);

    return WCH_ISP_OK;
}

wch_isp_err_t wch_cmd_erase(struct wch_isp_handle* handle,
                             uint32_t sector_count)
{
    if (!handle) {
        return WCH_ISP_ERR_INVALID_ARG;
    }

    // Prepare sector count (32-bit little-endian)
    uint8_t buffer[4];
    buffer[0] = (sector_count >> 0) & 0xFF;
    buffer[1] = (sector_count >> 8) & 0xFF;
    buffer[2] = (sector_count >> 16) & 0xFF;
    buffer[3] = (sector_count >> 24) & 0xFF;

    // Send ERASE command
    wch_isp_err_t err = wch_protocol_send_packet(handle, WCH_CMD_ERASE,
                                                  buffer, 4);
    if (err != WCH_ISP_OK) {
        return err;
    }

    // Receive response (use extended timeout for erase)
    uint32_t saved_timeout = handle->timeout_ms;
    handle->timeout_ms = 5000;  // Erase can take a while

    uint8_t resp[2];
    uint16_t resp_len;
    err = wch_protocol_recv_packet(handle, WCH_CMD_ERASE,
                                   resp, sizeof(resp), &resp_len);

    handle->timeout_ms = saved_timeout;  // Restore timeout

    if (err != WCH_ISP_OK) {
        return err;
    }

    // Check status
    if (resp_len >= 2 && (resp[0] != 0 || resp[1] != 0)) {
        ESP_LOGE(TAG, "ERASE failed: status=0x%02X 0x%02X", resp[0], resp[1]);
        return WCH_ISP_ERR_ERASE;
    }

    ESP_LOGI(TAG, "Erased %lu sectors", sector_count);

    return WCH_ISP_OK;
}

wch_isp_err_t wch_cmd_program(struct wch_isp_handle* handle,
                               uint32_t address,
                               const uint8_t* data,
                               uint16_t len)
{
    if (!handle) {
        return WCH_ISP_ERR_INVALID_ARG;
    }

    if (len > WCH_MAX_PROGRAM_DATA) {
        ESP_LOGE(TAG, "Program data too large: %d > %d", len, WCH_MAX_PROGRAM_DATA);
        return WCH_ISP_ERR_INVALID_ARG;
    }

    // Prepare packet: [address(4B) | padding(1B) | data]
    uint8_t buffer[61];
    buffer[0] = (address >> 0) & 0xFF;
    buffer[1] = (address >> 8) & 0xFF;
    buffer[2] = (address >> 16) & 0xFF;
    buffer[3] = (address >> 24) & 0xFF;
    buffer[4] = 0;  // Padding

    uint16_t packet_len = 5;
    if (len > 0 && data != NULL) {
        memcpy(&buffer[5], data, len);
        packet_len += len;
    }

    // Send PROGRAM command
    wch_isp_err_t err = wch_protocol_send_packet(handle, WCH_CMD_PROGRAM,
                                                  buffer, packet_len);
    if (err != WCH_ISP_OK) {
        return err;
    }

    // Receive response
    uint8_t resp[2];
    uint16_t resp_len;
    err = wch_protocol_recv_packet(handle, WCH_CMD_PROGRAM,
                                   resp, sizeof(resp), &resp_len);
    if (err != WCH_ISP_OK) {
        return err;
    }

    // Check status
    if (resp_len >= 2 && (resp[0] != 0 || resp[1] != 0)) {
        ESP_LOGE(TAG, "PROGRAM failed at 0x%08lX: status=0x%02X 0x%02X",
                 address, resp[0], resp[1]);
        return WCH_ISP_ERR_WRITE;
    }

    return WCH_ISP_OK;
}

wch_isp_err_t wch_cmd_verify(struct wch_isp_handle* handle,
                              uint32_t address,
                              const uint8_t* data,
                              uint16_t len)
{
    if (!handle) {
        return WCH_ISP_ERR_INVALID_ARG;
    }

    if (len > WCH_MAX_PROGRAM_DATA) {
        ESP_LOGE(TAG, "Verify data too large: %d > %d", len, WCH_MAX_PROGRAM_DATA);
        return WCH_ISP_ERR_INVALID_ARG;
    }

    // Prepare packet: [address(4B) | padding(1B) | data]
    uint8_t buffer[61];
    buffer[0] = (address >> 0) & 0xFF;
    buffer[1] = (address >> 8) & 0xFF;
    buffer[2] = (address >> 16) & 0xFF;
    buffer[3] = (address >> 24) & 0xFF;
    buffer[4] = 0;  // Padding

    uint16_t packet_len = 5;
    if (len > 0 && data != NULL) {
        memcpy(&buffer[5], data, len);
        packet_len += len;
    }

    // Send VERIFY command
    wch_isp_err_t err = wch_protocol_send_packet(handle, WCH_CMD_VERIFY,
                                                  buffer, packet_len);
    if (err != WCH_ISP_OK) {
        return err;
    }

    // Receive response
    uint8_t resp[2];
    uint16_t resp_len;
    err = wch_protocol_recv_packet(handle, WCH_CMD_VERIFY,
                                   resp, sizeof(resp), &resp_len);
    if (err != WCH_ISP_OK) {
        return err;
    }

    // Check status
    if (resp_len >= 2 && (resp[0] != 0 || resp[1] != 0)) {
        ESP_LOGE(TAG, "VERIFY failed at 0x%08lX: status=0x%02X 0x%02X",
                 address, resp[0], resp[1]);
        return WCH_ISP_ERR_VERIFY;
    }

    return WCH_ISP_OK;
}

wch_isp_err_t wch_cmd_isp_end(struct wch_isp_handle* handle,
                               uint8_t reset_flag)
{
    if (!handle) {
        return WCH_ISP_ERR_INVALID_ARG;
    }

    // Send ISP_END command
    wch_isp_err_t err = wch_protocol_send_packet(handle, WCH_CMD_ISP_END,
                                                  &reset_flag, 1);
    if (err != WCH_ISP_OK) {
        return err;
    }

    // Receive response
    uint8_t resp[2];
    uint16_t resp_len;
    err = wch_protocol_recv_packet(handle, WCH_CMD_ISP_END,
                                   resp, sizeof(resp), &resp_len);
    if (err != WCH_ISP_OK) {
        return err;
    }

    ESP_LOGI(TAG, "ISP session ended");

    return WCH_ISP_OK;
}

void wch_calculate_xor_key(struct wch_isp_handle* handle,
                           const uint8_t* isp_key,
                           uint16_t key_len)
{
    if (!handle || !isp_key) {
        return;
    }

    // Calculate UID sum
    uint8_t uid_sum = 0;
    for (int i = 0; i < 8; i++) {
        uid_sum += handle->uid[i];
    }

    // CRITICAL: XOR key calculation using specific index formula
    // Formula: xor_key[i] = uid_sum ^ isp_key[key_len / divisor * multiplier]
    // For key_len=30:
    //   xor_key[0] = uid_sum ^ isp_key[30/7*4] = uid_sum ^ isp_key[16]
    //   xor_key[1] = uid_sum ^ isp_key[30/5*1] = uid_sum ^ isp_key[6]
    //   etc.
    handle->xor_key[0] = uid_sum ^ isp_key[key_len / 7 * 4];  // [16]
    handle->xor_key[1] = uid_sum ^ isp_key[key_len / 5 * 1];  // [6]
    handle->xor_key[2] = uid_sum ^ isp_key[key_len / 7 * 1];  // [4]
    handle->xor_key[3] = uid_sum ^ isp_key[key_len / 7 * 6];  // [25]
    handle->xor_key[4] = uid_sum ^ isp_key[key_len / 7 * 3];  // [12]
    handle->xor_key[5] = uid_sum ^ isp_key[key_len / 5 * 3];  // [18]
    handle->xor_key[6] = uid_sum ^ isp_key[key_len / 7 * 5];  // [21]
    handle->xor_key[7] = handle->chip_id + handle->xor_key[0];  // Uses chip variant!

    ESP_LOGD(TAG, "XOR key calculated: uid_sum=0x%02X, chip_variant=0x%02X",
             uid_sum, handle->chip_id);
}

void wch_xor_encrypt(uint8_t* data, uint16_t len, const uint8_t* xor_key)
{
    if (!data || !xor_key) {
        return;
    }

    for (uint16_t i = 0; i < len; i++) {
        data[i] ^= xor_key[i % 8];
    }
}
