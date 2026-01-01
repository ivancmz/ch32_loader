/**
 * @file wch_isp_protocol.h
 * @brief Internal WCH ISP protocol definitions and functions
 */

#ifndef WCH_ISP_PROTOCOL_H
#define WCH_ISP_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include "wch_isp.h"
#include "driver/uart.h"
#include "driver/gpio.h"

// Command codes
#define WCH_CMD_IDENTIFY        0xA1
#define WCH_CMD_ISP_END         0xA2
#define WCH_CMD_ISP_KEY         0xA3
#define WCH_CMD_ERASE           0xA4
#define WCH_CMD_PROGRAM         0xA5
#define WCH_CMD_VERIFY          0xA6
#define WCH_CMD_READ_CONFIG     0xA7
#define WCH_CMD_WRITE_CONFIG    0xA8

// Configuration masks for READ_CONFIG/WRITE_CONFIG
#define CFG_MASK_ALL            0x1F
#define CFG_MASK_RDPR_USER      0x07
#define CFG_MASK_DATA           0x03
#define CFG_MASK_WRPR           0x0F
#define CFG_MASK_BTVER          0x08
#define CFG_MASK_UID            0x10

// Packet constants
#define WCH_PACKET_HEADER_SEND   0x57AB
#define WCH_PACKET_HEADER_RECV   0x55AA
#define WCH_MAX_DATA_SIZE        61        // Max data payload per packet
#define WCH_MAX_PROGRAM_DATA     56        // Max for PROGRAM/VERIFY (61 - 5 address header)
#define WCH_ISP_KEY_LENGTH       30        // ISP key length

// Device constants
#define CH32V303_DEVICE_ID       0x17
#define CH32V303_DEVICE_TYPE     0x30

/**
 * @brief Internal WCH ISP handle structure
 */
struct wch_isp_handle {
    // UART configuration
    uart_port_t uart_port;
    gpio_num_t tx_pin;
    gpio_num_t rx_pin;
    uint32_t baudrate;

    // GPIO control
    gpio_num_t rst_pin;
    gpio_num_t boot0_pin;

    // Device state
    uint8_t device_type;        // Device family/type (e.g., 0x17 for CH32V30x)
    uint8_t chip_id;            // Chip variant ID (e.g., 0x33 for CH32V303CBT6)
    uint8_t uid[8];             // Unique device ID
    uint16_t bootloader_ver;    // Bootloader version
    uint8_t xor_key[8];         // XOR encryption key
    uint32_t option_bytes[3];   // Option bytes cache

    // Protocol state
    bool initialized;
    bool in_bootloader;
    uint32_t timeout_ms;

    // Buffers
    uint8_t tx_buffer[67];      // Max packet: 2+1+2+61+1=67
    uint8_t rx_buffer[67];
};

/**
 * @brief Initialize UART for ISP communication
 *
 * @param handle ISP handle
 * @return WCH_ISP_OK on success, error code otherwise
 */
wch_isp_err_t wch_uart_init(struct wch_isp_handle* handle);

/**
 * @brief Deinitialize UART
 *
 * @param handle ISP handle
 * @return WCH_ISP_OK on success, error code otherwise
 */
wch_isp_err_t wch_uart_deinit(struct wch_isp_handle* handle);

/**
 * @brief Flush UART RX buffer
 *
 * @param handle ISP handle
 * @return WCH_ISP_OK on success, error code otherwise
 */
wch_isp_err_t wch_uart_flush(struct wch_isp_handle* handle);

/**
 * @brief Send packet to device
 *
 * Packet format: [0x57 0xAB | cmd | len_low | len_high | data | checksum]
 *
 * @param handle ISP handle
 * @param cmd Command byte
 * @param data Data payload (can be NULL if len is 0)
 * @param len Data length (max 61 bytes)
 * @return WCH_ISP_OK on success, error code otherwise
 */
wch_isp_err_t wch_protocol_send_packet(struct wch_isp_handle* handle,
                                        uint8_t cmd,
                                        const uint8_t* data,
                                        uint16_t len);

/**
 * @brief Receive packet from device
 *
 * Packet format: [0x55 0xAA | cmd | 0x00 | len_low | len_high | data | checksum]
 *
 * @param handle ISP handle
 * @param expected_cmd Expected command byte
 * @param data Buffer to store received data
 * @param max_len Maximum data buffer size
 * @param actual_len Actual data length received (can be NULL)
 * @return WCH_ISP_OK on success, error code otherwise
 */
wch_isp_err_t wch_protocol_recv_packet(struct wch_isp_handle* handle,
                                        uint8_t expected_cmd,
                                        uint8_t* data,
                                        uint16_t max_len,
                                        uint16_t* actual_len);

/**
 * @brief Send IDENTIFY command
 *
 * @param handle ISP handle
 * @param device_type Pointer to store device family/type (e.g., 0x17)
 * @param chip_id Pointer to store chip variant ID (e.g., 0x33)
 * @return WCH_ISP_OK on success, error code otherwise
 */
wch_isp_err_t wch_cmd_identify(struct wch_isp_handle* handle,
                                uint8_t* device_type,
                                uint8_t* chip_id);

/**
 * @brief Send READ_CONFIG command
 *
 * @param handle ISP handle
 * @param cfg_mask Configuration mask (CFG_MASK_*)
 * @param data Buffer to store configuration data
 * @param len Expected data length
 * @return WCH_ISP_OK on success, error code otherwise
 */
wch_isp_err_t wch_cmd_read_config(struct wch_isp_handle* handle,
                                   uint8_t cfg_mask,
                                   uint8_t* data,
                                   uint16_t len);

/**
 * @brief Send WRITE_CONFIG command
 *
 * @param handle ISP handle
 * @param cfg_mask Configuration mask (CFG_MASK_*)
 * @param data Configuration data to write
 * @param len Data length
 * @return WCH_ISP_OK on success, error code otherwise
 */
wch_isp_err_t wch_cmd_write_config(struct wch_isp_handle* handle,
                                    uint8_t cfg_mask,
                                    const uint8_t* data,
                                    uint16_t len);

/**
 * @brief Send ISP_KEY command
 *
 * @param handle ISP handle
 * @param key ISP key data (typically 30 bytes)
 * @param key_len Key length
 * @return WCH_ISP_OK on success, error code otherwise
 */
wch_isp_err_t wch_cmd_isp_key(struct wch_isp_handle* handle,
                               const uint8_t* key,
                               uint16_t key_len);

/**
 * @brief Send ERASE command
 *
 * @param handle ISP handle
 * @param sector_count Number of 1KB sectors to erase
 * @return WCH_ISP_OK on success, error code otherwise
 */
wch_isp_err_t wch_cmd_erase(struct wch_isp_handle* handle,
                             uint32_t sector_count);

/**
 * @brief Send PROGRAM command
 *
 * @param handle ISP handle
 * @param address Flash address
 * @param data Data to write (must be XOR encrypted before calling)
 * @param len Data length (max 56 bytes, will be aligned to 8 bytes)
 * @return WCH_ISP_OK on success, error code otherwise
 */
wch_isp_err_t wch_cmd_program(struct wch_isp_handle* handle,
                               uint32_t address,
                               const uint8_t* data,
                               uint16_t len);

/**
 * @brief Send VERIFY command
 *
 * @param handle ISP handle
 * @param address Flash address
 * @param data Data to verify (must be XOR encrypted before calling)
 * @param len Data length (max 56 bytes, will be aligned to 8 bytes)
 * @return WCH_ISP_OK on success, error code otherwise
 */
wch_isp_err_t wch_cmd_verify(struct wch_isp_handle* handle,
                              uint32_t address,
                              const uint8_t* data,
                              uint16_t len);

/**
 * @brief Send ISP_END command
 *
 * @param handle ISP handle
 * @param reset_flag 1 to reset device, 0 to stay in bootloader
 * @return WCH_ISP_OK on success, error code otherwise
 */
wch_isp_err_t wch_cmd_isp_end(struct wch_isp_handle* handle,
                               uint8_t reset_flag);

/**
 * @brief Calculate XOR encryption key from UID
 *
 * CRITICAL: Uses specific index formula from reference implementation
 *
 * @param handle ISP handle (must have valid UID and chip_id)
 * @param isp_key ISP key data (typically 30 bytes of zeros)
 * @param key_len ISP key length
 */
void wch_calculate_xor_key(struct wch_isp_handle* handle,
                           const uint8_t* isp_key,
                           uint16_t key_len);

/**
 * @brief XOR encrypt/decrypt data in place
 *
 * @param data Data buffer to encrypt/decrypt
 * @param len Data length
 * @param xor_key XOR key (8 bytes)
 */
void wch_xor_encrypt(uint8_t* data, uint16_t len, const uint8_t* xor_key);

#endif // WCH_ISP_PROTOCOL_H
