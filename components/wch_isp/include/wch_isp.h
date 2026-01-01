/**
 * @file wch_isp.h
 * @brief WCH ISP (In-System Programming) API for CH32V303 microcontrollers
 *
 * This component implements the WCH ISP protocol for programming CH32V303
 * microcontrollers via UART from ESP32.
 */

#ifndef WCH_ISP_H
#define WCH_ISP_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "driver/uart.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief WCH ISP error codes
 */
typedef enum {
    WCH_ISP_OK = 0,                    /**< Operation successful */
    WCH_ISP_ERR_INVALID_ARG = -1,      /**< Invalid argument */
    WCH_ISP_ERR_NO_MEM = -2,           /**< Out of memory */
    WCH_ISP_ERR_TIMEOUT = -3,          /**< Operation timed out */
    WCH_ISP_ERR_UART = -4,             /**< UART communication error */
    WCH_ISP_ERR_PROTOCOL = -5,         /**< Protocol error (invalid packet) */
    WCH_ISP_ERR_CHECKSUM = -6,         /**< Checksum mismatch */
    WCH_ISP_ERR_ERASE = -7,            /**< Flash erase failed */
    WCH_ISP_ERR_WRITE = -8,            /**< Flash write failed */
    WCH_ISP_ERR_VERIFY = -9,           /**< Flash verify failed */
    WCH_ISP_ERR_DEVICE_LOCKED = -10,   /**< Device is read-protected */
    WCH_ISP_ERR_NOT_IN_BOOTLOADER = -11, /**< Device not in bootloader mode */
} wch_isp_err_t;

/**
 * @brief WCH ISP configuration structure
 */
typedef struct {
    uart_port_t uart_port;      /**< UART port number (UART_NUM_0, UART_NUM_1, UART_NUM_2) */
    gpio_num_t tx_pin;          /**< UART TX GPIO number */
    gpio_num_t rx_pin;          /**< UART RX GPIO number */
    uint32_t baudrate;          /**< UART baud rate (default: 115200) */
    gpio_num_t rst_pin;         /**< CH32 RST GPIO number */
    gpio_num_t boot0_pin;       /**< CH32 BOOT0 GPIO number */
    uint32_t timeout_ms;        /**< Command timeout in milliseconds (default: 1000) */
} wch_isp_config_t;

/**
 * @brief WCH device information structure
 */
typedef struct {
    uint8_t device_type;        /**< Device family/type (e.g., 0x17 for CH32V30x family) */
    uint8_t chip_id;            /**< Chip variant ID (e.g., 0x33 for CH32V303CBT6) */
    uint8_t uid[8];             /**< Unique device ID */
    uint16_t bootloader_ver;    /**< Bootloader version */
    uint32_t flash_size;        /**< Flash size in bytes */
    char device_name[32];       /**< Device name string */
} wch_isp_device_info_t;

/**
 * @brief WCH ISP handle (opaque pointer)
 */
typedef struct wch_isp_handle* wch_isp_handle_t;

/**
 * @brief Initialize WCH ISP component
 *
 * @param config Configuration structure
 * @param handle Pointer to store the ISP handle
 * @return WCH_ISP_OK on success, error code otherwise
 */
wch_isp_err_t wch_isp_init(const wch_isp_config_t* config, wch_isp_handle_t* handle);

/**
 * @brief Deinitialize WCH ISP component and free resources
 *
 * @param handle ISP handle
 * @return WCH_ISP_OK on success, error code otherwise
 */
wch_isp_err_t wch_isp_deinit(wch_isp_handle_t handle);

/**
 * @brief Enter bootloader mode
 *
 * Asserts BOOT0, pulses RST, and verifies bootloader response
 *
 * @param handle ISP handle
 * @return WCH_ISP_OK on success, error code otherwise
 */
wch_isp_err_t wch_isp_enter_bootloader(wch_isp_handle_t handle);

/**
 * @brief Exit bootloader mode and reset to application
 *
 * @param handle ISP handle
 * @param reset If true, reset the device after exiting bootloader
 * @return WCH_ISP_OK on success, error code otherwise
 */
wch_isp_err_t wch_isp_exit_bootloader(wch_isp_handle_t handle, bool reset);

/**
 * @brief Get device information
 *
 * Must be called after wch_isp_enter_bootloader()
 *
 * @param handle ISP handle
 * @param info Pointer to device info structure to fill
 * @return WCH_ISP_OK on success, error code otherwise
 */
wch_isp_err_t wch_isp_get_device_info(wch_isp_handle_t handle, wch_isp_device_info_t* info);

/**
 * @brief Erase flash memory
 *
 * @param handle ISP handle
 * @param size Size of firmware to erase (used to calculate sector count)
 * @return WCH_ISP_OK on success, error code otherwise
 */
wch_isp_err_t wch_isp_erase_flash(wch_isp_handle_t handle, size_t size);

/**
 * @brief Flash firmware to device
 *
 * Performs complete programming sequence: identify, read config, setup key,
 * erase, write firmware
 *
 * @param handle ISP handle
 * @param data Firmware data buffer
 * @param size Firmware size in bytes
 * @param address Flash start address (usually 0x08000000)
 * @return WCH_ISP_OK on success, error code otherwise
 */
wch_isp_err_t wch_isp_flash_firmware(wch_isp_handle_t handle,
                                      const uint8_t* data,
                                      size_t size,
                                      uint32_t address);

/**
 * @brief Verify flash contents
 *
 * @param handle ISP handle
 * @param data Expected firmware data buffer
 * @param size Data size in bytes
 * @param address Flash start address (usually 0x08000000)
 * @return WCH_ISP_OK if flash matches data, error code otherwise
 */
wch_isp_err_t wch_isp_verify_firmware(wch_isp_handle_t handle,
                                       const uint8_t* data,
                                       size_t size,
                                       uint32_t address);

/**
 * @brief Read option bytes
 *
 * @param handle ISP handle
 * @param option_bytes Buffer to store option bytes (3x uint32_t)
 * @param count Number of 32-bit words to read (typically 3)
 * @return WCH_ISP_OK on success, error code otherwise
 */
wch_isp_err_t wch_isp_read_option_bytes(wch_isp_handle_t handle,
                                         uint32_t* option_bytes,
                                         size_t count);

/**
 * @brief Write option bytes
 *
 * WARNING: Writing incorrect values can brick the device!
 *
 * @param handle ISP handle
 * @param option_bytes Option bytes to write (3x uint32_t)
 * @param count Number of 32-bit words to write (typically 3)
 * @return WCH_ISP_OK on success, error code otherwise
 */
wch_isp_err_t wch_isp_write_option_bytes(wch_isp_handle_t handle,
                                          const uint32_t* option_bytes,
                                          size_t count);

/**
 * @brief Unlock device (disable read protection)
 *
 * WARNING: This will erase the flash!
 *
 * @param handle ISP handle
 * @return WCH_ISP_OK on success, error code otherwise
 */
wch_isp_err_t wch_isp_unlock_device(wch_isp_handle_t handle);

/**
 * @brief Convert error code to string
 *
 * @param err Error code
 * @return Human-readable error string
 */
const char* wch_isp_err_to_string(wch_isp_err_t err);

#ifdef __cplusplus
}
#endif

#endif // WCH_ISP_H
