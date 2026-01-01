/**
 * @file wch_isp_flash.h
 * @brief Internal flash operations for WCH ISP
 */

#ifndef WCH_ISP_FLASH_H
#define WCH_ISP_FLASH_H

#include <stdint.h>
#include <stddef.h>
#include "wch_isp.h"
#include "wch_isp_protocol.h"

/**
 * @brief Erase flash sectors
 *
 * @param handle ISP handle
 * @param firmware_size Firmware size in bytes (used to calculate sector count)
 * @return WCH_ISP_OK on success, error code otherwise
 */
wch_isp_err_t wch_flash_erase_sectors(struct wch_isp_handle* handle,
                                       size_t firmware_size);

/**
 * @brief Write single chunk to flash
 *
 * @param handle ISP handle
 * @param address Flash address
 * @param data Data to write (will be XOR encrypted internally)
 * @param len Data length (max 56 bytes)
 * @return WCH_ISP_OK on success, error code otherwise
 */
wch_isp_err_t wch_flash_write_chunk(struct wch_isp_handle* handle,
                                     uint32_t address,
                                     const uint8_t* data,
                                     uint16_t len);

/**
 * @brief Verify single chunk
 *
 * @param handle ISP handle
 * @param address Flash address
 * @param data Data to verify against (will be XOR encrypted internally)
 * @param len Data length (max 56 bytes)
 * @return WCH_ISP_OK on success, error code otherwise
 */
wch_isp_err_t wch_flash_verify_chunk(struct wch_isp_handle* handle,
                                      uint32_t address,
                                      const uint8_t* data,
                                      uint16_t len);

/**
 * @brief Write entire firmware to flash
 *
 * @param handle ISP handle
 * @param address Flash start address
 * @param data Firmware data
 * @param size Firmware size
 * @return WCH_ISP_OK on success, error code otherwise
 */
wch_isp_err_t wch_flash_write_all(struct wch_isp_handle* handle,
                                   uint32_t address,
                                   const uint8_t* data,
                                   size_t size);

/**
 * @brief Verify entire firmware
 *
 * @param handle ISP handle
 * @param address Flash start address
 * @param data Expected firmware data
 * @param size Firmware size
 * @return WCH_ISP_OK on success, error code otherwise
 */
wch_isp_err_t wch_flash_verify_all(struct wch_isp_handle* handle,
                                    uint32_t address,
                                    const uint8_t* data,
                                    size_t size);

#endif // WCH_ISP_FLASH_H
