#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_event.h"
#include "esp_console.h"
#include "nvs_flash.h"
#include "console_simple_init.h"
#include "wch_isp.h"


// Tag for logging
static const char *TAG = "CH32_LOADER";

// CH32 firmware address
#define CH32_FLASH_ADDRESS  0x08000000

// UART configuration for CH32 communication
#define CH32_UART_PORT      UART_NUM_2
#define CH32_UART_TX_PIN    GPIO_NUM_19
#define CH32_UART_RX_PIN    GPIO_NUM_18
#define CH32_UART_BAUDRATE  115200
#define CH32_RST_PIN        GPIO_NUM_5
#define CH32_BOOT0_PIN      GPIO_NUM_4

// Buffer for serial passthrough
#define UART_RX_BUF_SIZE    256

// Define the function prototypes for commands
int ch32_info_cmd(int argc, char **argv);       //get mcu info
int ch32_verify_cmd(int argc, char **argv);     //verify flash
int ch32_flash_cmd(int argc, char **argv);      //flash firmware
int ch32_rdp_cmd(int argc, char **argv);        //set read protect
int ch32_reset_cmd(int argc, char **argv);      //reset CH32
int ch32_passthrough_cmd(int argc, char **argv); //serial passthrough

/**
 * @brief Create WCH ISP handle with configuration
 * Configuration can be modified here before creating the handle
 */
static wch_isp_handle_t create_isp_handle(void)
{
    wch_isp_config_t config = {
        .uart_port = CH32_UART_PORT,
        .tx_pin = CH32_UART_TX_PIN,
        .rx_pin = CH32_UART_RX_PIN,
        .baudrate = CH32_UART_BAUDRATE,
        .rst_pin = CH32_RST_PIN,
        .boot0_pin = CH32_BOOT0_PIN,
        .timeout_ms = 1000,
    };

    wch_isp_handle_t handle;
    wch_isp_err_t err = wch_isp_init(&config, &handle);
    if (err != WCH_ISP_OK) {
        ESP_LOGE(TAG, "Failed to initialize WCH ISP: %s", wch_isp_err_to_string(err));
        return NULL;
    }

    return handle;
}

// Global flag to track passthrough state
static bool passthrough_active = false;

/**
 * @brief Initialize UART for CH32 serial passthrough
 */
static void init_ch32_uart_passthrough(void)
{
    if (passthrough_active) {
        return;  // Already initialized
    }

    uart_config_t uart_config = {
        .baud_rate = CH32_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(CH32_UART_PORT, UART_RX_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(CH32_UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(CH32_UART_PORT, CH32_UART_TX_PIN, CH32_UART_RX_PIN,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    passthrough_active = true;
    ESP_LOGI(TAG, "CH32 UART passthrough enabled");
}

/**
 * @brief Stop UART passthrough (needed before ISP operations)
 */
static void stop_ch32_uart_passthrough(void)
{
    if (!passthrough_active) {
        return;  // Already stopped
    }

    uart_driver_delete(CH32_UART_PORT);
    passthrough_active = false;
    ESP_LOGI(TAG, "CH32 UART passthrough disabled");
}

/**
 * @brief Command: Get CH32 device information
 */
int ch32_info_cmd(int argc, char **argv)
{
    printf("Getting CH32 device info...\n");

    // Initialize ISP
    wch_isp_handle_t handle = create_isp_handle();
    if (!handle) {
        printf("Error: Failed to initialize WCH ISP\n");
        return 1;
    }

    // Enter bootloader
    wch_isp_err_t err = wch_isp_enter_bootloader(handle);
    if (err != WCH_ISP_OK) {
        printf("Error: Failed to enter bootloader: %s\n", wch_isp_err_to_string(err));
        wch_isp_deinit(handle);
        return 1;
    }

    // Get device info
    wch_isp_device_info_t info;
    err = wch_isp_get_device_info(handle, &info);
    if (err != WCH_ISP_OK) {
        printf("Error: Failed to get device info: %s\n", wch_isp_err_to_string(err));
        wch_isp_exit_bootloader(handle, true);
        wch_isp_deinit(handle);
        return 1;
    }

    // Display info
    printf("\n=== CH32 Device Information ===\n");
    printf("Chip:        %s\n", info.device_name);
    printf("Chip ID:     0x%02X (Family: 0x%02X)\n", info.chip_id, info.device_type);
    printf("Flash:       %lu KB\n", info.flash_size / 1024);
    printf("Bootloader:  v%04X\n", info.bootloader_ver);
    printf("UID:         %02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X\n",
           info.uid[0], info.uid[1], info.uid[2], info.uid[3],
           info.uid[4], info.uid[5], info.uid[6], info.uid[7]);
    printf("===============================\n\n");

    // Exit bootloader
    wch_isp_exit_bootloader(handle, true);
    wch_isp_deinit(handle);

    return 0;
}

/**
 * @brief Command: Flash CH32 firmware
 */
int ch32_flash_cmd(int argc, char **argv)
{
    printf("Flashing CH32 firmware...\n");

    // Access embedded firmware
    extern const uint8_t ch32_firmware_start[] asm("_binary_ch32_firmware_bin_start");
    extern const uint8_t ch32_firmware_end[] asm("_binary_ch32_firmware_bin_end");
    size_t fw_size = ch32_firmware_end - ch32_firmware_start;

    printf("Embedded firmware size: %zu bytes\n", fw_size);

    // Initialize ISP
    wch_isp_handle_t handle = create_isp_handle();
    if (!handle) {
        printf("Error: Failed to initialize WCH ISP\n");
        return 1;
    }

    // Enter bootloader
    wch_isp_err_t err = wch_isp_enter_bootloader(handle);
    if (err != WCH_ISP_OK) {
        printf("Error: Failed to enter bootloader: %s\n", wch_isp_err_to_string(err));
        wch_isp_deinit(handle);
        return 1;
    }

    // Flash firmware
    printf("Flashing %zu bytes...\n", fw_size);
    err = wch_isp_flash_firmware(handle, ch32_firmware_start, fw_size, CH32_FLASH_ADDRESS);
    if (err != WCH_ISP_OK) {
        printf("Error: Flash failed: %s\n", wch_isp_err_to_string(err));
        wch_isp_exit_bootloader(handle, true);
        wch_isp_deinit(handle);
        return 1;
    }

    printf("Flash complete!\n");

    // Verify
    printf("Verifying...\n");
    err = wch_isp_verify_firmware(handle, ch32_firmware_start, fw_size, CH32_FLASH_ADDRESS);
    if (err != WCH_ISP_OK) {
        printf("Error: Verify failed: %s\n", wch_isp_err_to_string(err));
        wch_isp_exit_bootloader(handle, true);
        wch_isp_deinit(handle);
        return 1;
    }

    printf("Verify complete!\n");

    // Exit bootloader and reset
    wch_isp_exit_bootloader(handle, true);
    wch_isp_deinit(handle);

    printf("Programming successful!\n");

    return 0;
}

/**
 * @brief Command: Verify CH32 flash contents
 */
int ch32_verify_cmd(int argc, char **argv)
{
    printf("Verifying CH32 flash...\n");

    // Access embedded firmware
    extern const uint8_t ch32_firmware_start[] asm("_binary_ch32_firmware_bin_start");
    extern const uint8_t ch32_firmware_end[] asm("_binary_ch32_firmware_bin_end");
    size_t fw_size = ch32_firmware_end - ch32_firmware_start;

    printf("Embedded firmware size: %zu bytes\n", fw_size);

    // Initialize ISP
    wch_isp_handle_t handle = create_isp_handle();
    if (!handle) {
        printf("Error: Failed to initialize WCH ISP\n");
        return 1;
    }

    // Enter bootloader
    wch_isp_err_t err = wch_isp_enter_bootloader(handle);
    if (err != WCH_ISP_OK) {
        printf("Error: Failed to enter bootloader: %s\n", wch_isp_err_to_string(err));
        wch_isp_deinit(handle);
        return 1;
    }

    // Verify firmware
    printf("Verifying %zu bytes...\n", fw_size);
    err = wch_isp_verify_firmware(handle, ch32_firmware_start, fw_size, CH32_FLASH_ADDRESS);
    if (err != WCH_ISP_OK) {
        printf("Error: Verify failed: %s\n", wch_isp_err_to_string(err));
        wch_isp_exit_bootloader(handle, true);
        wch_isp_deinit(handle);
        return 1;
    }

    printf("Verify successful!\n");

    // Exit bootloader and reset
    wch_isp_exit_bootloader(handle, true);
    wch_isp_deinit(handle);

    return 0;
}

/**
 * @brief Command: Enable/disable CH32 read protection
 */
int ch32_rdp_cmd(int argc, char **argv)
{
    if (argc < 2) {
        printf("Usage: rdp <lock|unlock>\n");
        printf("  lock   - Enable read protection (WARNING: Can only be unlocked by erasing flash!)\n");
        printf("  unlock - Disable read protection (WARNING: Will erase flash!)\n");
        return 1;
    }

    bool lock = false;
    if (strcmp(argv[1], "lock") == 0) {
        lock = true;
    } else if (strcmp(argv[1], "unlock") == 0) {
        lock = false;
    } else {
        printf("Error: Invalid argument '%s'. Use 'lock' or 'unlock'\n", argv[1]);
        return 1;
    }

    // Confirm with user
    if (lock) {
        printf("WARNING: This will enable read protection!\n");
        printf("The device can only be unlocked by erasing all flash memory.\n");
    } else {
        printf("WARNING: This will ERASE ALL FLASH MEMORY!\n");
        printf("All firmware on the CH32 will be lost.\n");
    }
    printf("Type 'yes' to confirm: ");
    fflush(stdout);

    char confirm[10];
    if (fgets(confirm, sizeof(confirm), stdin) == NULL || strcmp(confirm, "yes\n") != 0) {
        printf("Operation cancelled\n");
        return 0;
    }

    // Initialize ISP
    wch_isp_handle_t handle = create_isp_handle();
    if (!handle) {
        printf("Error: Failed to initialize WCH ISP\n");
        return 1;
    }

    // Enter bootloader
    wch_isp_err_t err = wch_isp_enter_bootloader(handle);
    if (err != WCH_ISP_OK) {
        printf("Error: Failed to enter bootloader: %s\n", wch_isp_err_to_string(err));
        wch_isp_deinit(handle);
        return 1;
    }

    if (lock) {
        // Read current option bytes
        uint32_t option_bytes[3];
        err = wch_isp_read_option_bytes(handle, option_bytes, 3);
        if (err != WCH_ISP_OK) {
            printf("Error: Failed to read option bytes: %s\n", wch_isp_err_to_string(err));
            wch_isp_exit_bootloader(handle, true);
            wch_isp_deinit(handle);
            return 1;
        }

        // Set RDPR to non-0xA5 value (locked)
        // Word 0: RDPR | ~RDPR | USER | ~USER
        uint8_t rdpr = 0x00;  // Any value != 0xA5 means locked
        uint8_t nrdpr = ~rdpr;
        uint8_t user = (option_bytes[0] >> 16) & 0xFF;
        uint8_t nuser = ~user;

        option_bytes[0] = rdpr | ((uint32_t)nrdpr << 8) |
                          ((uint32_t)user << 16) | ((uint32_t)nuser << 24);

        // Write option bytes
        err = wch_isp_write_option_bytes(handle, option_bytes, 3);
        if (err != WCH_ISP_OK) {
            printf("Error: Failed to lock device: %s\n", wch_isp_err_to_string(err));
            wch_isp_exit_bootloader(handle, true);
            wch_isp_deinit(handle);
            return 1;
        }

        printf("Device locked successfully\n");
    } else {
        // Unlock device
        err = wch_isp_unlock_device(handle);
        if (err != WCH_ISP_OK) {
            printf("Error: Failed to unlock device: %s\n", wch_isp_err_to_string(err));
            wch_isp_exit_bootloader(handle, true);
            wch_isp_deinit(handle);
            return 1;
        }

        printf("Device unlocked successfully (flash erased)\n");
    }

    // Exit bootloader
    wch_isp_exit_bootloader(handle, true);
    wch_isp_deinit(handle);

    return 0;
}

/**
 * @brief Command: Reset CH32 device
 */
int ch32_reset_cmd(int argc, char **argv)
{
    printf("Resetting CH32 device...\n");

    // Configure RST pin as output if not already configured
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << CH32_RST_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    // Pulse RST low for 50ms
    gpio_set_level(CH32_RST_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(CH32_RST_PIN, 1);

    printf("CH32 reset complete\n");

    return 0;
}

/**
 * @brief Command: Serial passthrough mode
 * Forwards CH32 serial output to console until Enter is pressed
 */
int ch32_passthrough_cmd(int argc, char **argv)
{
    printf("Entering serial passthrough mode...\n");
    printf("Press ENTER to exit passthrough mode\n\n");

    // Initialize UART for passthrough
    init_ch32_uart_passthrough();

    // Buffer for receiving data
    uint8_t* rx_buffer = (uint8_t*) malloc(UART_RX_BUF_SIZE);
    if (!rx_buffer) {
        printf("Error: Failed to allocate RX buffer\n");
        stop_ch32_uart_passthrough();
        return 1;
    }

    // Read from UART and echo to console until user presses Enter
    uint8_t console_buf[1];
    while (1) {
        // Read data from CH32 UART (non-blocking with short timeout)
        int len = uart_read_bytes(CH32_UART_PORT, rx_buffer, UART_RX_BUF_SIZE - 1, pdMS_TO_TICKS(10));

        if (len > 0) {
            // Print the received data
            rx_buffer[len] = '\0';
            printf("%s", (char*)rx_buffer);
            fflush(stdout);
        }

        // Check if user pressed Enter on console UART (UART_NUM_0)
        int console_len = uart_read_bytes(UART_NUM_0, console_buf, 1, pdMS_TO_TICKS(10));
        if (console_len > 0) {
            // Check for Enter key (0x0D = CR, 0x0A = LF)
            if (console_buf[0] == 0x0D || console_buf[0] == 0x0A) {
                break;
            }
        }

        // Small delay
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    free(rx_buffer);

    // Stop passthrough
    stop_ch32_uart_passthrough();

    printf("\n\nExited passthrough mode\n");

    return 0;
}

// Main app code
void app_main(void)
{
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_err_t ret = nvs_flash_init();   //Initialize NVS
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(console_cmd_init());     // Initialize console

    // Register CH32 ISP commands with help text
    const esp_console_cmd_t commands[] = {
        {
            .command = "info",
            .help = "Get CH32 device information (chip type, UID, bootloader version)",
            .func = &ch32_info_cmd
        },
        {
            .command = "flash",
            .help = "Flash embedded CH32 firmware and verify",
            .func = &ch32_flash_cmd
        },
        {
            .command = "verify",
            .help = "Verify CH32 flash contents against embedded firmware",
            .func = &ch32_verify_cmd
        },
        {
            .command = "rdp",
            .help = "Manage read protection. Usage: 'rdp lock' or 'rdp unlock' (WARNING: unlock erases flash!)",
            .func = &ch32_rdp_cmd
        },
        {
            .command = "reset",
            .help = "Reset CH32 device by pulsing RST pin",
            .func = &ch32_reset_cmd
        },
        {
            .command = "passthrough",
            .help = "Enter serial passthrough mode to monitor CH32 output (press ENTER to exit)",
            .func = &ch32_passthrough_cmd
        }
    };

    // Register all commands
    for (int i = 0; i < sizeof(commands) / sizeof(commands[0]); i++) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&commands[i]));
    }

    // Register any other plugin command added to your project
    ESP_ERROR_CHECK(console_cmd_all_register());

    ESP_ERROR_CHECK(console_cmd_start());    // Start console

    ESP_LOGI(TAG, "CH32 ISP Loader ready. Type 'help' to see available commands.");
}