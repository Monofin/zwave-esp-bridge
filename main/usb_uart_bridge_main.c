/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_app_desc.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"
#include "soc/rtc_cntl_reg.h"

#include "tinyusb.h"
#include "tusb.h"
#include "tusb_cdc_acm.h"

static const char *TAG = "USB2UART";

// To allow entering the command mode to access the ESP32 bootloader and restart chips
// *without* RTS/DTR, set these baudrates on the serial port in this order.
// Some OSes reset the serial port to a default baudrate so
// as long as these are opened in this order (but not necessarily sequentially), the
// command mode will be triggered.
const uint32_t MAGIC_CMDMODE_TRIGGER_BAUDRATES[] = {150, 300, 1200};
const uint64_t MAGIC_CMDMODE_TRIGGER_TIMEOUT_MICROS = 5000000;

// Minimum interval between repeated warnings while a host stall persists.
const int64_t STALL_LOG_INTERVAL_MICROS = 1000000;

#define BOARD_UART_PORT        UART_NUM_1
#define BOARD_UART_TXD_PIN     CONFIG_BOARD_UART_TXD_PIN
#define BOARD_UART_RXD_PIN     CONFIG_BOARD_UART_RXD_PIN
#define UART_RX_BUF_SIZE       CONFIG_UART_RX_BUF_SIZE
#define UART_TX_BUF_SIZE       CONFIG_UART_TX_BUF_SIZE
#define UART_QUEUE_SIZE        16

#define BOARD_ZG23_RESET_PIN   CONFIG_BOARD_ZG23_RESET_PIN
#define BOARD_ZG23_BTL_PIN     CONFIG_BOARD_ZG23_BTL_PIN

#define USB_RX_BUF_SIZE CONFIG_USB_RX_BUF_SIZE
#define USB_TX_BUF_SIZE CONFIG_USB_TX_BUF_SIZE

#define CFG_BAUD_RATE(b) (b)
#define CFG_STOP_BITS(s) (((s)==2)?UART_STOP_BITS_2:(((s)==1)?UART_STOP_BITS_1_5:UART_STOP_BITS_1))
#define CFG_PARITY(p) (((p)==2)?UART_PARITY_EVEN:(((p)==1)?UART_PARITY_ODD:UART_PARITY_DISABLE))
#define CFG_DATA_BITS(b) (((b)==5)?UART_DATA_5_BITS:(((b)==6)?UART_DATA_6_BITS:(((b)==7)?UART_DATA_7_BITS:UART_DATA_8_BITS)))

#define STR_STOP_BITS(s) (((s)==2)?"UART_STOP_BITS_2":(((s)==1)?"UART_STOP_BITS_1_5":"UART_STOP_BITS_1"))
#define STR_PARITY(p) (((p)==2)?"UART_PARITY_EVEN":(((p)==1)?"UART_PARITY_ODD":"UART_PARITY_DISABLE"))
#define STR_DATA_BITS(b) (((b)==5)?"UART_DATA_5_BITS":(((b)==6)?"UART_DATA_6_BITS":(((b)==7)?"UART_DATA_7_BITS":"UART_DATA_8_BITS")))

/*
* uint32_t bit_rate;
* uint8_t  stop_bits; ///< 0: 1 stop bit - 1: 1.5 stop bits - 2: 2 stop bits
* uint8_t  parity;    ///< 0: None - 1: Odd - 2: Even - 3: Mark - 4: Space
* uint8_t  data_bits; ///< can be 5, 6, 7, 8 or 16
* */
static uint32_t s_baud_rate_active = 115200;
static uint8_t s_stop_bits_active = 0;
static uint8_t s_parity_active = 0;
static uint8_t s_data_bits_active = 8;
static bool s_cmd_mode_enabled = false;
volatile bool s_reset_to_flash = false;
volatile bool s_wait_reset = false;
volatile bool s_in_boot = false;
static RingbufHandle_t s_usb_tx_ringbuf = NULL;
static RingbufHandle_t s_usb_rx_ringbuf = NULL;
static SemaphoreHandle_t s_trigger_bootloader = NULL;
static TaskHandle_t s_usb_tx_handle = NULL;
static SemaphoreHandle_t s_tx_lock;
static uint32_t s_tx_generation;
static bool s_tx_closed;
static bool s_dtr;

// The USB callbacks run in task context. Never wait for USB progress under this lock.
static void reset_usb_tx(bool closed)
{
    xSemaphoreTake(s_tx_lock, portMAX_DELAY);
    ++s_tx_generation;
    s_tx_closed = closed;
    size_t size;
    void *item;
    while ((item = xRingbufferReceive(s_usb_tx_ringbuf, &size, 0)) != NULL) {
        vRingbufferReturnItem(s_usb_tx_ringbuf, item);
    }
    tud_cdc_n_write_clear(0);
    xSemaphoreGive(s_tx_lock);
}

static void resume_usb_tx(void)
{
    xSemaphoreTake(s_tx_lock, portMAX_DELAY);
    s_tx_closed = false;
    xSemaphoreGive(s_tx_lock);
}

static BaseType_t enqueue_usb_tx(const void *data, size_t length)
{
    xSemaphoreTake(s_tx_lock, portMAX_DELAY);
    // Data received between serial sessions has no reader.
    BaseType_t result = pdTRUE;
    if (!s_tx_closed) {
        result = xRingbufferSend(s_usb_tx_ringbuf, data, length, 0);
        if (result == pdTRUE) {
            xTaskNotifyGive(s_usb_tx_handle);
        }
    }
    xSemaphoreGive(s_tx_lock);
    return result;
}

static bool board_zg23_reset_gpio_init(void)
{
    // Make sure both pins are inactive before configuring them as outputs.
    // Otherwise, this will reset the ZG23.
    gpio_set_level(BOARD_ZG23_RESET_PIN, 1);
    gpio_set_level(BOARD_ZG23_BTL_PIN, 1);

    // The BOOT line is active low, and not used otherwise.
    // Set it as an output with a pull-up resistor, so it is in a defined state when not in use.
    gpio_config_t io_conf = {0};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set
    io_conf.pin_bit_mask = (1ULL << BOARD_ZG23_BTL_PIN);
    //disable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    //enable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    //configure GPIO with the given settings
    esp_err_t ret = gpio_config(&io_conf);

    if (ret != ESP_OK) {
        return false;
    }

    // The RESET line has a pull-up resistor and we should not control it
    // unless we're resetting, because that messes with the ZG23 debugger.
    // Therefore configure it as an open-drain output, so the debugger
    // can drive it while we don't.
    io_conf = (gpio_config_t) {0};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT_OD;
    //bit mask of the pins that you want to set
    io_conf.pin_bit_mask = (1ULL << BOARD_ZG23_RESET_PIN);
    //disable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    //enable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    //configure GPIO with the given settings
    ret = gpio_config(&io_conf);

    if (ret != ESP_OK) {
        return false;
    }

    return true;
}

// Invoked when device is mounted
void tud_mount_cb(void)
{
    s_dtr = false;
    reset_usb_tx(false);
    ESP_LOGI(TAG, "USB Mounted");
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
    s_dtr = false;
    reset_usb_tx(true);
    ESP_LOGI(TAG, "USB Unmounted");
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allows us to perform remote wakeup
// USB Specs: Within 7ms, device must draw an average current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
    ESP_LOGI(TAG, "USB Suspend");
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
    ESP_LOGI(TAG, "USB Resume");
}

void tud_cdc_tx_complete_cb(const uint8_t itf)
{
    ESP_LOGD(TAG, "USB TX completed");
}

static void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    /* initialization */
    size_t rx_size = 0;
    static uint8_t rx_buf[USB_RX_BUF_SIZE] = {0};

    /* read from usb */
    esp_err_t ret = tinyusb_cdcacm_read(itf, rx_buf, USB_RX_BUF_SIZE, &rx_size);
    ESP_LOGD(TAG, "tinyusb_cdc_rx_callback (size: %u)", rx_size);
    ESP_LOG_BUFFER_HEXDUMP(TAG, rx_buf, rx_size, ESP_LOG_VERBOSE);

    if (ret == ESP_OK && rx_size > 0) {
        // A host that does not drive DTR can start a session by sending data.
        resume_usb_tx();
        if (s_cmd_mode_enabled) {
            switch (rx_buf[0]) {
                case 'I':
                case 'i':
                    // Respond with version info
                    const esp_app_desc_t *app_desc = esp_app_get_description();
                    char info_buf[64]; // 31 static + 32 max version length + 1 byte null terminator
                    int len = snprintf(info_buf, sizeof(info_buf), "USB2UART bridge\r\nversion %s\r\n", app_desc->version);
                    enqueue_usb_tx(info_buf, len);
                    // Do not exit command mode
                    return;
                case 'B':
                case 'b':
                    if (rx_buf[1] == 'e' || rx_buf[1] == 'E') {
                        // Trigger bootloader
                        ESP_LOGW(TAG, "Invoking ESP32-S3 bootloader");
                        xSemaphoreGive(s_trigger_bootloader);
                    } else if (rx_buf[1] == 'z' || rx_buf[1] == 'Z') {
                        // Trigger ZG23 bootloader
                        ESP_LOGW(TAG, "Invoking ZG23 bootloader");
                        gpio_set_level(BOARD_ZG23_BTL_PIN, true);
                        gpio_set_level(BOARD_ZG23_RESET_PIN, false);
                        vTaskDelay(pdMS_TO_TICKS(100));
                        gpio_set_level(BOARD_ZG23_BTL_PIN, false);
                        gpio_set_level(BOARD_ZG23_RESET_PIN, true);
                        vTaskDelay(pdMS_TO_TICKS(500));
                        gpio_set_level(BOARD_ZG23_BTL_PIN, true);
                    } else {
                        ESP_LOGW(TAG, "Unknown command: %c%c", rx_buf[0], rx_buf[1]);
                    }
                    break;
                case 'R':
                case 'r':
                    if (rx_buf[1] == 'e' || rx_buf[1] == 'E') {
                        // Reboot ESP32-S3
                        ESP_LOGW(TAG, "Rebooting ESP32-S3");
                        esp_restart();
                    } else if (rx_buf[1] == 'z' || rx_buf[1] == 'Z') {
                        // Reboot ZG23
                        ESP_LOGW(TAG, "Rebooting ZG23");
                        gpio_set_level(BOARD_ZG23_RESET_PIN, false);
                        vTaskDelay(pdMS_TO_TICKS(50));
                        gpio_set_level(BOARD_ZG23_RESET_PIN, true);
                    } else {
                        ESP_LOGW(TAG, "Unknown command: %c%c", rx_buf[0], rx_buf[1]);
                    }
                    break;
                case 'X':
                case 'x':
                    // Exit command mode without doing anything
                    break;
                default:
                    ESP_LOGW(TAG, "Unknown command: %c", rx_buf[0]);
                    break;
            }
            s_cmd_mode_enabled = false;
            ESP_LOGW(TAG, "Exited command mode");
            return;
        }

        BaseType_t send_res = xRingbufferSend(s_usb_rx_ringbuf, rx_buf, rx_size, 0);
        if (send_res != pdTRUE) {
            ESP_LOGE(TAG, "USB RX to UART RingBuf: Buffer full, %u bytes lost", rx_size);
        } else {
            ESP_LOGD(TAG, "USB RX to UART RingBuf: Queued %u bytes", rx_size);
        }
    }
}

static void tinyusb_cdc_line_state_changed_callback(int itf, cdcacm_event_t *event)
{
    int dtr = event->line_state_changed_data.dtr;
    int rts = event->line_state_changed_data.rts;
    if (s_dtr && !dtr) {
        reset_usb_tx(true);
    } else if (dtr) {
        resume_usb_tx();
    }
    s_dtr = dtr;
    ESP_LOGD(TAG, "Line state changed! dtr:%d, rts:%d ", dtr, rts);
}

static void tinyusb_cdc_line_coding_changed_callback(int itf, cdcacm_event_t *event)
{
    uint32_t bit_rate = event->line_coding_changed_data.p_line_coding->bit_rate;
    uint8_t stop_bits = event->line_coding_changed_data.p_line_coding->stop_bits;
    uint8_t parity = event->line_coding_changed_data.p_line_coding->parity;
    uint8_t data_bits = event->line_coding_changed_data.p_line_coding->data_bits;
    ESP_LOGV(TAG, "host require bit_rate=%" PRIu32 " stop_bits=%u parity=%u data_bits=%u", bit_rate, stop_bits, parity, data_bits);

    static uint8_t magic_cmdmode_trigger_stage = 0;
    static uint64_t magic_cmdmode_trigger_time = 0;

    uint64_t now = esp_timer_get_time();

    if ((now - magic_cmdmode_trigger_time > MAGIC_CMDMODE_TRIGGER_TIMEOUT_MICROS) && magic_cmdmode_trigger_stage > 0) {
        ESP_LOGW(TAG, "Magic command mode baudrate period timed out, resetting state");
        magic_cmdmode_trigger_stage = 0;
    }

    if (bit_rate == MAGIC_CMDMODE_TRIGGER_BAUDRATES[magic_cmdmode_trigger_stage]) {
        if (magic_cmdmode_trigger_stage == 0) {
            magic_cmdmode_trigger_time = now;
        }

        ESP_LOGW(TAG, "Received magic command mode baudrate, stage %d (%" PRIu32 " baud)", magic_cmdmode_trigger_stage, bit_rate);
        magic_cmdmode_trigger_stage++;

        if (magic_cmdmode_trigger_stage == sizeof(MAGIC_CMDMODE_TRIGGER_BAUDRATES) / sizeof(MAGIC_CMDMODE_TRIGGER_BAUDRATES[0])) {
            ESP_LOGW(TAG, "Enabling command mode");
            magic_cmdmode_trigger_stage = 0;
            s_cmd_mode_enabled = true;

            // Send 'cmd> ' prompt to host, so it can detect that command mode is active
            resume_usb_tx();
            const char *cmd_prompt = "cmd> ";
            enqueue_usb_tx(cmd_prompt, strlen(cmd_prompt));

            return;
        }
    }

    if (s_baud_rate_active != bit_rate) {
        if (ESP_OK == uart_set_baudrate(BOARD_UART_PORT, CFG_BAUD_RATE(bit_rate))) {
            s_baud_rate_active = bit_rate;
            ESP_LOGI(TAG, "set bit_rate=%" PRIu32, bit_rate);
        }
    }

    if (s_stop_bits_active != stop_bits) {
        if (ESP_OK == uart_set_stop_bits(BOARD_UART_PORT, CFG_STOP_BITS(stop_bits))) {
            s_stop_bits_active = stop_bits;
            ESP_LOGI(TAG, "set stop_bits=%s", STR_STOP_BITS(stop_bits));
        }
    }

    if (s_parity_active != parity) {
        if (ESP_OK == uart_set_parity(BOARD_UART_PORT, CFG_PARITY(parity))) {
            s_parity_active = parity;
            ESP_LOGI(TAG, "set parity=%s", STR_PARITY(parity));
        }
    }

    if (s_data_bits_active != data_bits) {
        if (ESP_OK == uart_set_word_length(BOARD_UART_PORT, CFG_DATA_BITS(data_bits))) {
            s_data_bits_active = data_bits;
            ESP_LOGI(TAG, "set data_bits=%s", STR_DATA_BITS(data_bits));
        }
    }
}

static esp_err_t _ringbuf_read_bytes(RingbufHandle_t ring_buf, uint8_t *out_buf, size_t req_bytes, size_t *read_bytes, TickType_t xTicksToWait)
{
    uint8_t *buf = xRingbufferReceiveUpTo(ring_buf, read_bytes, xTicksToWait, req_bytes);

    if (buf) {
        memcpy(out_buf, buf, *read_bytes);
        vRingbufferReturnItem(ring_buf, (void *)(buf));
        return ESP_OK;
    } else {
        return ESP_FAIL;
    }
}

static esp_err_t ringbuf_read_bytes(RingbufHandle_t ring_buf, uint8_t *out_buf, size_t out_buf_sz, size_t *rx_data_size, TickType_t xTicksToWait)
{
    size_t read_sz;

    esp_err_t res = _ringbuf_read_bytes(ring_buf, out_buf, out_buf_sz, &read_sz, xTicksToWait);

    if (res != ESP_OK) {
        return res;
    }

    *rx_data_size = read_sz;

    /* Buffer's data can be wrapped, at that situations we should make another retrievement */
    if (_ringbuf_read_bytes(ring_buf, out_buf + read_sz, out_buf_sz - read_sz, &read_sz, 0) == ESP_OK) {
        *rx_data_size += read_sz;
    }

    return ESP_OK;
}

static void usb_tx_task(void *arg)
{
    tinyusb_config_cdcacm_t *acm_cfg = (tinyusb_config_cdcacm_t *)arg;
    tinyusb_cdcacm_itf_t itf = acm_cfg->cdc_port;

    uint8_t data[USB_TX_BUF_SIZE] = {0};
    size_t tx_data_size = 0;
    // Back-dated so a stall right after boot still logs immediately
    int64_t stall_log_time = -STALL_LOG_INTERVAL_MICROS;

    while (1) {
        // Wait for a notification from the UART read task
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // When we do wake up, we can be sure there is data in the ring buffer
        xSemaphoreTake(s_tx_lock, portMAX_DELAY);
        uint32_t generation = s_tx_generation;
        esp_err_t ret = ringbuf_read_bytes(s_usb_tx_ringbuf, data, USB_TX_BUF_SIZE, &tx_data_size, 0);
        xSemaphoreGive(s_tx_lock);

        if (ret != ESP_OK) {
            // Closing the port can discard a batch after the task was notified.
            continue;
        } else if (tx_data_size == 0) {
            ESP_LOGE(TAG, "usb tx ringbuf read succeeded with empty read, skipping");
            continue;
        }

        ESP_LOGD(TAG, "read %d bytes from USB TX buffer", tx_data_size);
        ESP_LOG_BUFFER_HEXDUMP(TAG, data, tx_data_size, ESP_LOG_VERBOSE);

        // Serial data will be split up into 64 byte chunks to be sent over USB so this
        // usually will take multiple iterations
        uint8_t *data_head = &data[0];

        for (;;) {
            xSemaphoreTake(s_tx_lock, portMAX_DELAY);
            if (generation != s_tx_generation) {
                xSemaphoreGive(s_tx_lock);
                break;
            }
            size_t queued = tx_data_size ? tinyusb_cdcacm_write_queue(itf, data_head, tx_data_size) : 0;
            xSemaphoreGive(s_tx_lock);
            ESP_LOGV(TAG, "usb tx enqueued: size=%d, queued=%u", tx_data_size, queued);

            tx_data_size -= queued;
            data_head += queued;

            ESP_LOGV(TAG, "usb tx: waiting 10ms for flush");
            esp_err_t flush_ret = tinyusb_cdcacm_write_flush(itf, pdMS_TO_TICKS(10));

            if (flush_ret == ESP_OK) {
                if (tx_data_size == 0) {
                    break;
                }
                continue;
            }

            // Bytes not yet handed to TinyUSB plus bytes still sitting in its transmit
            // FIFO. (tud_cdc_n_write_occupied() is a private helper inside esp_tinyusb,
            // so derive the occupancy from the FIFO depth TinyUSB is configured with.)
            size_t pending = tx_data_size + (CFG_TUD_CDC_TX_BUFSIZE - tud_cdc_n_write_available(itf));

            // Keep ownership until the final queued bytes have drained or the session ends.
            if (flush_ret == ESP_ERR_TIMEOUT && tud_cdc_n_connected(itf)) {
                int64_t now = esp_timer_get_time();
                if (now - stall_log_time >= STALL_LOG_INTERVAL_MICROS) {
                    stall_log_time = now;
                    ESP_LOGW(TAG, "usb tx: host not reading, %u bytes pending", (unsigned) pending);
                }
                continue;
            }

            if (flush_ret == ESP_ERR_TIMEOUT) {
                ESP_LOGW(TAG, "usb tx: not connected, dropping %u bytes", (unsigned) pending);
            } else {
                ESP_LOGE(TAG, "usb tx: flush failed (%s), dropping %u bytes",
                         esp_err_to_name(flush_ret), (unsigned) pending);
            }
            xSemaphoreTake(s_tx_lock, portMAX_DELAY);
            if (generation == s_tx_generation) {
                tud_cdc_n_write_clear(itf);
            }
            xSemaphoreGive(s_tx_lock);
            break;
        }
    }
}

typedef struct uart_rx_task_param_t {
    QueueHandle_t uart_queue;
} uart_rx_task_param_t;


static void uart_rx_task(void *arg)
{
    QueueHandle_t uart_queue = ((uart_rx_task_param_t*)arg)->uart_queue;

    uint8_t data[UART_RX_BUF_SIZE] = {0};
    uart_event_t event;
    // The driver can receive data before CDC initialization has completed.
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    // Running total of bytes dropped because the USB TX ring buffer was full (never
    // reset), and the back-dated timestamp of the last warning about it.
    uint32_t dropped_total = 0;
    int64_t drop_log_time = -STALL_LOG_INTERVAL_MICROS;

    while (1) {
        if (!xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY)) {
            continue;
        }

        switch (event.type) {
            case UART_DATA:
                while (1) {
                    const int rx_data_size = uart_read_bytes(BOARD_UART_PORT, data, MIN(UART_RX_BUF_SIZE, event.size), 0);
                    ESP_LOGD(TAG, "uart rx: %d bytes", rx_data_size);

                    if (rx_data_size == 0) {
                        // There's no more data to read
                        break;
                    }

                    if (rx_data_size < 0) {
                        ESP_LOGE(TAG, "uart read failed: %d", rx_data_size);
                        break;
                    }

                    int res = enqueue_usb_tx(data, rx_data_size);
                    if (res != pdTRUE) {
                        // While the host stalls, the USB TX task holds pending data and
                        // the ring buffer stays full (that is the intended
                        // backpressure), so this runs for every read; throttle the
                        // warning. The counter is a running total so bytes dropped in
                        // the tail of one stall are still reported by the next line,
                        // and the very first drop since boot always logs.
                        bool first_drop = dropped_total == 0;
                        dropped_total += rx_data_size;
                        int64_t now = esp_timer_get_time();
                        if (first_drop || now - drop_log_time >= STALL_LOG_INTERVAL_MICROS) {
                            drop_log_time = now;
                            ESP_LOGW(TAG, "usb tx ring buffer full, %" PRIu32 " bytes dropped total", dropped_total);
                        }
                    }
                }

                break;

            default:
                ESP_LOGI(TAG, "uart event type: %d", event.type);
                break;
        }
    }
}

static void uart_tx_task(void *arg) {
    uint8_t data_to_uart[CONFIG_USB_RX_BUF_SIZE];
    size_t rx_size;

    while (1) {
        ESP_LOGD(TAG, "waiting for data to send to uart");
        esp_err_t ret = ringbuf_read_bytes(s_usb_rx_ringbuf, data_to_uart, sizeof(data_to_uart), &rx_size, portMAX_DELAY);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "usb rx ringbuf read failed");
            continue;
        }

        size_t xfer_size = uart_write_bytes(BOARD_UART_PORT, data_to_uart, rx_size);

        if (xfer_size != rx_size) {
            ESP_LOGD(TAG, "uart write lost (%d/%d)", xfer_size, rx_size);
        }

        ESP_LOGD(TAG, "waiting for UART buffer to flush");
        ESP_ERROR_CHECK(uart_wait_tx_done(BOARD_UART_PORT, portMAX_DELAY));
    }
}

static void bootloader_reset_task(void *arg) {
    xSemaphoreTake(s_trigger_bootloader, portMAX_DELAY);

    // Give the USB stack a little bit more time to finish up, this is triggered from an
    // interrupt context
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGW(TAG, "Uninstalling tinyUSB driver");
    tinyusb_driver_uninstall();

    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGW(TAG, "Triggering bootloader");;
    REG_WRITE(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);
    esp_restart();
}

void app_main(void)
{
    // Only for debugging - do not leave uncommented in production:
    // esp_log_level_set("*", ESP_LOG_VERBOSE);

    uart_config_t uart_config = {
        .baud_rate = CFG_BAUD_RATE(s_baud_rate_active),
        .data_bits = CFG_DATA_BITS(s_data_bits_active),
        .parity = CFG_PARITY(s_parity_active),
        .stop_bits = CFG_STOP_BITS(s_stop_bits_active),
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    QueueHandle_t uart_queue = NULL;
    ESP_ERROR_CHECK(uart_driver_install(BOARD_UART_PORT, UART_RX_BUF_SIZE, UART_TX_BUF_SIZE, UART_QUEUE_SIZE, &uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(BOARD_UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(BOARD_UART_PORT, BOARD_UART_TXD_PIN, BOARD_UART_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "init UART%d: %"PRIu32 " %s %s %s", BOARD_UART_PORT, s_baud_rate_active, STR_DATA_BITS(s_data_bits_active), STR_PARITY(s_parity_active), STR_STOP_BITS(s_stop_bits_active));

    ESP_ERROR_CHECK(board_zg23_reset_gpio_init() ? ESP_OK : ESP_FAIL);

    s_usb_tx_ringbuf = xRingbufferCreate(USB_TX_BUF_SIZE, RINGBUF_TYPE_BYTEBUF);
    s_trigger_bootloader = xSemaphoreCreateBinary();
    s_tx_lock = xSemaphoreCreateMutex();

    s_usb_rx_ringbuf = xRingbufferCreate(UART_TX_BUF_SIZE, RINGBUF_TYPE_BYTEBUF);
    ESP_ERROR_CHECK(s_usb_tx_ringbuf && s_usb_rx_ringbuf && s_trigger_bootloader && s_tx_lock
                    ? ESP_OK : ESP_ERR_NO_MEM);

    // Determine MAC address and use it as the serial number
    uint8_t mac[8];
    ESP_ERROR_CHECK(esp_efuse_mac_get_default(mac));

    // Convert MAC to hex string for serial number
    char serial_str[13];  // 6 bytes * 2 chars per byte + null terminator
    snprintf(serial_str, sizeof(serial_str), "%02X%02X%02X%02X%02X%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    const char *string_descriptor[] = {
        // array of pointer to string descriptors
        (char[]){0x09, 0x04},                    // 0: is supported language is English (0x0409)
        // 1: Manufacturer
        #ifdef ADAPTER_USB_MANUFACTURER_OVERRIDE
            ADAPTER_USB_MANUFACTURER_OVERRIDE,
        #else
            CONFIG_TINYUSB_DESC_MANUFACTURER_STRING,
        #endif
        // 2: Product
        #ifdef ADAPTER_USB_PRODUCT_OVERRIDE
            ADAPTER_USB_PRODUCT_OVERRIDE,
        #else
            CONFIG_TINYUSB_DESC_PRODUCT_STRING,
        #endif
        serial_str,                              // 3: Serials, should use chip ID

    #if CONFIG_TINYUSB_CDC_ENABLED
        // 4: CDC Interface
        #ifdef ADAPTER_USB_CDC_STRING_OVERRIDE
            ADAPTER_USB_CDC_STRING_OVERRIDE,
        #else
            CONFIG_TINYUSB_DESC_CDC_STRING,
        #endif
    #else
        "",
    #endif

    #if CONFIG_TINYUSB_MSC_ENABLED
        CONFIG_TINYUSB_DESC_MSC_STRING,          // 5: MSC Interface
    #else
        "",
    #endif

    #if CONFIG_TINYUSB_NET_MODE_ECM_RNDIS || CONFIG_TINYUSB_NET_MODE_NCM
        "USB net",                               // 6. NET Interface
        "",                                      // 7. MAC
    #endif
        NULL                                     // NULL: Must be last. Indicates end of array
    };


    tinyusb_config_t tusb_cfg = {
        .descriptor = NULL,
        .string_descriptor = string_descriptor,
        .string_descriptor_count = 6,
        .external_phy = false
    };

    tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = USB_RX_BUF_SIZE,
        .callback_rx = &tinyusb_cdc_rx_callback, // the first way to register a callback
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = &tinyusb_cdc_line_state_changed_callback,
        .callback_line_coding_changed = &tinyusb_cdc_line_coding_changed_callback
    };

    TaskHandle_t usb_tx_handle = NULL;

    size_t stack_size = 4096;

    if (esp_log_level_get(TAG) >= ESP_LOG_DEBUG) {
        stack_size = 8192;  // Increase stack size for debug logging
    }

    ESP_ERROR_CHECK(xTaskCreate(usb_tx_task, "usb_tx", stack_size, &acm_cfg, 5, &usb_tx_handle) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    s_usb_tx_handle = usb_tx_handle; // Store in global variable for access in tinyusb callbacks
    ESP_ERROR_CHECK(xTaskCreate(uart_tx_task, "uart_tx", stack_size, NULL, 4, NULL) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(xTaskCreate(bootloader_reset_task, "bootloader_reset", 2048, NULL, 4, NULL) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);

    uart_rx_task_param_t uart_rx_task_param = {
        .uart_queue = uart_queue,
    };
    TaskHandle_t uart_rx_handle;
    ESP_ERROR_CHECK(xTaskCreate(uart_rx_task, "uart_rx", stack_size, &uart_rx_task_param, 4, &uart_rx_handle) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));
    xTaskNotifyGive(uart_rx_handle);

    ESP_LOGI(TAG, "USB initialization DONE");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
