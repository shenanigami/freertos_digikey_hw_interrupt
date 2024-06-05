#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_adc/adc_continuous.h"

/* configs */
// configTASK_NOTIFICATION_ARRAY_ENTRIES set to 2

// uart driver configuration
#define ECHO_TEST_TXD GPIO_NUM_1
#define ECHO_TEST_RXD GPIO_NUM_3
// no flow control
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define ECHO_UART_PORT_NUM      UART_NUM_0
#define ECHO_UART_BAUD_RATE     115200

#define AVG_BUF_SIZE 3
#define DATA_BUF_SIZE 2
#define BUF_SIZE 256

#define ADC_READ_LEN 4

static const char *TAG = "[HW_INTERRUPT] ";

static TaskHandle_t x_compute_avg_task = NULL,
                    x_echo_avg_task = NULL;
static QueueHandle_t queue1, queue2;
static adc_continuous_handle_t adc_handle = NULL;
#define TASK_STACK_SIZE 2*1024
#define QUEUE_SIZE 10

portMUX_TYPE adc_mutex = portMUX_INITIALIZER_UNLOCKED;

static float avg = 0.0;

static gptimer_handle_t adc_read_timer = NULL;

static void continuous_adc_init(adc_continuous_handle_t *adc_handle)
{
    // sample adc value
    // resource allocation
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 40,       // 10 conversion frames is good
        .conv_frame_size = 4,           // conversion frame size, in bytes. This should be in multiples of SOC_ADC_DIGI_DATA_BYTES_PER_CONV
                                        // conversion frame consisting of single conversion result is enough for this program
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, adc_handle));

    adc_digi_pattern_config_t *adc_pattern = malloc(sizeof(adc_digi_pattern_config_t));
    assert(adc_pattern);
    *adc_pattern = (adc_digi_pattern_config_t) {
        .atten = ADC_ATTEN_DB_0,            // No input attenuation, ADC can measure up to approx.
        .channel = ADC_CHANNEL_7,
        .unit = ADC_UNIT_1,
        .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,     // 12 bits
    };

    // ADC Configurations
    adc_continuous_config_t dig_cfg = {
        .pattern_num = 1,                           // number of ADC channels that will be used
        .adc_pattern = adc_pattern,                 // list of configs for each ADC channel that will be used
        .sample_freq_hz = 20 * 1000,                // minimum sample rate, defined in soc/soc_caps.h
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,        // Only use ADC1 for conversion
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,     // type 1 seems to be the suitable type for esp32 adc1 single unit conversion
    };

    ESP_ERROR_CHECK(adc_continuous_config(*adc_handle, &dig_cfg));
    ESP_ERROR_CHECK(adc_continuous_start(*adc_handle));
}

// fromISR
// return bool value is - Whether a high priority task has been waken up by this function
static bool adc_sampling_timer_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    uint8_t *queue_num = (uint8_t *) user_ctx;

    uint8_t result[ADC_READ_LEN] = { 0 };
    uint32_t out_length = 0;
    QueueHandle_t queue_cb = *queue_num == 1 ? queue1 : queue2;

    adc_continuous_read(adc_handle, result, ADC_READ_LEN, &out_length, 0);

    if (xQueueSendFromISR(queue_cb, (void *) result, NULL) != pdPASS) {
        // notify compute_avg task - index 0
        xTaskNotifyIndexedFromISR(x_compute_avg_task, 0, (uint32_t) (*queue_num), eSetValueWithOverwrite, NULL);
        *queue_num ^= 0x03;    // toggle between 1 and 2
    }

    return false;
}


static void compute_avg_task(void *arg)
{

    static uint32_t num = 0;
    static uint32_t queue_num = 0;
    int i = 0;
    while(1) {
        xTaskNotifyWaitIndexed(0,               // index 0
                               0x00,            // no need to clear notification value
                               0x00,            // again, nothing to clear on exit
                               &queue_num,      // notification value that holds queue num
                               portMAX_DELAY    // max time to wait in blocked state for notifcation
                              );
        avg = 0.0; i = 0;
        QueueHandle_t queue_handle = queue_num == 1 ? queue1 : queue2;
        while (xQueueReceive(queue_handle, &num, 0) == pdPASS) {
            i++;
            avg += num;
        }
        avg /= i;
        ESP_LOGI(TAG, "avg of queue%lu is %f", queue_num, avg);
    }

}

// no timeout
static bool readUntilWhitespaceOrAvg(size_t cmd_word_size, const char* cmd_word)
{
    bool is_cmd_word = true;
    uint8_t c = 0;
    int i = 1;

    int len = uart_read_bytes(ECHO_UART_PORT_NUM, &c, 1, 20 / portTICK_PERIOD_MS);
    while (i < cmd_word_size) {

        if (len > 0 && c > 0) {
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*) &c, len);
            if (c == 'a') {
                i = 0;
                ESP_LOGD(TAG, "avg soon?");
            } else if (c != cmd_word[i]) {
                is_cmd_word = false;
                break;
            }
            i++;
        }
        len = uart_read_bytes(ECHO_UART_PORT_NUM, &c, 1, 20 / portTICK_PERIOD_MS);
    }

    return is_cmd_word;
}

static void echo_avg_task(void *arg)
{
    // Configure parameters of an UART driver,
     // communication pins and install the driver
    uart_config_t uart_config = {
        .baud_rate = ECHO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif
    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));

    // Configure a temporary buffer for the incoming data
    uint8_t data[DATA_BUF_SIZE] = { 0 };
    bool is_avg = false;
    const char* avg_cmd = "avg";

    while (1) {
        // Read data from the UART
        int len = uart_read_bytes(ECHO_UART_PORT_NUM, data, 1, 20 / portTICK_PERIOD_MS);
        if (*data == '\r')
            *data = '\n';
        else if (*data == 'a') {
            ESP_LOGD(TAG, "avg, is that you?");
            is_avg = true;
        }

        // Write data back to the UART
        uart_write_bytes(ECHO_UART_PORT_NUM, (const char*) data, len);
        if (len) {
            data[len] = '\0';
            ESP_LOGD(TAG, "Recv str: %s", (char *) data);
        }

        if (is_avg) {
            float local_avg = avg;
            if (readUntilWhitespaceOrAvg(AVG_BUF_SIZE, avg_cmd))
                ESP_LOGI(TAG, "avg is %f", local_avg);
            else
                ESP_LOGI(TAG, "not avg");

            is_avg = false;
        }
        *data = '\0';
    }

}

void app_main(void)
{
    // resource allocation timer
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1 * 1000 * 1000,  // 1 MHz, 1 tick = 1 us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &adc_read_timer));

    // alarm config
    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,  // counter will reload with 0 on alarm event
        .alarm_count = 100000,  // period = 100ms @resolution 1MHz
        .flags.auto_reload_on_alarm = true,  // enable auto-reload
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(adc_read_timer, &alarm_config));

    // callback config
    gptimer_event_callbacks_t cbs = {
        .on_alarm = adc_sampling_timer_cb,  // register user callback
    };

    uint8_t *queue_num = malloc(sizeof(uint8_t));
    assert(queue_num);
    *queue_num = 1;

    ESP_ERROR_CHECK(gptimer_register_event_callbacks(adc_read_timer, &cbs, (void *) queue_num));
    // Create queues
    queue1 = xQueueCreate(QUEUE_SIZE, sizeof(uint32_t));
    queue2 = xQueueCreate(QUEUE_SIZE, sizeof(uint32_t));

    xTaskCreate(compute_avg_task, "compute_avg_task", TASK_STACK_SIZE, NULL, 1, &x_compute_avg_task);
    xTaskCreate(echo_avg_task, "echo_avg_task", TASK_STACK_SIZE, NULL, 1, &x_echo_avg_task);

    continuous_adc_init(&adc_handle);

    ESP_ERROR_CHECK(gptimer_enable(adc_read_timer));
    ESP_ERROR_CHECK(gptimer_start(adc_read_timer));
}
