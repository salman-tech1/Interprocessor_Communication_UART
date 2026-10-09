#include "com.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>

#define COM_UART_PORT_NUM UART_NUM_1
#define COM_UART_BAUD_RATE 115200U
#define COM_UART_TX_PIN 17 /* GPIO17 = TX */
#define COM_UART_RX_PIN 16 /* GPIO16 = RX */

/* ESP-IDF driver's own internal RX/TX ring buffers. */
#define COM_UART_RX_BUF_SIZE 1024U
#define COM_UART_TX_BUF_SIZE 1024U

/* Depth of the driver's UART event queue. */
#define COM_UART_EVENT_QUEUE_LEN 20U

/* Largest single buffer any upper layer will ever hand to Com_Send(). */
#define COM_MAX_TX_SIZE 600U

/* How long UART RX poll */
#define COM_RX_POLL_TIMEOUT_MS 10U

/* Com Tx to wait  */
#define COM_TX_WAIT_TIMEOUT_MS 100U

static QueueHandle_t uart_event_queue = NULL;  // Events Queue
static QueueHandle_t uart_queu_receive = NULL; // reciever queue
static Com_EventCallback_t s_event_cb = NULL;
static SemaphoreHandle_t uart_mutex = NULL;
static TaskHandle_t uart_task_handle = NULL;

static const char *TAG = "COM";

typedef struct
{
    uint8_t buf[512];
    size_t len;
} rx_buff_t;

// Look for any events related to UART
void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    rx_buff_t rx_q;

    for (;;)
    {
        if (xQueueReceive(uart_event_queue, (void *)&event, portMAX_DELAY))
        {
            switch (event.type)
            {
            case UART_DATA:
            {
                size_t len;
                // get the size of the recieved bytes
                ESP_ERROR_CHECK(uart_get_buffered_data_len(COM_UART_PORT_NUM, &len));
                // check the size first if it over flows
                if (len > sizeof(rx_q.buf))
                {
                    len = sizeof(rx_q.buf);
                    ESP_LOGW(TAG, "Truncating received data to fit buffer");
                }

                // read bytes from the ring buffer : return the actual length of the bytes read
                rx_q.len = uart_read_bytes(COM_UART_PORT_NUM, rx_q.buf, len, pdMS_TO_TICKS(100));

                if (rx_q.len > 0 && uart_queu_receive != NULL)
                {
                    // Send the read bytes
                    if (xQueueSend(uart_queu_receive, &rx_q, pdMS_TO_TICKS(1000)) != pdTRUE)
                    {
                        ESP_LOGW(TAG, "Failed to queue received data");

                        // error :
                        if (s_event_cb)
                        {
                            s_event_cb(COM_EVENT_ERROR);
                        }
                    }
                }
                break;
            }
            case UART_FIFO_OVF:
            case UART_BUFFER_FULL:
                ESP_LOGW(TAG, "UART FIFO overflow or buffer full");
                uart_flush_input(COM_UART_PORT_NUM);
                xQueueReset(uart_event_queue);
                xQueueReset(uart_queu_receive);
                if (s_event_cb)
                {
                    s_event_cb(COM_EVENT_ERROR);
                }
                break;
            default:
                break;
            }
        }
    }
}

// Read bytes up to length
Com_Status_t Com_recieve(uint8_t *data, uint32_t *len)
{
    if (uart_queu_receive == NULL || data == NULL || len == NULL)
    {
        return COM_ERROR;
    }

    //
    rx_buff_t rx_q;

    if (xQueueReceive(uart_queu_receive, &rx_q, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        // Copy data to the output buffer
        uint32_t copy_len = rx_q.len;

        // recieve bytes
        if (copy_len > *len)
        {
            copy_len = *len;
            ESP_LOGW(TAG, "Truncating data to fit output buffer");
        }

        // copy the number of bytes into the data
        memcpy(data, rx_q.buf, copy_len);
        *len = copy_len;

        //  ESP_LOGI(TAG, "Received %d bytes", copy_len);

        if (s_event_cb)
        {
            s_event_cb(COM_EVENT_RX_DATA);
        }

        return COM_OK;
    }

    return COM_BUSY;
}

// Send bytes
Com_Status_t Com_Send(const uint8_t *data, uint16_t length)
{
    if (data == NULL || length == 0)
    {
        return COM_ERROR;
    }

    // Take mutex to ensure thread safety
    if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(COM_TX_WAIT_TIMEOUT_MS)) != pdTRUE)
    {
        return COM_BUSY;
    }

    int num = uart_write_bytes(COM_UART_PORT_NUM, data, length);
    if (num != (int)length)
    {
        xSemaphoreGive(uart_mutex);
        return COM_ERROR;
    }

    esp_err_t err = uart_wait_tx_done(COM_UART_PORT_NUM, pdMS_TO_TICKS(1000));
    xSemaphoreGive(uart_mutex);

    if (err == ESP_OK)
    {
        //    ESP_LOGI(TAG, "Bytes written successfully");
        return COM_OK;
    }

    return COM_TIMEOUT;
}

// Register event callback
void Com_RegisterEventCallback(Com_EventCallback_t callback)
{
    s_event_cb = callback;
}

// Recover from error state
Com_Status_t Com_Recover(void)
{
    if (uart_is_driver_installed(COM_UART_PORT_NUM) != true)
    {
        return COM_ERROR;
    }

    uart_flush_input(COM_UART_PORT_NUM);
    xQueueReset(uart_event_queue);
    xQueueReset(uart_queu_receive);

    ESP_LOGI(TAG, "UART recovered from error state");
    return COM_OK;
}

// Initialize COM driver
Com_Status_t Com_Init(void)
{

    // install the Driver accoring to these parameters
    const uart_config_t uart_config = {
        .baud_rate = COM_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // Create mutex for thread safety
    uart_mutex = xSemaphoreCreateMutex();
    if (uart_mutex == NULL)
    {
        ESP_LOGE(TAG, "Failed to create UART mutex");
        return COM_ERROR;
    }

    // Set parameters :
    if (uart_param_config(COM_UART_PORT_NUM, &uart_config) != ESP_OK)
    {
        ESP_LOGE(TAG, "uart_param_config failed");
        return COM_ERROR;
    }

    // Set pinouts :
    if (uart_set_pin(COM_UART_PORT_NUM, COM_UART_TX_PIN, COM_UART_RX_PIN,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK)
    {
        ESP_LOGE(TAG, "uart_set_pin failed");
        return COM_ERROR;
    }

    // PULL UP GPIO to vdd incase if it left floating to not catch
    // noise.
    gpio_set_pull_mode(GPIO_NUM_16, GPIO_PULLUP_ONLY);

    // UART Driver install :
    if (uart_driver_install(COM_UART_PORT_NUM, COM_UART_RX_BUF_SIZE,
                            COM_UART_TX_BUF_SIZE, COM_UART_EVENT_QUEUE_LEN,
                            &uart_event_queue, 0) != ESP_OK)
    {
        ESP_LOGE(TAG, "uart_driver_install failed");
        return COM_ERROR;
    }

    if (uart_is_driver_installed(COM_UART_PORT_NUM) == true)
    {
        ESP_LOGI(TAG, "Driver is installed");
    }
    else
    {
        ESP_LOGE(TAG, "Driver is not installed");
        return COM_ERROR;
    }

    // Create a queue for received data
    uart_queu_receive = xQueueCreate(5, sizeof(rx_buff_t));

    if (uart_queu_receive == NULL)
    {
        ESP_LOGE(TAG, "Failed to create receive queue");

        return COM_ERROR;
    }

    // reset everything
    Com_Recover();
    // Create a task to handle UART events from ISR
    BaseType_t result = xTaskCreate(uart_event_task, "uart_event_task", 3072, NULL, 6, &uart_task_handle);
    if (result != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create UART event task");
        vQueueDelete(uart_queu_receive);
        uart_queu_receive = NULL;
        uart_driver_delete(COM_UART_PORT_NUM);
        vSemaphoreDelete(uart_mutex);
        uart_mutex = NULL;
        return COM_ERROR;
    }

    return COM_OK;
}

// Clean up resources
Com_Status_t Com_DeInit(void)
{

    if (uart_task_handle != NULL)
    {
        vTaskDelete(uart_task_handle);
        uart_task_handle = NULL;
    }

    if (uart_queu_receive != NULL)
    {
        vQueueDelete(uart_queu_receive);
        uart_queu_receive = NULL;
    }

    if (uart_is_driver_installed(COM_UART_PORT_NUM))
    {
        uart_driver_delete(COM_UART_PORT_NUM);
    }

    if (uart_mutex != NULL)
    {
        vSemaphoreDelete(uart_mutex);
        uart_mutex = NULL;
    }

    ESP_LOGI(TAG, "COM driver deinitialized");
    return COM_OK;
}