/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <stdbool.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "com.h"
#include "frame.h"

static char *TAG = "MAIN";

uint8_t seq = 0;

void com_event_handler(Com_Event_t event)
{
    if (event == COM_EVENT_RX_DATA)
    {
        ESP_LOGI(TAG, "COM Event: New data received ");
    }
    else if (event == COM_EVENT_ERROR)
    {
        //  ESP_LOGE(TAG, "COM Event: Error occurred, recovering...");
        Com_Recover();
    }
}

void app_main(void)
{
    // Register event callback before initialization
    Com_RegisterEventCallback(com_event_handler);

    // Initialize COM
    Com_Status_t init_status = Com_Init();
    if (init_status != COM_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize COM driver");
        return;
    }

    char uart_buff[] = "Salman\r\n";
    uint8_t recv_buff[50];
    uint8_t len = sizeof(uart_buff);
    uint32_t rec_len = sizeof(recv_buff);

    Com_Status_t st = Com_Send((uint8_t *)uart_buff, len);
    if (st == COM_OK)
    {
        //
        Com_recieve(recv_buff, &rec_len);
        printf("%s \n", recv_buff);
    }

    for (;;)
    {

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}