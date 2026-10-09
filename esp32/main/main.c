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

static const char *TAG = "MAIN";

void com_event_handler(Com_Event_t event)
{
    if (event == COM_EVENT_RX_DATA)
    {
        //  ESP_LOGI(TAG, "COM Event: New data received ");
    }
    else if (event == COM_EVENT_ERROR)
    {
        ESP_LOGE(TAG, "COM error occurred; attempting recovery");
        Com_Status_t status = Com_Recover();
        if (status != COM_OK)
        {
            ESP_LOGE(TAG, "COM recovery failed: %d", (int)status);
        }
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

    frame_init();
    ESP_LOGI(TAG, "Frame module initialized; sending heartbeat requests");

    TickType_t last_request = xTaskGetTickCount() - pdMS_TO_TICKS(15000U);
    for (;;)
    {
        poll_packets();

        if ((xTaskGetTickCount() - last_request) >= pdMS_TO_TICKS(15000U))
        {
            frame_request(NULL, 0U, CMD_HEART_BEAT_REQ);
            last_request = xTaskGetTickCount();
        }

        vTaskDelay(pdMS_TO_TICKS(10U));
    }
}