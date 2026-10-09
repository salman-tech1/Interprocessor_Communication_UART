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

static char *TAG = "MAIN";

void com_event_handler(Com_Event_t event) {
    if (event == COM_EVENT_RX_DATA) {
        ESP_LOGI(TAG, "COM Event: New data received");
    } else if (event == COM_EVENT_ERROR) {
        ESP_LOGE(TAG, "COM Event: Error occurred, recovering...");
        Com_Recover();
    }
}

void app_main(void) {
    // Register event callback before initialization
    Com_RegisterEventCallback(com_event_handler);
    
    // Initialize COM
    Com_Status_t init_status = Com_Init();
    if (init_status != COM_OK) {
        ESP_LOGE(TAG, "Failed to initialize COM driver");
        return;
    }
    
    char read_buff[512];
    uint32_t len;
    bool rdy_to_send = false ; 
     const char buff[30] = "Salman \r\n";

    for (;;) {
    Com_Status_t st ; 
        if(rdy_to_send == true )
        {
             st = Com_Send((uint8_t *)buff, strlen(buff));
        
        if (st == COM_OK) {
            ESP_LOGI(TAG, "Bytes Sent: %s", buff);
            rdy_to_send = false ; 
        } else {
            ESP_LOGE(TAG, "Sending Error: %d", st);
        }
        
         }
        
        if(rdy_to_send == false )
        {
         len = sizeof(read_buff);
          st = Com_recieve((uint8_t *)read_buff, &len);
         if (st == COM_OK && len > 0) {
            ESP_LOGI(TAG, "Received %d bytes: %.*s", len, len, read_buff);
              rdy_to_send = true ; 
        }
      
        }
     

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}