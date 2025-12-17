/**
 * @file wifi_task.c
 * @brief Simple task to initialize WiFi and signal connectivity.
 */
#include "wifi.h"

void wifi_task(void *pvParameters)
{
    wifi_init_sta();   // Tu función original

    // Señalizar que WiFi está listo
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);

    vTaskDelete(NULL);
}
