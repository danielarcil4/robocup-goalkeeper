/**
 * @file wifi.h
 * @brief WiFi tasks and initialization for RoboCup Goalkeeper firmware.
 *
 * Provides `wifi_task` used by the system and `wifi_init_sta` for station mode setup.
 */
#ifndef __WIFI_H__
#define __WIFI_H__
#include "ssl_defs.h"

void wifi_task(void *pvParameters);
void wifi_init_sta(void);

#endif // __WIFI_H__