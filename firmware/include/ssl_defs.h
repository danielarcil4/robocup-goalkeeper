/**
 * @file ssl_defs.h
 * @brief Common definitions and types used by the SSL networking stack.
 *
 * Contains packet types, shared queues and event group handles used by
 * the receiver and parser tasks.
 */
#ifndef __SSL_DEF_H__
#define __SSL_DEF_H__

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "config_utils.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"

#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/igmp.h"

#include "pb_decode.h"
#include "messages_robocup_ssl_wrapper.pb.h"
#include "messages_robocup_ssl_detection.pb.h"
#include "messages_robocup_ssl_geometry.pb.h"

extern QueueHandle_t xPktQueue;
extern EventGroupHandle_t s_wifi_event_group;

static const char *TAG_WIFI = "SSL_VISION";

typedef struct {
    size_t len;
    uint8_t data[MAX_PACKET_SIZE];
} udp_packet_t;

#endif // __SSL_DEF_H__