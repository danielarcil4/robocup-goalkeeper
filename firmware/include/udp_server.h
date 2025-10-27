/* BSD Socket API Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "wifi_connect.h"

#ifndef __UDP_SERVER_H__
#define __UDP_SERVER_H__


#define WIFI_SSID "Howlers - UdeA"
#define WIFI_PASS "9876543210"
#define MAXIMUM_RETRY  10
#define UDP_PORT 3333


#define INSTR_QUEUE_LENGTH 


/**
 * @brief Instruction structure 
 */

typedef struct
{
    char cmd;
    int8_t direction;     // Foward/CW 1 & Backward/CCW 0 
    float velocity;  // Movement velocity (linear or angular)
    float distance;  // Distance or Circle Radius
    float angle;     // Movement/Rotation angle
}instr_t;



/**
 * @brief Decode a message received via the UDP protocol into a command instruction to execute
 * 
 * @param msg message to decode (char buffer)
 * @param new_instr Data struct where save from decoded message
 * 
 * @return True if the message contains a valid intruction, False otherwise.
 */
bool decode_udp_mesage(char *msg, instr_t *new_instr);

bool init_comm(void);


/*void vTaskExecuteInstr(void *pvParameters);*/

void vTaskUdpServer(void *pvParameters);

QueueHandle_t return_queue_handle(void);


#endif