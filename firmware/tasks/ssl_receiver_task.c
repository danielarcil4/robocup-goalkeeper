/**
 * @file ssl_receiver_task.c
 * @brief Task that receives multicast SSL vision packets and enqueues them for parsing.
 */
/**
 * @brief Task entry point: receives multicast UDP packets and pushes to the packet queue.
 *
 * The task joins the multicast group, blocks on recvfrom, and forwards received packets
 * into `xPktQueue` for the parser task.
 *
 * @param pvParameters FreeRTOS task parameter (unused)
 */
#include "ssl_receiver.h"

void ssl_receiver_task(void *pvParameters)
{
    int sock;
    struct sockaddr_in saddr;
    struct ip_mreq imreq;

    udp_packet_t pkt;

    sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG_WIFI, "socket fail");
        vTaskDelete(NULL);
    }

    int reuse = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    memset(&saddr, 0, sizeof(saddr));
    saddr.sin_family = AF_INET;
    saddr.sin_port = htons(MULTICAST_PORT);
    saddr.sin_addr.s_addr = htonl(INADDR_ANY);
    bind(sock, (struct sockaddr *)&saddr, sizeof(saddr));

    imreq.imr_multiaddr.s_addr = inet_addr(MULTICAST_IPV4_ADDR);
    imreq.imr_interface.s_addr = htonl(INADDR_ANY);
    setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, &imreq, sizeof(imreq));

    ESP_LOGI(TAG_WIFI, "Multicast listo");

    while (1) {
        struct sockaddr_in src;
        socklen_t slen = sizeof(src);

        int len = recvfrom(sock, pkt.data, MAX_PACKET_SIZE, 0,
                           (struct sockaddr *)&src, &slen);

        if (len <= 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        pkt.len = len;

        // Enviar paquete a la cola
        if (xQueueSend(xPktQueue, &pkt, 0) != pdTRUE) {
            ESP_LOGW(TAG_WIFI, "Cola llena, paquete descartado");
        }
    }
}
