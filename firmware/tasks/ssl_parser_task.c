/**
 * @file ssl_parser_task.c
 * @brief Task that decodes protobuf SSL frames and dispatches detection/geometry processing.
 */
#include "ssl_parser.h"

/**
 * @brief Task that decodes queued UDP packets into protobuf wrappers and dispatches handlers.
 *
 * Waits on `xPktQueue` for incoming UDP packets, decodes them with nanopb and
 * calls `process_detection` / `process_geometry` as appropriate.
 *
 * @param pvParameters FreeRTOS task parameter (unused)
 */
void ssl_parser_task(void *pvParameters)
{
    udp_packet_t pkt;

    while (1) {
        if (xQueueReceive(xPktQueue, &pkt, portMAX_DELAY) == pdTRUE) {

            SSL_WrapperPacket wrapper = SSL_WrapperPacket_init_zero;
            pb_istream_t stream = pb_istream_from_buffer(pkt.data, pkt.len);

            bool status = pb_decode(&stream, SSL_WrapperPacket_fields, &wrapper);

            if (!status) {
                ESP_LOGE(TAG_WIFI, "Error protobuf: %s", PB_GET_ERROR(&stream));
                continue;
            }

            if (wrapper.has_detection) {
                process_detection(&wrapper.detection);
            }

            if (wrapper.has_geometry) {
                process_geometry(&wrapper.geometry);
            }
        }
    }
} 
