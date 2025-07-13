#include "udp_server.h"

const char *TAG_UDP = "UDP_ex";
const char *TAG_INSTR = "Instr_Task";
static QueueHandle_t xInstrQueue = NULL;
/**
 * @brief Queue to store instructions to be executed
 */

bool decode_udp_mesage(char *msg, instr_t *new_instr){
    char copia[128]; // Creamos una copia para no modificar la original
    strcpy(copia, msg);
    
    // Usamos strtok para dividir la cadena por comas
    char* token = strtok(copia, ",");
    if (token) new_instr->cmd = token[0]; // Tomamos el primer carácter

    switch (new_instr->cmd)
    {
    case 'L':
        /* code */
        token = strtok(NULL, ",");
        if (token) new_instr->direction = atoi(token) != 0;
        
        token = strtok(NULL, ",");
        if (token) new_instr->velocity = (float)atof(token) != 0;

        token = strtok(NULL, ",");
        if (token) new_instr->distance = (float)atof(token);

        token = strtok(NULL, ",");
        if (token) new_instr->angle = (float)atof(token);
        break;
    case 'R':
        token = strtok(NULL, ",");
        if (token) new_instr->direction = atoi(token) != 0;
        
        token = strtok(NULL, ",");
        if (token) new_instr->velocity = (float)atof(token) != 0;

        token = strtok(NULL, ",");
        if (token) new_instr->distance = (float)atof(token);

        token = strtok(NULL, ",");
        if (token) new_instr->angle = (float)atof(token);
        break;
    case 'C':
        token = strtok(NULL, ",");
        if (token) new_instr->direction = atoi(token) != 0;
        
        token = strtok(NULL, ",");
        if (token) new_instr->velocity = (float)atof(token);

        token = strtok(NULL, ",");
        if (token) new_instr->distance = (float)atof(token);

        token = strtok(NULL, ",");
        if (token) new_instr->angle = (float)atof(token);
        break;
    default:
        ESP_LOGE(TAG_UDP, "Instr unknow, unable to decode");
        return 1;
        break;
    }
    
    return 0;
}
/*
void vTaskExecuteInstr(void *pvParameters){
    instr_t instr_2_process;
    ESP_LOGE(TAG_INSTR,"Tarea Ejecucion Creada");
    while (1)
    {
        if(xInstrQueue != NULL){
            if(xQueueReceive(xInstrQueue,&instr_2_process,pdMS_TO_TICKS(500)) == pdPASS){
                ESP_LOGI(TAG_UDP, "Instruction execute");
                ESP_LOGI(TAG_INSTR, "\nCMD : %c\nDirection : %d\nVel: %hu\nAngle : %hu\n", instr_2_process.cmd,instr_2_process.direction,instr_2_process.velocity,instr_2_process.angle);
                vTaskDelay(pdMS_TO_TICKS(1e5));
            }
            else{
                 ESP_LOGI(TAG_UDP, "Instruction queue empty");
            }
        }
        else{
            ESP_LOGE(TAG_UDP, "Queue doesnt exist");
            vTaskDelay(pdMS_TO_TICKS(1e4));
        }
    }
}
*/
    
void vTaskUdpServer(void *pvParameters){
    char rx_buffer[128];
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;

    instr_t instr_data;

    while (1) {

        if (addr_family == AF_INET) {
            struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
            dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
            dest_addr_ip4->sin_family = AF_INET;
            dest_addr_ip4->sin_port = htons(UDP_PORT);
            ip_protocol = IPPROTO_IP;
        } else if (addr_family == AF_INET6) {
            bzero(&dest_addr.sin6_addr.un, sizeof(dest_addr.sin6_addr.un));
            dest_addr.sin6_family = AF_INET6;
            dest_addr.sin6_port = htons(UDP_PORT);
            ip_protocol = IPPROTO_IPV6;
        }

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG_UDP, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG_UDP, "Socket created");

#if defined(CONFIG_LWIP_NETBUF_RECVINFO) && !defined(CONFIG_EXAMPLE_IPV6)
        int enable = 1;
        lwip_setsockopt(sock, IPPROTO_IP, IP_PKTINFO, &enable, sizeof(enable));
#endif

#if defined(CONFIG_EXAMPLE_IPV4) && defined(CONFIG_EXAMPLE_IPV6)
        if (addr_family == AF_INET6) {
            // Note that by default IPV6 binds to both protocols, it is must be disabled
            // if both protocols used at the same time (used in CI)
            int opt = 1;
            setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
            setsockopt(sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
        }
#endif
        // Set timeout
        struct timeval timeout;
        timeout.tv_sec = 60;
        timeout.tv_usec = 0;
        setsockopt (sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

        int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE(TAG_UDP, "Socket unable to bind: errno %d", errno);
        }
        ESP_LOGI(TAG_UDP, "Socket bound, port %d", UDP_PORT);

        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t socklen = sizeof(source_addr);

        #if defined(CONFIG_LWIP_NETBUF_RECVINFO) && !defined(CONFIG_EXAMPLE_IPV6)
        struct iovec iov;
        struct msghdr msg;
        struct cmsghdr *cmsgtmp;
        u8_t cmsg_buf[CMSG_SPACE(sizeof(struct in_pktinfo))];

        iov.iov_base = rx_buffer;
        iov.iov_len = sizeof(rx_buffer);
        msg.msg_control = cmsg_buf;
        msg.msg_controllen = sizeof(cmsg_buf);
        msg.msg_flags = 0;
        msg.msg_iov = &iov;
        msg.msg_iovlen = 1;
        msg.msg_name = (struct sockaddr *)&source_addr;
        msg.msg_namelen = socklen;
        #endif

        while (1) {
            ESP_LOGI(TAG_UDP, "Waiting for data");
            #if defined(CONFIG_LWIP_NETBUF_RECVINFO) && !defined(CONFIG_EXAMPLE_IPV6)
            int len = recvmsg(sock, &msg, 0);
            #else
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
            #endif
            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG_UDP, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                // Get the sender's ip address as string
                if (source_addr.ss_family == PF_INET) {
                    inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
                    #if defined(CONFIG_LWIP_NETBUF_RECVINFO) && !defined(CONFIG_EXAMPLE_IPV6)
                    for ( cmsgtmp = CMSG_FIRSTHDR(&msg); cmsgtmp != NULL; cmsgtmp = CMSG_NXTHDR(&msg, cmsgtmp) ) {
                        if ( cmsgtmp->cmsg_level == IPPROTO_IP && cmsgtmp->cmsg_type == IP_PKTINFO ) {
                            struct in_pktinfo *pktinfo;
                            pktinfo = (struct in_pktinfo*)CMSG_DATA(cmsgtmp);
                            ESP_LOGI(TAG_UDP, "dest ip: %s", inet_ntoa(pktinfo->ipi_addr));
                        }
                    }
                    #endif
                } else if (source_addr.ss_family == PF_INET6) {
                    inet6_ntoa_r(((struct sockaddr_in6 *)&source_addr)->sin6_addr, addr_str, sizeof(addr_str) - 1);
                }

                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string...
                ESP_LOGI(TAG_UDP, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TAG_UDP, "%s", rx_buffer);

                
                int err = sendto(sock, rx_buffer, len, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
                if (err < 0) {
                    ESP_LOGE(TAG_UDP, "Error occurred during sending: errno %d", errno);
                    break;
                }
                
                if(!decode_udp_mesage(rx_buffer,&instr_data)){
                    xQueueSendToBack(xInstrQueue,(void *) &instr_data,(TickType_t) 0);
                }
                
            }
        }

        if (sock != -1) {
            ESP_LOGE(TAG_UDP, "Shutting down socket and restarting...");
            ESP_LOGE(TAG_UDP, "Num of Instr on queue : %d",uxQueueMessagesWaiting(xInstrQueue));
            shutdown(sock, 0);
            close(sock);

        }
    }
    vTaskDelete(NULL);
}

bool init_comm(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_sta();

    instr_t intr_test;
    xInstrQueue = xQueueCreate(10,sizeof(intr_test));

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    xTaskCreate(vTaskUdpServer, "udp_server", 4096, (void*)AF_INET, 9, NULL);
    //xTaskCreate(vTaskExecuteInstr,"Exe Instr", 2048, NULL, 4, NULL);
    return 0;
}

QueueHandle_t return_queue_handle(void){
    return xInstrQueue;
}