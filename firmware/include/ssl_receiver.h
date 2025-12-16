/**
 * @file ssl_receiver.h
 * @brief UDP receiver task and packet queue definitions for SSL vision data.
 *
 * The receiver reads UDP packets and enqueues them for parsing by the parser task.
 */
#ifndef __SSL_RECEIVER_H__
#define __SSL_RECEIVER_H__

#include "ssl_defs.h"
#include <stdint.h>

void ssl_receiver_task(void *pvParameters);

#endif // __SSL_RECEIVER_H__