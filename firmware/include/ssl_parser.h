/**
 * @file ssl_parser.h
 * @brief Parser for SSL (Small Size League) vision frames.
 *
 * Provides functions to parse detection and geometry frames into internal structures.
 */
#ifndef __SSL_PARSER_H__
#define __SSL_PARSER_H__
#include "ssl_defs.h"

void ssl_parser_task(void *pvParameters);
void process_detection(const SSL_DetectionFrame *detection);
void process_geometry(const SSL_GeometryData *geometry);

#endif // __SSL_PARSER_H__