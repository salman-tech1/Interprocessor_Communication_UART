/*
 * frame.h
 *
 *  Created on: Oct 7, 2026
 *      Author: Muhmmad Salman
 */

#ifndef FRAME_H
#define FRAME_H

#include <stdint.h>

/* Packet format:
 * SYNC0 | SYNC1 | LENGTH | CMD | SEQ | PAYLOAD | CRC (CRC16, low byte first)
 */

#define PACKET_PARSE_TIMEOUT 2000U  // Mid Frame Silence Timeout
#define FRAME_RESPONSE_TIMEOUT 3000U // no response timeout in milliseconds

#define PAYLOAD_SIZE 200U //  bytes

#define FRAME_HEADER_SIZE 5U // SYNC0 + SYNC1 + LENGTH + CMD + SEQ
#define FRAME_CRC_SIZE 2U    // 16-bit CRC
#define FRAME_SIZE (FRAME_HEADER_SIZE + PAYLOAD_SIZE + FRAME_CRC_SIZE)

#define SYNC0 0x55U
#define SYNC1 0xAAU

#define MAX_RETRIES 3U

// COMMANDS
#define CMD_REQ_LED_NUM 0x10U
#define CMD_RES_LED_NUM 0x11U

#define CMD_REQ_DELAY 0x20U
#define CMD_RES_DELAY 0x21U

#define CMD_HEART_BEAT_REQ 0x30U
#define CMD_HEART_BEAT_RES 0x31U

/* Initialize the frame parser after Com_Init() succeeds. */
void frame_init(void);

/* Send a request. Responses are parsed from poll_packets(). */
void frame_request(const uint8_t *const data, uint16_t len, uint8_t cmd);

/* Call regularly from a task to process received data and timeout handling. */
void poll_packets(void);

#endif /* FRAME_H */
