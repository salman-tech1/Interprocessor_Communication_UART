/*
 * frame.h
 *
 *  Created on: Oct 7, 2026
 *      Author: Muhmmad Salman
 */

#ifndef FRAME_H
#define FRAME_H

#include <stdint.h>

/* Packet format  :
 * SYNC | LENGTH | CMD | SEQ | PAYLOAD | CRC
 */

#define PACKET_PARSE_TIMEOUT 2000U	// Mid Frame Silence Timeout
#define FRAME_RESPONSE_TIMEOUT 3000 // no response at all timeout

#define PAYLOAD_SIZE 200U //  bytes

#define FRAME_HEADER_SIZE 5U // syn1 + syn2 + len + seq + cmd
#define FRAME_CRC_SIZE 2U	 // 16bit crc = crc && 0xff | crc >> 8
#define FRAME_SIZE (FRAME_HEADER_SIZE + PAYLOAD_SIZE + FRAME_CRC_SIZE)

#define SYNC0 0x55U
#define SYNC1 0xAAU

//
#define MAX_RETRIES 3U

// COMMANDS
#define CMD_REQ_LED_NUM 0x10U
#define CMD_RES_LED_NUM 0x11U

#define CMD_REQ_DELAY 0x20U
#define CMD_RES_DELAY 0x21U

#define CMD_HEART_BEAT_REQ 0x30U
#define CMD_HEART_BEAT_RES 0x31U

void frame_init(void);
void frame_request(const uint8_t *const data, uint16_t len, uint8_t cmd);
void poll_packets(void);


#endif /* FRAME_H */
