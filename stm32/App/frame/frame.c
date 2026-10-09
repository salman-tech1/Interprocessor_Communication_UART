/*
 * frame.c
 *
 *  Created on: Oct 7, 2026
 *      Author: Muhmmad Salman
 *
 *      This module includes parse and Frame modules
 */

#include "main.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "frame.h"
#include "com.h"

typedef enum
{
	PARSER_EVENT_ERR = 0xAB,
	PARSER_EVENT_RECVD = 0xBA,
	PARSER_EVENT_TIMEOUT = 0xBB,
	PARSER_RESPONSE_EVENT_TIMEOUT = 0xBC,
} PARSER_EVENT_T;

typedef struct
{
	uint8_t cmd;
	uint8_t seq;
	uint16_t len;
	uint8_t payload[PAYLOAD_SIZE];
} packet_rcvd_t;

typedef void (*parser_cb_t)(PARSER_EVENT_T evnt, packet_rcvd_t *rcvd);

typedef enum
{
	SYNC0_STATE,
	SYNC1_STATE,
	LENGTH_STATE,
	CMD_STATE,
	SEQ_STATE,
	PAYLOAD_STATE,
	CRC_STATE,
} PACKET_STATE_T;

// calculate Crc16 of the data
static uint16_t calculate_crc16(const uint8_t *data, size_t len)
{
	uint16_t crc = 0xFFFFU;
	for (size_t i = 0U; i < len; i++)
	{
		crc ^= (uint16_t)data[i];
		for (uint8_t b = 0U; b < 8U; b++)
		{
			crc = (crc & 0x0001U) ? (uint16_t)((crc >> 1U) ^ 0xA001U) : (uint16_t)(crc >> 1U);
		}
	}
	return crc;
}

///////////////////// Packet Formater Tx Side ////////////////
// variables for the Format Packet
// these are global because we need to use it in other function to send
static uint8_t txbuff[FRAME_SIZE]; // TxBuff only for transmission
static bool packet_formatted = false;
static uint16_t tx_idx = 0;

// Format packet as well calcualte crc  serialize all the data to be send
static void format_packet(const uint8_t *const data, uint8_t len, uint8_t seq,
						  uint8_t cmd)
{

	if ((data == NULL && len > 0) || len > PAYLOAD_SIZE || packet_formatted == true)
	{
		printf("Frame packet format rejected\r\n");
		return;
	}

	tx_idx = 0;
	// Build the frame :
	txbuff[tx_idx++] = SYNC0; // 0x55
	txbuff[tx_idx++] = SYNC1; // 0xAA
	txbuff[tx_idx++] = len;	  // length of payload
	txbuff[tx_idx++] = cmd;	  // command
	txbuff[tx_idx++] = seq;	  // command

	// check the len
	if (len > 0)
	{
		// copy the pay load that we are going to send
		memcpy(&txbuff[tx_idx], data, len);
		tx_idx += len; // move the offset
	}

	//
	uint16_t crc_ = calculate_crc16(txbuff, tx_idx);

	// place lsb first than msb
	txbuff[tx_idx++] = (crc_ & 0xff);
	txbuff[tx_idx++] = (crc_ >> 8);

	// packet formated
	packet_formatted = true;
}

// request sent here
static void packet_send(void)
{

	// if packet is not formatted we need to look
	if (packet_formatted != true || tx_idx < (FRAME_HEADER_SIZE + FRAME_CRC_SIZE))
	{
		printf("Frame send rejected: no formatted packet\r\n");
		return;
	}

	// write the data : we used the tx_idx as the length for the com write function
	com.write(txbuff, tx_idx);

	// reset the packet sending
	tx_idx = 0;
	packet_formatted = false;
	memset(txbuff, 0, FRAME_SIZE);
}

////////////////////// PARSER SIDE (RX Side ) //////////////////////////
static uint8_t payloadBuff[PAYLOAD_SIZE]; // to keep the incoming Payload data

static struct
{
	PACKET_STATE_T state_; // track state in which state the parser are
	uint32_t prev_timer;   // tracks time or timeout
	bool check_timeout;	   // check timeout if it sets to true
	uint16_t len;		   // recieved length
	uint8_t cmd;		   // tracks command
	uint16_t payload_idx;  // index
	uint8_t seq;		   // sequence
	uint16_t crc;		   // calculated crc
	uint8_t crc_index;	   // this is crc_index as crc comes into byte at a time

	uint16_t frame_idx;			// Frame index
	uint8_t rxbuff[FRAME_SIZE]; // to keep all the frames to calculate crc from it
} ps;

static struct
{
	bool is_requested;			 // set the flag that we requested something
	bool frm_init;				 // frame module flag
	uint32_t response_time;		 // Track the frame response time :
	uint8_t retry;				 // count retries
	uint8_t rt_buff[FRAME_SIZE]; // transmit buff : Save the frame
	uint16_t rt_len;			 // Retransmission length

	uint8_t tx_seq; // next seq for OUR requests (auto-increment)

	uint8_t last_rx_req_seq; // last received REQUEST seq
	bool last_rx_req_seq_valid;
	uint8_t last_rx_res_seq; // last received RESPONSE seq
	bool last_rx_res_seq_valid;
} frm;

static bool prsr_init = false;			   // global prser init variable
static parser_cb_t parser_callback = NULL; // it should call a higher level function if frame recieved
static void frame_func(PARSER_EVENT_T evnt, packet_rcvd_t *rcv);
static void frame_retransmit(void);
static void frame_response(const uint8_t *const data, uint16_t len,
						   uint8_t seq, uint8_t cmd);

// This will reset the parser
static void parser_reset(void)
{
	ps.state_ = SYNC0_STATE;
	ps.prev_timer = 0;
	ps.check_timeout = false;
	ps.len = 0;
	ps.cmd = 0;
	ps.payload_idx = 0;
	ps.seq = 0;
	ps.crc = 0;
	ps.crc_index = 0;
	ps.frame_idx = 0;
}

static void parser_init(parser_cb_t cb)
{
	//
	if (prsr_init == true)
		return;

	//
	parser_reset();

	// frame Callback when error or Recieve sometime
	if (parser_callback == NULL)
	{
		// this is the callback it will call
		parser_callback = cb;
	}
	prsr_init = true;
}

static void parse_packets(uint8_t packet)
{

	if (prsr_init == false)
		return;

	switch (ps.state_)
	{
	case SYNC0_STATE:
	{
		// if the incoming packet matches move to next and reset the timer
		if (SYNC0 == packet)
		{

			ps.rxbuff[ps.frame_idx++] = packet; // store the packet for CRC calculation

			ps.prev_timer = HAL_GetTick(); // reset this on every
			ps.check_timeout = true;	   // Now look for the timeout
			ps.state_ = SYNC1_STATE;
		}
	}
	break;
	case SYNC1_STATE:
	{
		// if the incoming packet matches move to next and reset the timer
		if (SYNC1 == packet)
		{

			// Track the frame index for size
			ps.rxbuff[ps.frame_idx++] = packet; // store the packet

			ps.prev_timer = HAL_GetTick(); // reset this on every
			ps.state_ = LENGTH_STATE;	   // move to other state
		}
		// if it sends the packet Sync0 mid section :
		// rset to the Sync1
		else if (packet == SYNC0)
		{
			// Treat this as a new possible start
			ps.frame_idx = 1;
			ps.rxbuff[0] = SYNC0; // store the sync
			ps.prev_timer = HAL_GetTick();
		}
		else
		{
			parser_reset();
		}
	}
	break;
		// check
	case LENGTH_STATE:
	{
		// if the incoming packet matches move to next and reset the timer

		ps.prev_timer = HAL_GetTick(); // reset

		// if the length is bigger reset the parser
		if (packet > PAYLOAD_SIZE)
		{
			parser_reset();
		}
		else
		{

			// store the packets for CRC
			ps.rxbuff[ps.frame_idx++] = packet;

			ps.len = packet; // store the length
			ps.state_ = CMD_STATE;
		}
	}
	break;
	case CMD_STATE:
	{

		// if the incoming packet matches move to next and reset the timer
		ps.prev_timer = HAL_GetTick(); // reset this on every
		switch (packet)
		{
		case CMD_REQ_LED_NUM:
		case CMD_RES_LED_NUM:
		case CMD_REQ_DELAY:
		case CMD_RES_DELAY:
		case CMD_HEART_BEAT_REQ:
		case CMD_HEART_BEAT_RES:
		{

			ps.rxbuff[ps.frame_idx++] = packet; // store the packet
			ps.cmd = packet;					// Store the Command
			ps.state_ = SEQ_STATE;
		}
		break;
		default:
		{
			parser_reset();
		}
		break;
		}
	}
	break;
	case SEQ_STATE:
	{
		// if the incoming packet matches move to next and reset the timer
		ps.prev_timer = HAL_GetTick(); // reset this on every

		ps.seq = packet; // store the sequence

		ps.rxbuff[ps.frame_idx++] = packet; // store the packet

		// if length is 0 means no Payload data so jump straight to CRC
		if (ps.len == 0)
		{
			ps.payload_idx = 0;
			ps.state_ = CRC_STATE;
		}
		// otherwise reset the payload index and jump to payload
		else
		{
			ps.payload_idx = 0; // reset the payload index
			ps.state_ = PAYLOAD_STATE;
		}
	}
	break;

	case PAYLOAD_STATE:
	{
		// if the incoming packet matches move to next and reset the timer
		ps.prev_timer = HAL_GetTick(); // reset this on every

		// store the Packets as it arrives
		payloadBuff[(ps.payload_idx)++] = packet; // keep the incoming data
		ps.rxbuff[ps.frame_idx++] = packet;		  // store the packet

		// track the index if it reaches the len than jump to another state
		if (ps.payload_idx >= ps.len)
		{
			// change state :
			ps.state_ = CRC_STATE;
		}
	}
	break;

	case CRC_STATE:
	{

		ps.prev_timer = HAL_GetTick();

		if (ps.crc_index == 0)
		{
			// first CRC packet LSB move it
			ps.crc = packet;
			// get other byte:
			ps.crc_index = 1;
		}
		else
		{
			ps.crc |= ((uint16_t)packet << 8);
			uint16_t calculated_crc = calculate_crc16(ps.rxbuff,
													  ps.frame_idx);
			if (calculated_crc == ps.crc)
			{
				//
				static packet_rcvd_t pckt__; // static so it survives the call
				pckt__.cmd = ps.cmd;		 // command
				pckt__.seq = ps.seq;		 // sequence
				pckt__.len = ps.len;		 // length

				// copy the payload data
				memcpy(pckt__.payload, payloadBuff, ps.len);

				if (parser_callback)
				{
					parser_callback(PARSER_EVENT_RECVD, &pckt__);
				}
			}
			else
			{
				// CRC Error : report Error
				parser_callback(PARSER_EVENT_ERR, NULL);
			}
			parser_reset();
		}

		break;
	}
	}
}

static bool is_response_cmd(uint8_t cmd)
{
	return (cmd == CMD_RES_LED_NUM ||
			cmd == CMD_RES_DELAY ||
			cmd == CMD_HEART_BEAT_RES);
}

static void frame_reset(void)
{
	frm.is_requested = false;
	frm.response_time = 0;
	frm.retry = 0;
	frm.rt_len = 0;

	// Sequence
	frm.tx_seq = 1;

	frm.last_rx_req_seq = 0;
	frm.last_rx_req_seq_valid = false;
	frm.last_rx_res_seq = 0;
	frm.last_rx_res_seq_valid = false;
}

// Init Frame Module:
void frame_init(void)
{
	if (frm.frm_init != false)
		return;

	frame_reset();
	// Init frame and Parser
	parser_init(frame_func);

	frm.frm_init = true;
}

// Send Request :
void frame_request(const uint8_t *const data, uint16_t len, uint8_t cmd)
{
	if (frm.frm_init != true || len > PAYLOAD_SIZE || (data == NULL && len > 0U))
		return;

	uint8_t seq = frm.tx_seq++;
	format_packet(data, (uint8_t)len, seq, cmd);

	// Save the frame BEFORE packet_send() clears txbuff
	// store the length and buffer
	// Save the frame for retransmission
	frm.rt_len = tx_idx;
	memcpy(frm.rt_buff, txbuff, frm.rt_len);

	packet_send();

	frm.is_requested = true;
	frm.response_time = HAL_GetTick();
	frm.retry = 0;
}

// Retry sending
static void frame_retransmit(void)
{
	com.write(frm.rt_buff, frm.rt_len);
	frm.response_time = HAL_GetTick(); // restart the timer for another timeout
}

static void frame_response(const uint8_t *const data, uint16_t len,
						   uint8_t seq, uint8_t cmd)
{
	if (!frm.frm_init || len > PAYLOAD_SIZE || (data == NULL && len > 0U))
		return;

	format_packet(data, (uint8_t)len, seq, cmd);
	packet_send();

	/* No retry for responses */
}

static void frame_func(PARSER_EVENT_T evnt, packet_rcvd_t *rcv)
{
	if (evnt == PARSER_EVENT_ERR)
	{
		printf("Frame Error\r\n");
		return;
	}

	if (evnt == PARSER_EVENT_TIMEOUT)
	{
		printf("Parser Timeout\r\n");
	}
	else if (evnt == PARSER_RESPONSE_EVENT_TIMEOUT && frm.is_requested)
	{
		frm.retry++;
		if (frm.retry <= MAX_RETRIES)
		{
			frame_retransmit();
		}
		else
		{
			frm.retry = 0;
			frm.is_requested = false;
			printf("MAX Retry Reached\r\n");
		}
	}

	if (evnt != PARSER_EVENT_RECVD || rcv == NULL)
		return;

	if (is_response_cmd(rcv->cmd))
	{
		if (frm.last_rx_res_seq_valid && rcv->seq == frm.last_rx_res_seq)
		{
			printf("Duplicate response seq=%u - ignored\r\n", rcv->seq);
			return;
		}
		frm.last_rx_res_seq = rcv->seq;
		frm.last_rx_res_seq_valid = true;
	}
	else
	{
		if (frm.last_rx_req_seq_valid && rcv->seq == frm.last_rx_req_seq)
		{
			printf("Duplicate request seq=%u - re-responding\r\n", rcv->seq);
		}
		frm.last_rx_req_seq = rcv->seq;
		frm.last_rx_req_seq_valid = true;
	}

	if (is_response_cmd(rcv->cmd))
	{
		if (!frm.is_requested)
		{
			printf("Unsolicited/late response seq=%u - ignored\r\n", rcv->seq);
			return;
		}
		if (rcv->seq != (uint8_t)(frm.tx_seq - 1U))
		{
			printf("Stale response seq=%u (expected %u) - ignored\r\n",
				   rcv->seq, (uint8_t)(frm.tx_seq - 1U));
			return;
		}
		frm.retry = 0;
		frm.is_requested = false;
	}

	switch (rcv->cmd)
	{
	case CMD_REQ_LED_NUM:
	{
		uint8_t num_leds = 8U;
		frame_response(&num_leds, 1U, rcv->seq, CMD_RES_LED_NUM);
		break;
	}
	case CMD_REQ_DELAY:
	{
		uint8_t delay_val = 100U;
		frame_response(&delay_val, 1U, rcv->seq, CMD_RES_DELAY);
		break;
	}
	case CMD_HEART_BEAT_REQ:
		frame_response(NULL, 0U, rcv->seq, CMD_HEART_BEAT_RES);
		break;
	case CMD_RES_LED_NUM:
		if (rcv->len > 0U)
			printf("Got LED_NUM response: %u\r\n", rcv->payload[0]);
		else
			printf("LED_NUM response has no payload\r\n");
		break;
	case CMD_RES_DELAY:
		if (rcv->len > 0U)
			printf("Got DELAY response: %u\r\n", rcv->payload[0]);
		else
			printf("DELAY response has no payload\r\n");
		break;
	case CMD_HEART_BEAT_RES:
		printf("Got HEARTBEAT response\r\n");
		break;
	default:
		printf("Unknown cmd: 0x%02X\r\n", rcv->cmd);
		break;
	}
}

void poll_packets(void)
{

	// if parser is not initialized return immediately
	if (prsr_init == false)
		return;
	// We need each byte to be read exactly as it arrives
	uint8_t rd_buff[20];
	uint16_t rd_len = 0;

	// if it reads data than parse it
	if ((rd_len = com.read(rd_buff, sizeof(rd_buff))) > 0)
	{
		for (unsigned int i = 0; i < rd_len; ++i)
		{
			// Read bytes and Parse it one by one
			parse_packets(rd_buff[i]);
		}
	}

	// timeout for the parser : if the first byte arrives and than stop arriving further or mid packet missing than reset the parser
	if (ps.check_timeout && (HAL_GetTick() - ps.prev_timer >= PACKET_PARSE_TIMEOUT))
	{
		// tell Application layer as well about this
		parser_callback(PARSER_EVENT_TIMEOUT, NULL);
		// reset Parser if timeouts
		parser_reset();
	}

	// if frame is requested and during the timeout there is no response than retransmit
	if (frm.is_requested && (HAL_GetTick() - frm.response_time >= FRAME_RESPONSE_TIMEOUT))
	{
		// tell
		parser_callback(PARSER_RESPONSE_EVENT_TIMEOUT, NULL);
	}
}
