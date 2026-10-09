/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdbool.h>
#include <stdio.h>

#include "com.h"
#include "retarget.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
#define PACKET_PARSE_TIMEOUT 2000	// Mid Frame Silence Timeout
#define FRAME_RESPONSE_TIMEOUT 3000 // no response at all timeout

#define PAYLOAD_SIZE 200 //  bytes

#define FRAME_HEADER_SIZE 5U // syn1 + syn2 + len + seq + cmd
#define FRAME_CRC_SIZE 2U	 // 16bit crc = crc && 0xff | crc >> 8
#define FRAME_SIZE (FRAME_HEADER_SIZE + PAYLOAD_SIZE + FRAME_CRC_SIZE)

#define SYNC0 0x55
#define SYNC1 0xAA

//
#define MAX_RETRIES 3

// COMMANDS
#define CMD_REQ_LED_NUM 0x10
#define CMD_RES_LED_NUM 0x11

#define CMD_REQ_DELAY 0x20
#define CMD_RES_DELAY 0x21

#define CMD_HEART_BEAT_REQ 0x30
#define CMD_HEART_BEAT_RES 0x31

/* Packet format  :
 * SYNC | LENGTH | CMD | SEQ | PAYLOAD | CRC
 */

typedef enum
{
	PARSER_EVENT_ERR = 0xAB,
	PARSER_EVENT_RECVD = 0xBA,
	PARSER_EVENT_TIMEOUT = 0xBB,
	PARSER_RESPONSE_EVENT_TIMEOUT = 0xBC,

} PARSER_EVENT_T;

/* Received packet structure */
typedef struct
{
	uint8_t cmd;				   // command it recieved
	uint8_t seq;				   // seq number
	uint16_t len;				   // length
	uint8_t payload[PAYLOAD_SIZE]; // the PAYLOAD
} packet_rcvd_t;

// track
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

/*  we need to include the data it recieved like , cmd , Length , Payload , and so */
typedef void (*parser_cb_t)(PARSER_EVENT_T evnt, packet_rcvd_t *rcvd);

/* Parser state-machine states */
typedef enum
{
	SYNC0_STATE,   // 1 byte
	SYNC1_STATE,   // 1 byte
	LENGTH_STATE,  // 1 byte
	CMD_STATE,	   // 1 byte
	SEQ_STATE,	   // 1 byte
	PAYLOAD_STATE, //
	CRC_STATE,	   // 2 bytes
} PACKET_STATE_T;

// This will be a packet to send :

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
void format_packet(const uint8_t *const data, uint8_t len, uint8_t seq,
				   uint8_t cmd)
{

	if ((data == NULL && len > 0) || len > PAYLOAD_SIZE || packet_formatted == true)
	{
		HAL_UART_Transmit(&huart2, (uint8_t *)"format_packet\r\n",
						  strlen("format_packet\r\n"), 0xff);
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
void packet_send()
{

	// if packet is not formatted we need to look
	if (packet_formatted != true || tx_idx < (FRAME_HEADER_SIZE + FRAME_CRC_SIZE))
	{
		HAL_UART_Transmit(&huart2, (uint8_t *)"packet_send\r\n",
						  strlen("packet_send\r\n"), 0xff);
		return;
	}

	// write the data : we used the tx_idx as the length for the com write function
	com.write(txbuff, tx_idx);

	// reset the packet sending
	tx_idx = 0;
	packet_formatted = false;
	memset(txbuff, 0, FRAME_SIZE);
}
/////////////////////////////////////////////

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

static bool prsr_init = false;			   // global prser init variable
static parser_cb_t parser_callback = NULL; // it should call a higher level function if frame recieved

// This will reset the parser
void parser_reset()
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

void parser_init(parser_cb_t cb)
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

void parse_packets(uint8_t packet)
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

/**
 * Call periodically from main loop / RTOS task.
 * Reads UART, feeds bytes to parser, checks both:
 *   1) parser mid-frame timeout
 *   2) IPC response timeout (no bytes at all)
 */
void poll_packets(void);

void frame_init();
void frame_request(const uint8_t *const data, uint16_t len, uint8_t cmd);
void frame_retransmit();
void frame_response(const uint8_t *const data, uint16_t len,
					uint8_t seq, uint8_t cmd);

///////////////////////////  sits between application and Parser  IPC LAYER /////

static bool is_response_cmd(uint8_t cmd)
{
	return (cmd == CMD_RES_LED_NUM ||
			cmd == CMD_RES_DELAY ||
			cmd == CMD_HEART_BEAT_RES);
}

// get called from parse packets
void frame_func(PARSER_EVENT_T evnt, packet_rcvd_t *rcv)
{

	// Frame error
	if (evnt == PARSER_EVENT_ERR)
	{
		// Send to application layer error
		printf("Frame Error\r\n");
		return;
	}
	// Check if timeout happened : means there is no response
	else if (evnt == PARSER_EVENT_TIMEOUT)
	{
		// tell upper layer
		printf("Parser Timeout \r\n");
		// do not reset the parser bcause inside polling we already resetting
	}
	// Now here deal with retry : check if frame is requested and timeout heppend retry upto Max before Giving up
	else if (evnt == PARSER_RESPONSE_EVENT_TIMEOUT && frm.is_requested == true)
	{
		// Response Timeout Now Retry
		frm.retry++;

		// check if retries are pending
		if (frm.retry <= MAX_RETRIES)
		{
			frame_retransmit();
		}
		else
		{
			frm.retry = 0;
			frm.is_requested = false;
			printf("MAX Retry Reached \r\n");
			// TODO: notify application layer
		}
	}

	// if you recieved frames
	if (evnt == PARSER_EVENT_RECVD && rcv != NULL)
	{

		if (is_response_cmd(rcv->cmd))
		{
			/* Check against last received RESPONSE seq */
			if (frm.last_rx_res_seq_valid && rcv->seq == frm.last_rx_res_seq)
			{
				printf("Duplicate response seq=%u — ignored\r\n", rcv->seq);
				return;
			}
			frm.last_rx_res_seq = rcv->seq;
			frm.last_rx_res_seq_valid = true;
		}
		else
		{
			/* Check against last received REQUEST seq */
			if (frm.last_rx_req_seq_valid && rcv->seq == frm.last_rx_req_seq)
			{
				/* Duplicate request — other side missed our response */
				printf("Duplicate request seq=%u — re-responding\r\n", rcv->seq);
				/* Fall through to re-handle */
			}
			frm.last_rx_req_seq = rcv->seq;
			frm.last_rx_req_seq_valid = true;
		}

		/*
			Check if we sent a request and we got a response back
			and  that response is not equal to the sequence of the request we sent

		*/
		if (is_response_cmd(rcv->cmd))
		{
			if (!frm.is_requested)
			{
				printf("Unsolicited/late response seq=%u — ignored\r\n", rcv->seq);
				return;
			}
			if (rcv->seq != (frm.tx_seq - 1))
			{
				printf("Stale response seq=%u (expected %u) — ignored\r\n",
					   rcv->seq, frm.tx_seq - 1);
				return;
			}
			frm.retry = 0;
			frm.is_requested = false;
		}

		switch (rcv->cmd)
		{
		case CMD_REQ_LED_NUM:
		{
			// Gather data
			uint8_t num_leds = 8; // read from your hardware/config

			frame_response(&num_leds, 1, rcv->seq, CMD_RES_LED_NUM);
			break;
		}

		case CMD_REQ_DELAY:
		{
			uint8_t delay_val = 100; // whatever delay it wants

			frame_response(&delay_val, 1, rcv->seq, CMD_RES_DELAY);
			break;
		}

		case CMD_HEART_BEAT_REQ:
		{
			// No payload needed in heartbeat response
			frame_response(NULL, 0, rcv->seq, CMD_HEART_BEAT_RES);
			break;
		}

			/* Responses of our request  */
		case CMD_RES_LED_NUM:
			printf("Got LED_NUM response: %u\r\n", rcv->payload[0]);
			break;
		case CMD_RES_DELAY:
			printf("Got DELAY response: %u\r\n", rcv->payload[0]);
			break;
		case CMD_HEART_BEAT_RES:
			printf("Got HEARTBEAT response\r\n");
			break;

		default:
			printf("Unknown cmd: 0x%02X\r\n", rcv->cmd);
			break;
		}
	}
}

void frame_reset()
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
void frame_init()
{
	if (frm.frm_init != false)
		return;

	frame_reset();

	frm.frm_init = true;
}

// Send Request :
void frame_request(const uint8_t *const data, uint16_t len, uint8_t cmd)
{
	//
	if (frm.frm_init != true)
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
void frame_retransmit()
{
	com.write(frm.rt_buff, frm.rt_len);
	frm.response_time = HAL_GetTick(); // restart the timer for another timeout
}

void frame_response(const uint8_t *const data, uint16_t len,
					uint8_t seq, uint8_t cmd)
{
	if (!frm.frm_init)
		return;

	format_packet(data, (uint8_t)len, seq, cmd);
	packet_send();

	/* No retry for responses */
}

void poll_packets()
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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();

	retarget_init(&huart2);

	printf("Application Started\r\n");
	// init : Test the frame parser and frame Formatter
	com.open();

	// format the packet
	frame_init();

	//  This callback will be used to call  ipc Layer
	parser_init(frame_func);

	// Store the last sequence number

	char test_data[] = "testData\r\n";

	/*
	 *
	 * MCU A  ──── CMD_REQ  seq=1 ────►  MCU B
	   MCU A  ◄─── CMD_RES  seq=1 ─────  MCU B
	 */
	frame_request((uint8_t *)test_data, strlen(test_data), CMD_REQ_LED_NUM);
	/* Infinite loop */
	while (1)
	{

		/* USER CODE END 3 */

		// This should be used in a FreeRTOS Tasks Later ON :
		poll_packets();
	}
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 180;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
		// Blink the LED rapidly — if you see this, clock init failed
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		for (volatile uint32_t i = 0; i < 200000; i++)
			;
	}
	/* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
