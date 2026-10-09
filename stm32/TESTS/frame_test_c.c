#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

/* ============================================================
 * Configuration
 * ============================================================ */

#define PACKET_PARSE_TIMEOUT 2000U
#define FRAME_RESPONSE_TIMEOUT 3000U
#define PAYLOAD_SIZE 200U
#define FRAME_HEADER_SIZE 5U
#define FRAME_CRC_SIZE 2U
#define FRAME_SIZE (FRAME_HEADER_SIZE + PAYLOAD_SIZE + FRAME_CRC_SIZE)

#define SYNC0 0x55
#define SYNC1 0xAA
#define MAX_RETRIES 3

#define CMD_REQ_LED_NUM 0x10
#define CMD_RES_LED_NUM 0x11
#define CMD_REQ_DELAY 0x20
#define CMD_RES_DELAY 0x21
#define CMD_HEART_BEAT_REQ 0x30
#define CMD_HEART_BEAT_RES 0x31

/* ============================================================
 * Mock: HAL_GetTick
 * ============================================================ */

static uint32_t mock_tick = 0;

uint32_t HAL_GetTick(void) { return mock_tick; }

void advance_time(uint32_t ms) { mock_tick += ms; }
void reset_time(void) { mock_tick = 0; }

/* ============================================================
 * Mock: com interface
 * ============================================================ */

static uint8_t com_rx_data[1024];
static uint16_t com_rx_len = 0;
static uint16_t com_rx_pos = 0;

static uint8_t com_tx_data[1024];
static uint16_t com_tx_len = 0;
static uint16_t com_tx_write_count = 0;

struct
{
  void (*open)(void);
  uint16_t (*read)(uint8_t *buf, uint16_t max);
  void (*write)(const uint8_t *data, uint16_t len);
} com;

static uint16_t mock_com_read(uint8_t *buf, uint16_t max)
{
  uint16_t count = 0;
  while (com_rx_pos < com_rx_len && count < max)
  {
    buf[count++] = com_rx_data[com_rx_pos++];
  }
  return count;
}

static void mock_com_write(const uint8_t *data, uint16_t len)
{
  if (com_tx_len + len < (uint16_t)sizeof(com_tx_data))
  {
    memcpy(com_tx_data + com_tx_len, data, len);
    com_tx_len += len;
  }
  com_tx_write_count++;
}

static void mock_com_open(void) {}

void com_reset(void)
{
  com_rx_len = 0;
  com_rx_pos = 0;
  com_tx_len = 0;
  com_tx_write_count = 0;
}

void inject_rx(const uint8_t *data, uint16_t len)
{
  memcpy(com_rx_data + com_rx_len, data, len);
  com_rx_len += len;
}

/* ============================================================
 * Types
 * ============================================================ */

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

/* ============================================================
 * frm (IPC / Frame layer state)
 * ============================================================ */

static struct
{
  bool is_requested;
  bool frm_init;
  uint32_t response_time;
  uint8_t retry;
  uint8_t rt_buff[FRAME_SIZE];
  uint16_t rt_len;
  uint8_t tx_seq;
  uint8_t last_rx_seq;
  bool last_rx_seq_valid;
} frm;

/* ============================================================
 * CRC-16
 * ============================================================ */
void frame_response(const uint8_t *const data, uint16_t len,
                    uint8_t seq, uint8_t cmd);
static uint16_t calculate_crc16(const uint8_t *data, size_t len)
{
  uint16_t crc = 0xFFFFU;
  for (size_t i = 0U; i < len; i++)
  {
    crc ^= (uint16_t)data[i];
    for (uint8_t b = 0U; b < 8U; b++)
    {
      crc = (crc & 0x0001U)
                ? (uint16_t)((crc >> 1U) ^ 0xA001U)
                : (uint16_t)(crc >> 1U);
    }
  }
  return crc;
}

/* ============================================================
 * Packet Formatter (TX)
 * ============================================================ */

static uint8_t txbuff[FRAME_SIZE];
static bool packet_formatted = false;
static uint16_t tx_idx = 0;

void format_packet(const uint8_t *const data, uint8_t len,
                   uint8_t seq, uint8_t cmd)
{
  if ((data == NULL && len > 0) || len > PAYLOAD_SIZE || packet_formatted)
  {
    printf("  [ERR] format_packet guard\r\n");
    return;
  }

  tx_idx = 0;
  txbuff[tx_idx++] = SYNC0;
  txbuff[tx_idx++] = SYNC1;
  txbuff[tx_idx++] = len;
  txbuff[tx_idx++] = cmd;
  txbuff[tx_idx++] = seq;

  if (len > 0)
  {
    memcpy(&txbuff[tx_idx], data, len);
    tx_idx += len;
  }

  uint16_t crc_ = calculate_crc16(txbuff, tx_idx);
  txbuff[tx_idx++] = (uint8_t)(crc_ & 0xFFU);
  txbuff[tx_idx++] = (uint8_t)((crc_ >> 8U) & 0xFFU);

  packet_formatted = true;
}

void packet_send(void)
{
  if (!packet_formatted || tx_idx < (FRAME_HEADER_SIZE + FRAME_CRC_SIZE))
  {
    printf("  [ERR] packet_send guard\r\n");
    return;
  }

  com.write(txbuff, tx_idx);

  tx_idx = 0;
  packet_formatted = false;
  memset(txbuff, 0, FRAME_SIZE);
}

/* ============================================================
 * Parser (RX)
 * ============================================================ */

static uint8_t payloadBuff[PAYLOAD_SIZE];

static struct
{
  PACKET_STATE_T state_;
  uint32_t prev_timer;
  bool check_timeout;
  uint16_t len;
  uint8_t cmd;
  uint16_t payload_idx;
  uint8_t seq;
  uint16_t crc;
  uint8_t crc_index;
  uint16_t frame_idx;
  uint8_t rxbuff[FRAME_SIZE];
} ps;

static bool prsr_init = false;
static parser_cb_t parser_callback = NULL;

void parser_reset(void)
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
  if (prsr_init)
    return;
  parser_reset();
  if (parser_callback == NULL)
  {
    parser_callback = cb;
  }
  prsr_init = true;
}

void parse_packets(uint8_t packet)
{
  if (!prsr_init)
    return;

  switch (ps.state_)
  {

  case SYNC0_STATE:
    if (SYNC0 == packet)
    {
      ps.rxbuff[ps.frame_idx++] = packet;
      ps.prev_timer = HAL_GetTick();
      ps.check_timeout = true;
      ps.state_ = SYNC1_STATE;
    }
    break;

  case SYNC1_STATE:
    if (SYNC1 == packet)
    {
      ps.rxbuff[ps.frame_idx++] = packet;
      ps.prev_timer = HAL_GetTick();
      ps.state_ = LENGTH_STATE;
    }
    else if (packet == SYNC0)
    {
      ps.frame_idx = 1;
      ps.rxbuff[0] = SYNC0;
      ps.prev_timer = HAL_GetTick();
    }
    else
    {
      parser_reset();
    }
    break;

  case LENGTH_STATE:
    ps.prev_timer = HAL_GetTick();
    if (packet > PAYLOAD_SIZE)
    {
      parser_reset();
    }
    else
    {
      ps.rxbuff[ps.frame_idx++] = packet;
      ps.len = packet;
      ps.state_ = CMD_STATE;
    }
    break;

  case CMD_STATE:
  {
    ps.prev_timer = HAL_GetTick();
    switch (packet)
    {
    case CMD_REQ_LED_NUM:
    case CMD_RES_LED_NUM:
    case CMD_REQ_DELAY:
    case CMD_RES_DELAY:
    case CMD_HEART_BEAT_REQ:
    case CMD_HEART_BEAT_RES:
      ps.rxbuff[ps.frame_idx++] = packet;
      ps.cmd = packet;
      ps.state_ = SEQ_STATE;
      break;
    default:
      parser_reset();
      break;
    }
    break;
  }

  case SEQ_STATE:
    ps.prev_timer = HAL_GetTick();
    ps.seq = packet;
    ps.rxbuff[ps.frame_idx++] = packet;
    if (ps.len == 0)
    {
      ps.state_ = CRC_STATE;
    }
    else
    {
      ps.payload_idx = 0;
      ps.state_ = PAYLOAD_STATE;
    }
    break;

  case PAYLOAD_STATE:
    ps.prev_timer = HAL_GetTick();
    payloadBuff[ps.payload_idx++] = packet;
    ps.rxbuff[ps.frame_idx++] = packet;
    if (ps.payload_idx >= ps.len)
    {
      ps.state_ = CRC_STATE;
    }
    break;

  case CRC_STATE:
    ps.prev_timer = HAL_GetTick();
    if (ps.crc_index == 0)
    {
      ps.crc = packet;
      ps.crc_index = 1;
    }
    else
    {
      ps.crc |= ((uint16_t)packet << 8);
      uint16_t calculated_crc = calculate_crc16(ps.rxbuff, ps.frame_idx);
      if (calculated_crc == ps.crc)
      {
        static packet_rcvd_t pckt__;
        pckt__.cmd = ps.cmd;
        pckt__.seq = ps.seq;
        pckt__.len = ps.len;
        memcpy(pckt__.payload, payloadBuff, ps.len);
        if (parser_callback)
        {
          parser_callback(PARSER_EVENT_RECVD, &pckt__);
        }
      }
      else
      {
        if (parser_callback)
        {
          parser_callback(PARSER_EVENT_ERR, NULL);
        }
      }
      parser_reset();
    }
    break;
  }
}

/* ============================================================
 * IPC / Frame Layer
 * ============================================================ */

static bool is_response_cmd(uint8_t cmd)
{
  return (cmd == CMD_RES_LED_NUM ||
          cmd == CMD_RES_DELAY ||
          cmd == CMD_HEART_BEAT_RES);
}

void frame_retransmit(void);

void frame_func(PARSER_EVENT_T evnt, packet_rcvd_t *rcv)
{
  if (evnt == PARSER_EVENT_ERR)
  {
    printf("  [IPC] Frame Error\r\n");
    return;
  }
  else if (evnt == PARSER_EVENT_TIMEOUT)
  {
    printf("  [IPC] Parser Timeout\r\n");
  }
  else if (evnt == PARSER_RESPONSE_EVENT_TIMEOUT && frm.is_requested)
  {
    frm.retry++;
    if (frm.retry <= MAX_RETRIES)
    {
      frame_retransmit();
      printf("  [IPC] Retrying... attempt %u/%u\r\n",
             frm.retry, MAX_RETRIES);
    }
    else
    {
      printf("  [IPC] *** MAX RETRIES REACHED ***\r\n");
      frm.retry = 0;
      frm.is_requested = false;
    }
  }

  if (evnt == PARSER_EVENT_RECVD && rcv != NULL)
  {

    /* ===== DUPLICATE DETECTION ===== */
    if (frm.last_rx_seq_valid && rcv->seq == frm.last_rx_seq)
    {
      if (is_response_cmd(rcv->cmd))
      {
        printf("  [IPC] Duplicate response seq=%u — IGNORED\r\n",
               rcv->seq);
        return;
      }
      printf("  [IPC] Duplicate request seq=%u — re-responding\r\n",
             rcv->seq);
    }

    /* Update last received sequence */
    frm.last_rx_seq = rcv->seq;
    frm.last_rx_seq_valid = true;

    /* If this is a response to OUR request, stop retrying */
    if (is_response_cmd(rcv->cmd) && frm.is_requested)
    {
      frm.retry = 0;
      frm.is_requested = false;
    }

    printf("  [IPC] Received cmd=0x%02X seq=%u len=%u\r\n",
           rcv->cmd, rcv->seq, rcv->len);

    switch (rcv->cmd)
    {
    case CMD_REQ_LED_NUM:
    {
      uint8_t num_leds = 8;
      frame_response(&num_leds, 1, rcv->seq, CMD_RES_LED_NUM);
      break;
    }
    case CMD_REQ_DELAY:
    {
      uint8_t delay_val = 100;
      frame_response(&delay_val, 1, rcv->seq, CMD_RES_DELAY);
      break;
    }
    case CMD_HEART_BEAT_REQ:
    {
      frame_response(NULL, 0, rcv->seq, CMD_HEART_BEAT_RES);
      break;
    }
    case CMD_RES_LED_NUM:
      printf("  [APP] Got LED_NUM response: %u\r\n", rcv->payload[0]);
      break;
    case CMD_RES_DELAY:
      printf("  [APP] Got DELAY response: %u\r\n", rcv->payload[0]);
      break;
    case CMD_HEART_BEAT_RES:
      printf("  [APP] Got HEARTBEAT response\r\n");
      break;
    default:
      printf("  [IPC] Unknown cmd: 0x%02X\r\n", rcv->cmd);
      break;
    }
  }
}

void frame_reset(void)
{
  frm.is_requested = false;
  frm.response_time = 0;
  frm.retry = 0;
  frm.rt_len = 0;
  frm.tx_seq = 1;
  frm.last_rx_seq = 0;
  frm.last_rx_seq_valid = false;
}

void frame_init(void)
{
  if (frm.frm_init)
    return;
  frame_reset();
  frm.frm_init = true;
}

void frame_request(const uint8_t *const data, uint16_t len, uint16_t cmd)
{
  if (!frm.frm_init)
    return;

  uint8_t seq = frm.tx_seq++;

  format_packet(data, (uint8_t)len, seq, (uint8_t)cmd);

  frm.rt_len = tx_idx;
  memcpy(frm.rt_buff, txbuff, frm.rt_len);

  packet_send();

  frm.is_requested = true;
  frm.response_time = HAL_GetTick();
  frm.retry = 0;
}

void frame_retransmit(void)
{
  com.write(frm.rt_buff, frm.rt_len);
  frm.response_time = HAL_GetTick();
}

void frame_response(const uint8_t *const data, uint16_t len,
                    uint8_t seq, uint8_t cmd)
{
  if (!frm.frm_init)
    return;
  format_packet(data, (uint8_t)len, seq, cmd);
  packet_send();
}

/* ============================================================
 * poll_packets
 * ============================================================ */

void poll_packets(void)
{
  if (!prsr_init)
    return;

  uint8_t rd_buff[20];
  uint16_t rd_len;

  if ((rd_len = com.read(rd_buff, sizeof(rd_buff))) > 0)
  {
    for (unsigned int i = 0; i < rd_len; ++i)
    {
      parse_packets(rd_buff[i]);
    }
  }

  if (ps.check_timeout &&
      (HAL_GetTick() - ps.prev_timer >= PACKET_PARSE_TIMEOUT))
  {
    if (parser_callback)
    {
      parser_callback(PARSER_EVENT_TIMEOUT, NULL);
    }
    parser_reset();
  }

  if (frm.is_requested &&
      (HAL_GetTick() - frm.response_time >= FRAME_RESPONSE_TIMEOUT))
  {
    if (parser_callback)
    {
      parser_callback(PARSER_RESPONSE_EVENT_TIMEOUT, NULL);
    }
  }
}

/* ============================================================
 * Test helpers
 * ============================================================ */

static int test_pass = 0;
static int test_fail = 0;

/* Track how many times frame_func processed a RECVD (for dup detection) */
static int recvd_count = 0;
static int dup_response_ignored_count = 0;

/* We wrap frame_func to count calls for test verification */
static parser_cb_t real_frame_func = NULL;

static void test_frame_func_wrapper(PARSER_EVENT_T evnt, packet_rcvd_t *rcv)
{
  if (evnt == PARSER_EVENT_RECVD && rcv != NULL)
  {
    recvd_count++;
  }
  /* We detect "ignored" by checking if recvd_count does NOT
     increment after feeding a second identical frame.
     We'll verify that outside the callback. */

  real_frame_func(evnt, rcv);
}

void CHECK(const char *name, bool condition)
{
  if (condition)
  {
    printf("  PASS: %s\r\n", name);
    test_pass++;
  }
  else
  {
    printf("  FAIL: %s\r\n", name);
    test_fail++;
  }
}

uint16_t build_frame(uint8_t *buffer, const uint8_t *data,
                     uint8_t len, uint8_t seq, uint8_t cmd)
{
  uint16_t idx = 0;
  buffer[idx++] = SYNC0;
  buffer[idx++] = SYNC1;
  buffer[idx++] = len;
  buffer[idx++] = cmd;
  buffer[idx++] = seq;
  if (len > 0 && data != NULL)
  {
    memcpy(&buffer[idx], data, len);
    idx += len;
  }
  uint16_t crc = calculate_crc16(buffer, idx);
  buffer[idx++] = (uint8_t)(crc & 0xFFU);
  buffer[idx++] = (uint8_t)((crc >> 8U) & 0xFFU);
  return idx;
}

void print_hex(const char *label, const uint8_t *data, uint16_t len)
{
  printf("  %s (%u bytes): ", label, len);
  for (uint16_t i = 0; i < len && i < 32; i++)
  {
    printf("%02X ", data[i]);
  }
  if (len > 32)
    printf("...");
  printf("\r\n");
}

void test_reset(void)
{
  prsr_init = false;
  parser_callback = NULL;
  parser_reset();

  frm.is_requested = false;
  frm.frm_init = false;
  frm.response_time = 0;
  frm.retry = 0;
  frm.rt_len = 0;
  frm.tx_seq = 1;
  frm.last_rx_seq = 0;
  frm.last_rx_seq_valid = false;
  memset(frm.rt_buff, 0, FRAME_SIZE);

  packet_formatted = false;
  tx_idx = 0;
  memset(txbuff, 0, FRAME_SIZE);

  com_reset();
  reset_time();

  recvd_count = 0;
  dup_response_ignored_count = 0;

  com.open = mock_com_open;
  com.read = mock_com_read;
  com.write = mock_com_write;
  com.open();

  frame_init();

  real_frame_func = frame_func;
  parser_init(test_frame_func_wrapper);
}

/* ============================================================
 * PREVIOUS TESTS (1-9) — unchanged logic, updated for
 * frame_request signature
 * ============================================================ */

void test_frame_request_saves_retransmit_buffer(void)
{
  printf("\n========================================\r\n");
  printf("TEST 1: frame_request saves retransmit buffer\r\n");
  printf("========================================\r\n");

  test_reset();

  uint8_t payload[] = "Hello";
  frame_request(payload, strlen((char *)payload), CMD_REQ_LED_NUM);

  CHECK("is_requested == true", frm.is_requested == true);
  CHECK("retry == 0", frm.retry == 0);
  CHECK("rt_len > 0", frm.rt_len > 0);
  CHECK("rt_len == com_tx_len", frm.rt_len == com_tx_len);
  CHECK("rt_buff == com_tx_data",
        memcmp(frm.rt_buff, com_tx_data, frm.rt_len) == 0);

  print_hex("TX (com.write)", com_tx_data, com_tx_len);
  print_hex("RT (saved)    ", frm.rt_buff, frm.rt_len);
}

void test_response_immediate_no_retry(void)
{
  printf("\n========================================\r\n");
  printf("TEST 2: Immediate response — no retry\r\n");
  printf("========================================\r\n");

  test_reset();

  uint8_t payload[] = "ReqData";
  frame_request(payload, strlen((char *)payload), CMD_REQ_LED_NUM);
  printf("  Sent request: is_requested=%u retry=%u\r\n",
         frm.is_requested, frm.retry);

  /* Build a response with seq matching what we sent (seq=1) */
  uint8_t resp_data = 42;
  uint8_t resp_frame[FRAME_SIZE];
  uint16_t resp_len = build_frame(resp_frame, &resp_data, 1,
                                  1 /*seq*/, CMD_RES_LED_NUM);
  inject_rx(resp_frame, resp_len);
  poll_packets();

  CHECK("is_requested == false", frm.is_requested == false);
  CHECK("retry == 0", frm.retry == 0);
  CHECK("only 1 com.write (original request)",
        com_tx_write_count == 1);
}

void test_no_response_max_retries(void)
{
  printf("\n========================================\r\n");
  printf("TEST 3: No response → max retries reached\r\n");
  printf("========================================\r\n");

  test_reset();

  uint8_t payload[] = "NoReply";
  frame_request(payload, strlen((char *)payload), CMD_REQ_LED_NUM);
  printf("  t=%u Sent request\r\n", HAL_GetTick());

  advance_time(FRAME_RESPONSE_TIMEOUT + 1);
  poll_packets();
  printf("  After timeout 1: retry=%u\r\n", frm.retry);

  advance_time(FRAME_RESPONSE_TIMEOUT + 1);
  poll_packets();
  printf("  After timeout 2: retry=%u\r\n", frm.retry);

  advance_time(FRAME_RESPONSE_TIMEOUT + 1);
  poll_packets();
  printf("  After timeout 3: retry=%u\r\n", frm.retry);

  advance_time(FRAME_RESPONSE_TIMEOUT + 1);
  poll_packets();
  printf("  After timeout 4: retry=%u is_requested=%u\r\n",
         frm.retry, frm.is_requested);

  CHECK("is_requested == false", frm.is_requested == false);
  CHECK("retry == 0", frm.retry == 0);
  CHECK("4 com.write calls (1 original + 3 retries)",
        com_tx_write_count == 4);
}

void test_response_after_retries(void)
{
  printf("\n========================================\r\n");
  printf("TEST 4: Response after 2 retries\r\n");
  printf("========================================\r\n");

  test_reset();

  uint8_t payload[] = "LateReply";
  frame_request(payload, strlen((char *)payload), CMD_REQ_LED_NUM);
  printf("  t=%u Sent request (seq=%u)\r\n", HAL_GetTick(), frm.tx_seq - 1);

  advance_time(FRAME_RESPONSE_TIMEOUT + 1);
  poll_packets();

  advance_time(FRAME_RESPONSE_TIMEOUT + 1);
  poll_packets();

  /* Inject response with seq=1 (matches our request) */
  uint8_t resp_data = 99;
  uint8_t resp_frame[FRAME_SIZE];
  uint16_t resp_len = build_frame(resp_frame, &resp_data, 1,
                                  1 /*seq*/, CMD_RES_LED_NUM);
  inject_rx(resp_frame, resp_len);
  poll_packets();

  CHECK("is_requested == false", frm.is_requested == false);
  CHECK("retry == 0", frm.retry == 0);
  CHECK("3 com.write calls", com_tx_write_count == 3);
}

void test_retransmit_identical_bytes(void)
{
  printf("\n========================================\r\n");
  printf("TEST 5: Retransmit sends identical frame\r\n");
  printf("========================================\r\n");

  test_reset();

  uint8_t payload[] = "Verify";
  frame_request(payload, strlen((char *)payload), CMD_REQ_LED_NUM);

  uint8_t original[FRAME_SIZE];
  uint16_t original_len = com_tx_len;
  memcpy(original, com_tx_data, original_len);

  advance_time(FRAME_RESPONSE_TIMEOUT + 1);
  poll_packets();

  uint16_t rt_start = original_len;
  uint16_t rt_len = com_tx_len - original_len;

  CHECK("retransmit length == original length",
        rt_len == original_len);
  if (rt_len == original_len)
  {
    CHECK("retransmit bytes == original bytes",
          memcmp(original, com_tx_data + rt_start, original_len) == 0);
  }

  print_hex("Original   ", original, original_len);
  print_hex("Retransmit ", com_tx_data + rt_start, rt_len);
}

void test_parser_midframe_timeout(void)
{
  printf("\n========================================\r\n");
  printf("TEST 6: Parser mid-frame timeout\r\n");
  printf("========================================\r\n");

  test_reset();

  uint8_t partial[] = {SYNC0, SYNC1};
  inject_rx(partial, 2);
  poll_packets();

  CHECK("parser in LENGTH_STATE", ps.state_ == LENGTH_STATE);
  CHECK("check_timeout == true", ps.check_timeout == true);

  advance_time(PACKET_PARSE_TIMEOUT + 1);
  poll_packets();

  CHECK("parser reset to SYNC0_STATE", ps.state_ == SYNC0_STATE);
  CHECK("check_timeout == false", ps.check_timeout == false);
}

void test_responder_sends_response(void)
{
  printf("\n========================================\r\n");
  printf("TEST 7: Responder handles request and sends response\r\n");
  printf("========================================\r\n");

  test_reset();

  uint8_t req_payload[] = "GetLEDs";
  uint8_t req_frame[FRAME_SIZE];
  uint16_t req_len = build_frame(req_frame, req_payload,
                                 strlen((char *)req_payload),
                                 10, CMD_REQ_LED_NUM);
  inject_rx(req_frame, req_len);
  poll_packets();

  CHECK("com.write was called", com_tx_write_count >= 1);
  if (com_tx_len >= 5)
  {
    CHECK("response starts with SYNC0+SYNC1",
          com_tx_data[0] == SYNC0 && com_tx_data[1] == SYNC1);
    CHECK("response cmd == CMD_RES_LED_NUM",
          com_tx_data[3] == CMD_RES_LED_NUM);
    CHECK("response seq == 10 (echoes requester)",
          com_tx_data[4] == 10);
  }

  print_hex("Response TX", com_tx_data, com_tx_len);
}

void test_heartbeat_request(void)
{
  printf("\n========================================\r\n");
  printf("TEST 8: Heartbeat request with empty payload\r\n");
  printf("========================================\r\n");

  test_reset();

  frame_request(NULL, 0, CMD_HEART_BEAT_REQ);

  CHECK("is_requested == true", frm.is_requested == true);
  CHECK("rt_len > 0", frm.rt_len > 0);
  CHECK("rt_len == 7", frm.rt_len == 7);

  if (frm.rt_len >= 5)
  {
    CHECK("cmd in saved frame == CMD_HEART_BEAT_REQ",
          frm.rt_buff[3] == CMD_HEART_BEAT_REQ);
  }

  print_hex("Heartbeat frame", frm.rt_buff, frm.rt_len);
}

void test_retry_does_not_leak_across_requests(void)
{
  printf("\n========================================\r\n");
  printf("TEST 9: Retry count does not leak across requests\r\n");
  printf("========================================\r\n");

  test_reset();

  uint8_t payload[] = "LeakTest";

  /* First request: 1 timeout, then response */
  frame_request(payload, strlen((char *)payload), CMD_REQ_LED_NUM);
  uint8_t seq1 = frm.tx_seq - 1;

  advance_time(FRAME_RESPONSE_TIMEOUT + 1);
  poll_packets();

  /* Inject response for seq1 */
  uint8_t resp_data = 1;
  uint8_t resp_frame[FRAME_SIZE];
  uint16_t resp_len = build_frame(resp_frame, &resp_data, 1,
                                  seq1, CMD_RES_LED_NUM);
  inject_rx(resp_frame, resp_len);
  poll_packets();

  CHECK("retry reset to 0", frm.retry == 0);

  /* Second request */
  frame_request(payload, strlen((char *)payload), CMD_REQ_LED_NUM);
  CHECK("new request starts with retry == 0", frm.retry == 0);

  for (int i = 1; i <= 4; i++)
  {
    advance_time(FRAME_RESPONSE_TIMEOUT + 1);
    poll_packets();
  }

  CHECK("is_requested == false (max retries reached)",
        frm.is_requested == false);
}

/* ============================================================
 * NEW TESTS: Sequence and Duplicate Detection
 * ============================================================ */

/* TEST 10: tx_seq auto-increments on each frame_request */
void test_tx_seq_auto_increments(void)
{
  printf("\n========================================\r\n");
  printf("TEST 10: tx_seq auto-increments on each request\r\n");
  printf("========================================\r\n");

  test_reset();

  CHECK("tx_seq starts at 1", frm.tx_seq == 1);

  uint8_t payload[] = "SeqTest";

  frame_request(payload, strlen((char *)payload), CMD_REQ_LED_NUM);
  uint8_t seq1 = frm.tx_seq - 1; /* the seq that was just used */
  printf("  Request 1: used seq=%u, tx_seq now=%u\r\n",
         seq1, frm.tx_seq);
  CHECK("first request used seq=1", seq1 == 1);
  CHECK("tx_seq now 2", frm.tx_seq == 2);

  /* Clear is_requested so we can send another */
  frm.is_requested = false;

  frame_request(payload, strlen((char *)payload), CMD_REQ_LED_NUM);
  uint8_t seq2 = frm.tx_seq - 1;
  printf("  Request 2: used seq=%u, tx_seq now=%u\r\n",
         seq2, frm.tx_seq);
  CHECK("second request used seq=2", seq2 == 2);
  CHECK("tx_seq now 3", frm.tx_seq == 3);

  /* Verify the two frames have different seq bytes (byte 4) */
  /* First frame is in com_tx_data from offset 0 to first send length */
  /* Second frame starts after that */
  /* Actually easier: just check the rt_buff which holds the LAST request */
  CHECK("rt_buff seq byte == 2", frm.rt_buff[4] == 2);
}

/* TEST 11: Duplicate response is ignored */
void test_duplicate_response_ignored(void)
{
  printf("\n========================================\r\n");
  printf("TEST 11: Duplicate response is IGNORED\r\n");
  printf("========================================\r\n");

  test_reset();

  uint8_t payload[] = "DupTest";
  frame_request(payload, strlen((char *)payload), CMD_REQ_LED_NUM);
  uint8_t used_seq = frm.tx_seq - 1;
  printf("  Sent request with seq=%u\r\n", used_seq);

  /* Inject first response — should be processed */
  uint8_t resp_data = 77;
  uint8_t resp_frame[FRAME_SIZE];
  uint16_t resp_len = build_frame(resp_frame, &resp_data, 1,
                                  used_seq, CMD_RES_LED_NUM);

  int count_before = recvd_count;
  inject_rx(resp_frame, resp_len);
  poll_packets();
  int count_after_first = recvd_count;

  printf("  After 1st response: recvd_count=%d (was %d)\r\n",
         count_after_first, count_before);
  CHECK("1st response was processed",
        count_after_first == count_before + 1);
  CHECK("is_requested == false after 1st response",
        frm.is_requested == false);

  /* Capture TX write count after first response */
  uint16_t tx_writes_after_first = com_tx_write_count;

  /* Inject the SAME response again (same seq) — should be IGNORED */
  inject_rx(resp_frame, resp_len);
  poll_packets();
  int count_after_second = recvd_count;

  printf("  After 2nd (duplicate) response: recvd_count=%d\r\n",
         count_after_second);
  CHECK("2nd response was NOT processed (duplicate ignored)",
        count_after_second == count_after_first);

  /* No additional com.write should have happened for the ignored dup */
  CHECK("no extra com.write for duplicate response",
        com_tx_write_count == tx_writes_after_first);

  /* last_rx_seq should still be the same */
  CHECK("last_rx_seq == used_seq", frm.last_rx_seq == used_seq);
}

/* TEST 12: Duplicate request triggers re-response */
void test_duplicate_request_re_responds(void)
{
  printf("\n========================================\r\n");
  printf("TEST 12: Duplicate request triggers RE-RESPONSE\r\n");
  printf("========================================\r\n");

  test_reset();

  /* Inject a request from the other MCU */
  uint8_t req_payload[] = "GetLED";
  uint8_t req_frame[FRAME_SIZE];
  uint16_t req_len = build_frame(req_frame, req_payload,
                                 strlen((char *)req_payload),
                                 5, /* seq from requester */
                                 CMD_REQ_LED_NUM);

  /* First request — should trigger a response */
  int count_before = recvd_count;
  inject_rx(req_frame, req_len);
  poll_packets();
  int count_after_first = recvd_count;
  uint16_t tx_writes_after_first = com_tx_write_count;

  printf("  After 1st request: recvd_count=%d, com_writes=%u\r\n",
         count_after_first, tx_writes_after_first);

  CHECK("1st request was processed",
        count_after_first == count_before + 1);
  CHECK("response was sent", tx_writes_after_first >= 1);

  /* Now send the SAME request again (same seq=5) —
   * this is a duplicate REQUEST, should be re-responded */
  inject_rx(req_frame, req_len);
  poll_packets();
  int count_after_second = recvd_count;
  uint16_t tx_writes_after_second = com_tx_write_count;

  printf("  After 2nd (duplicate) request: recvd_count=%d, com_writes=%u\r\n",
         count_after_second, tx_writes_after_second);

  /* Duplicate request IS processed (fall through) */
  CHECK("duplicate request was processed (re-responded)",
        count_after_second == count_after_first + 1);

  /* A second response was sent */
  CHECK("second response was sent",
        tx_writes_after_second == tx_writes_after_first + 1);
}

/* TEST 13: Different seq is NOT a duplicate */
void test_different_seq_not_duplicate(void)
{
  printf("\n========================================\r\n");
  printf("TEST 13: Different seq is NOT a duplicate\r\n");
  printf("========================================\r\n");

  test_reset();

  /* Receive response with seq=1 */
  uint8_t resp_data = 10;
  uint8_t resp_frame1[FRAME_SIZE];
  uint16_t resp_len1 = build_frame(resp_frame1, &resp_data, 1,
                                   1, CMD_RES_LED_NUM);
  inject_rx(resp_frame1, resp_len1);
  poll_packets();

  CHECK("last_rx_seq == 1", frm.last_rx_seq == 1);

  int count_after_first = recvd_count;

  /* Receive response with seq=2 — NOT a duplicate */
  uint8_t resp_frame2[FRAME_SIZE];
  uint16_t resp_len2 = build_frame(resp_frame2, &resp_data, 1,
                                   2, CMD_RES_LED_NUM);
  inject_rx(resp_frame2, resp_len2);
  poll_packets();

  CHECK("seq=2 was processed (not duplicate)",
        recvd_count == count_after_first + 1);
  CHECK("last_rx_seq updated to 2", frm.last_rx_seq == 2);
}

/* TEST 14: frame_response does not set is_requested */
void test_frame_response_no_retry(void)
{
  printf("\n========================================\r\n");
  printf("TEST 14: frame_response does NOT trigger retry\r\n");
  printf("========================================\r\n");

  test_reset();

  /* Simulate being a responder: call frame_response directly */
  uint8_t resp_data = 55;
  frame_response(&resp_data, 1, 7, CMD_RES_LED_NUM);

  CHECK("is_requested == false (response never sets it)",
        frm.is_requested == false);
  CHECK("retry == 0", frm.retry == 0);

  /* Advance time way past timeout — no retry should happen */
  advance_time(FRAME_RESPONSE_TIMEOUT * 10);
  poll_packets();

  CHECK("still no retry (only 1 com.write)",
        com_tx_write_count == 1);
}

/* ============================================================
 * MAIN
 * ============================================================ */

int main(void)
{
  printf("========================================\r\n");
  printf("  IPC RETRY + DUPLICATE DETECTION TEST\r\n");
  printf("========================================\r\n");

  /* Previous tests (retry mechanism) */
  test_frame_request_saves_retransmit_buffer();
  test_response_immediate_no_retry();
  test_no_response_max_retries();
  test_response_after_retries();
  test_retransmit_identical_bytes();
  test_parser_midframe_timeout();
  test_responder_sends_response();
  test_heartbeat_request();
  test_retry_does_not_leak_across_requests();

  /* New tests (sequence + duplicate detection) */
  test_tx_seq_auto_increments();
  test_duplicate_response_ignored();
  test_duplicate_request_re_responds();
  test_different_seq_not_duplicate();
  test_frame_response_no_retry();

  printf("\n========================================\r\n");
  printf("  RESULTS: %d passed, %d failed\r\n",
         test_pass, test_fail);
  printf("========================================\r\n");

  return test_fail > 0 ? 1 : 0;
}