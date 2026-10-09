## Inter-Processor Communication over UART
#### A reliable UART-based inter processor communication protocol for communication between STM32 and ESP32 microcontrollers, featuring automatic retry, duplicate detection, and CRC-16 error checking.

Protocol Frame Format

```| SYNC0 | SYNC1 | LENGTH | CMD | SEQ | PAYLOAD | CRC_LSB | CRC_MSB |```

```| 0x55 | 0xAA | 1 byte | 1b | 1b | N bytes | 1 byte | 1 byte |```

- **SYNC**: Fixed synchronization bytes (0x55, 0xAA)
- **LENGTH**: Payload length (0–200 bytes)
- **CMD**: Command identifier
- **SEQ**: Sequence number (request sets it, response echoes it)
- **PAYLOAD**: Variable-length data
- **CRC**: CRC-16 (Modbus) over all preceding bytes, LSB first

## Supported Commands

| Command | Code | Direction |
|---------|------|-----------|
| CMD_REQ_LED_NUM | 0x10 | Requester → Responder |
| CMD_RES_LED_NUM | 0x11 | Responder → Requester |
| CMD_REQ_DELAY | 0x20 | Requester → Responder |
| CMD_RES_DELAY | 0x21 | Responder → Requester |
| CMD_HEART_BEAT_REQ | 0x30 | Requester → Responder |
| CMD_HEART_BEAT_RES | 0x31 | Responder → Requester |

## Features

### Parser Layer
- Byte-by-byte state machine
- CRC-16 (Modbus) verification
- Mid-frame silence timeout

### IPC Layer
- **Automatic retry** — retransmits up to MAX_RETRIES on response timeout
- **Duplicate detection** — separate tracking for requests and responses
- **Stale response filtering** — ignores responses with mismatched sequence numbers
- **Sequence management** — auto-incrementing sequence numbers for requests
- **Request/Response separation** — `frame_request()` (with retry) vs `frame_response()` (no retry)

### Frame Formatter
- CRC-16 calculation and serialization
- Frame building with sync bytes and length field

## Architecture
<img width="6008" height="2020" alt="image" src="https://github.com/user-attachments/assets/72097b31-7702-486f-8fe0-5aec86dd6460" />

## Real Time Demonstration 
<img width="4732" height="2404" alt="image" src="https://github.com/user-attachments/assets/aa499569-b6c4-458c-a4cb-a499ec654fb4" />


<img width="2104" height="3856" alt="image" src="https://github.com/user-attachments/assets/d25a01a6-5f10-4e94-bc4f-496201c9356c" />



## Project Structure

| Directory                   | Description                       |
|-----------------------------|-----------------------------------|
|stm32/                       | Source files for the stm32 side   |
|esp32/                       |source files for the esp32 side    |

## Wiring
STM32 TX ───► ESP32 RX

STM32 RX ◄─── ESP32 TX

STM32 GND ──── ESP32 GND

## Configuration

| Parameter | Default | Description |
|-----------|---------|-------------|
| `PACKET_PARSE_TIMEOUT` | 2000 ms | Mid-frame silence timeout |
| `FRAME_RESPONSE_TIMEOUT` | 3000 ms | No-response timeout (triggers retry) |
| `MAX_RETRIES` | 3 | Maximum retry attempts |
| `PAYLOAD_SIZE` | 200 bytes | Maximum payload length |
| Baud rate | 115200 | UART baud rate |

## Usage

### Initialization
```c
com.open();
frame_init();
parser_init(frame_func);
Send a request (with retry)
c

uint8_t data[] = "request";
frame_request(data, strlen(data), CMD_REQ_LED_NUM);
Send a response (no retry, echoes requester's seq)
c

uint8_t result = 42;
frame_response(&result, 1, rcv->seq, CMD_RES_LED_NUM);
Main loop
c

while (1) {
    poll_packets();
}
Dependencies
STM32: STM32CubeIDE, STM32 HAL
ESP32: ESP-IDF (v5.x)
License
This project is provided as-is for educational and embedded systems use.
