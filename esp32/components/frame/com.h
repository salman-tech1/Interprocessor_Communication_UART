/*
 * com.h 
 *  Author: Muhmmad Salman
 */

#ifndef COM_H
#define COM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum {
    COM_OK      =  0,   /**< Operation successful */
    COM_ERROR   = -1,   /**< Generic error (bad args, not initialized) */
    COM_TIMEOUT = -2,   /**< Transmit did not complete in time */
    COM_BUSY    = -3,   /**< Transport busy */
} Com_Status_t;

typedef enum {
    COM_EVENT_RX_DATA, /**< New bytes are sitting in the transport buffer */
    COM_EVENT_ERROR,   /**< Transport hit a hardware error */
} Com_Event_t;

/**
 * @brief  Event notification.
 */
typedef void (*Com_EventCallback_t)(Com_Event_t event);

/**
 * @brief  Bring up the transport (UART peripheral + driver ring buffers).
 */
Com_Status_t Com_Init(void);

/**
 * @brief  Clean up COM driver resources.
 */
Com_Status_t Com_DeInit(void);

/**
 * @brief  Register the callback invoked on RX/error events.
 */
void Com_RegisterEventCallback(Com_EventCallback_t callback);

/**
 * @brief  Transmit a raw byte buffer (blocking, with timeout).
 */
Com_Status_t Com_Send(const uint8_t *data, uint16_t length);

/**
 * @brief  Pull whatever new bytes have arrived since the last call.
 */
Com_Status_t Com_recieve(uint8_t *data, uint32_t *len);
 
/**
 * @brief  Recover the transport after a COM_EVENT_ERROR notification.
 */
Com_Status_t Com_Recover(void);

#ifdef __cplusplus
}
#endif

#endif /* COM_H */