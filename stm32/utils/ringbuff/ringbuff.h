/*
 * ringbuff.h
 *
 *  Created on: Aug 5, 2026
 *      Author: Muhmmad Salman
 */

#ifndef RINGBUFF_H_
#define RINGBUFF_H_

#include <stdbool.h>

#define RING_BUFFER_SIZE 256
typedef volatile uint8_t ring_buff_type;

typedef struct
{
    ring_buff_type head; /* next index to write */
    ring_buff_type tail; /* next index to read  */
    ring_buff_type buff[RING_BUFFER_SIZE];
} ring_buffer_t;

// push data to ring buffer
 bool ring_buff_push(ring_buffer_t *r, uint8_t data);
// pop  data from ring buffer
 bool ring_buff_pop(ring_buffer_t *r, uint8_t *out_byte);


#endif
