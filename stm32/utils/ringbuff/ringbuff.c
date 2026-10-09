/*
 * ringbuff.c
 *
 *  Created on: Aug 5, 2026
 *      Author: Muhmmad Salman
 *
   Ring buffer works as two pointers one is head which is for writing into the
   buffer and other is tail which is for reading . if both are equal than it means
   empty . so on each write it increments pointers

 */

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include "ringbuff.h"



 bool ring_buff_pop(ring_buffer_t *r, uint8_t *out_byte)
{
    if (r->head == r->tail) return false; /* empty */

    if (out_byte != NULL)
    {
        *out_byte = r->buff[r->tail];
        // increment tail
        r->tail = (r->tail + 1) & (RING_BUFFER_SIZE - 1);
        return true;
    }

    return false;
}


 bool ring_buff_push(ring_buffer_t *r, uint8_t data)
{
	// next head
    uint8_t next_head = (r->head + 1) & (RING_BUFFER_SIZE - 1);

    //
    if (next_head == r->tail) return false; /* full */

    r->buff[r->head] = data;
    r->head = next_head; // increment head
    return true;
}
