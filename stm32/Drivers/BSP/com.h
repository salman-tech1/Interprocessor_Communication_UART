/*
 * com.h
 *
 *  Created on: Aug 5, 2026
 *      Author: Muhmmad Salman
 *      This layer is closed to hardware and only responsible for Recieving and Transmitting
 *      Raw Bytes
 */

#ifndef BSP_COM_H_
#define BSP_COM_H_

typedef enum
{
	COM_OK =0  ,
	COM_FAIL ,
	COM_TIMEOUT ,
	COM_INVALID_PARAMETER ,

}COM_Status_t;


typedef struct {
	COM_Status_t (*open)(void);
	COM_Status_t (*close)(void) ;
	uint16_t (*write)(const uint8_t *data,
                          uint16_t length);
    uint16_t (*read)(uint8_t *data,
                         uint16_t length);
} com_t;


extern const com_t com ;

#endif /* BSP_COM_H_ */
