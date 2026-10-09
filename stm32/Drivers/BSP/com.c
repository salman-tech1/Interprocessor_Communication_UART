/*
 * com.c
 *
 *  Created on: Aug 4, 2026
 *      Author: Muhmmad Salman
 */


#include <stdint.h>

#include "stm32f4xx.h"  /* ST Device header */
#include "com.h"
#include "ringbuff.h"


// MAX Timeout to wait
#define MAX_TIMEOUT 2000


static void GPIO_Init(void);
static void USART3_Init(void);
static void NVIC_Init(void);

static void USART3_DeInit(void) ;
static void NVIC_DeInit(void) ;
static void GPIO_DeInit(void) ;


// ring buffer
ring_buffer_t ringbuff = {.buff[0] = 0 , .head = 0 , .tail = 0 } ;

volatile static uint8_t rx_data =0 ;

/*
 * PB10 TX
 * PC5  RX
 *
 */

// initialize GPIO first
static void GPIO_Init(void)
{

   /*Enable clocks to GPIOB and GPIOC */
   RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

  /*PB10 (TX): MODER bits [21:20] -> Alternate Function mode (10) */
   GPIOB->MODER &= ~(3U << (10 * 2));
   GPIOB->MODER |=  (2U << (10 * 2));

   // Set speed to very high
   GPIOB->OSPEEDR |= (3U << (10 * 2));           /* very high speed, optional */
   // Alternate Function for pinB
   GPIOB->AFR[1] &= ~(0xFU << ((10 - 8) * 4));   /* AFRH, pin 10 -> index 2 */
   GPIOB->AFR[1] |=  (7U   << ((10 - 8) * 4));   /* AF7 = USART3 */

  /*  PC5 (RX): MODER bits [11:10] -> Alternate Function mode (10) */
   GPIOC->MODER &= ~(3U << (5 * 2));
   GPIOC->MODER |=  (2U << (5 * 2));
   // Set the speed to very high
   GPIOC->OSPEEDR |= (3U << (5 * 2)); /* very high speed, optional */
   // Set as alternate Pin
   GPIOC->AFR[0] &= ~(0xFU << (5 * 4));          /* AFRL, pin 5 -> index 5 */
   GPIOC->AFR[0] |=  (7U   << (5 * 4));          /* AF7 = USART3 */

   /*  pull PC5 (RX) high when nothing is driving it, so a disconnected
      * wire reads as idle line instead of floating noise. */
     GPIOC->PUPDR &= ~(3U << (5 * 2));
     GPIOC->PUPDR |=  (1U << (5 * 2));
}

// Set the UART baudrate as well configurations
static void USART3_Init(void)
{
	   /* Enable clock to USART3 (APB1 bus) */
	    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

	    // reset every configurations
	    USART3->CR1 = 0;   /* start from a known state */
	    USART3->CR2 = 0;   /* 1 stop bit (STOP = 00), which is the reset value */
	    USART3->CR3 = 0;
	    //

	     /* --- Baud rate ---
	     * USARTDIV = PCLK1 / (16 * baud)
	     * With SystemClock_Config() in main.c: HSE=8MHz, PLLM=4, PLLN=180, PLLP=2
	     *   -> SYSCLK = 180 MHz, APB1CLKDivider = /4  -> PCLK1 = 45 MHz
	     * USARTDIV = 45,000,000 / (16 * 115200) = 24.4141
	     * Mantissa = 24 (0x18), Fraction = round(0.4141*16) = 7 (0x7)
	     * BRR = (24 << 4) | 7 = 0x187
	     *
	     * If SystemClock_Config() ever changes, PCLK1 changes and this must
	     * be recalculated (or read at runtime via HAL_RCC_GetPCLK1Freq()).
	     */
	    USART3->BRR =0x187;   // mantissa=24, fraction=7, for PCLK1=45MHz, 115200 baud


	    /* Frame format: M=0 (8 data bits) and PCE=0 (no parity) are both
	     * the reset defaults in CR1, so nothing to set there for 8N1. */

	    /* Enable receiver, and the "RX buffer not empty" interrupt */
	    USART3->CR1 |= USART_CR1_RE;       /* receiver enable  */
	    USART3->CR1 |= USART_CR1_TE;       /* transmitter enable (optional; drop if RX-only) */
	    USART3->CR1 |= USART_CR1_RXNEIE;   /* RXNE interrupt enable */

	    /* Enable the USART peripheral itself - do this last */
	    USART3->CR1 |= USART_CR1_UE;
	}


// Enable NVIC for USART3
static void NVIC_Init(void)
{
    NVIC_SetPriority(USART3_IRQn, 1);
    NVIC_EnableIRQ(USART3_IRQn);
}





static void NVIC_DeInit(void)
{
    NVIC_DisableIRQ(USART3_IRQn);
    NVIC_ClearPendingIRQ(USART3_IRQn);
}


static void USART3_DeInit(void)
{

	// Clear any status registers
    USART3->CR1 &= ~(USART_CR1_RXNEIE | USART_CR1_TXEIE | USART_CR1_TCIE |
                      USART_CR1_PEIE  | USART_CR1_IDLEIE);
    USART3->CR2 &= ~(USART_CR2_LBDIE);
    USART3->CR3 &= ~(USART_CR3_EIE  | USART_CR3_CTSIE);


    // wait for the Byte to transmit
    if (USART3->CR1 & USART_CR1_TE)
    {
        while (!(USART3->SR & USART_SR_TC))
        {
            /* wait for in-flight byte to finish shifting out */
        }
    }

    /* 2c. Disable transmitter, receiver, and the peripheral itself. */
    USART3->CR1 &= ~(USART_CR1_TE | USART_CR1_RE | USART_CR1_UE);


    //
    RCC->APB1RSTR |=  RCC_APB1RSTR_USART3RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_USART3RST;

    /*  Turn off the peripheral clock - nothing is using it now. */
    RCC->APB1ENR &= ~RCC_APB1ENR_USART3EN;
}


static void GPIO_DeInit(void)
{
    /* --- PB10 --- */
    GPIOB->MODER   &= ~(3U   << (10 * 2));        /* 00 = input (reset default) */
    GPIOB->OSPEEDR &= ~(3U   << (10 * 2));        /* 00 = low speed (reset default) */
    GPIOB->PUPDR   &= ~(3U   << (10 * 2));        /* 00 = no pull (reset default) */
    GPIOB->AFR[1]  &= ~(0xFU << ((10 - 8) * 4));  /* clear AF selection */

    /* --- PC5 --- */
    GPIOC->MODER   &= ~(3U   << (5 * 2));
    GPIOC->OSPEEDR &= ~(3U   << (5 * 2));
    GPIOC->PUPDR   &= ~(3U   << (5 * 2));
    GPIOC->AFR[0]  &= ~(0xFU << (5 * 4));

}


COM_Status_t uart_deinit(void)
{
    NVIC_DeInit();     /* 1. silence the interrupt first   */
    USART3_DeInit();   /* 2. reset the peripheral registers */
    GPIO_DeInit();      /* 3. release the pins                */

    return COM_OK ;

}



// this is isr defined in .s file
// this is isr defined in .s file
void USART3_IRQHandler(void)
{
    /* Read statue register for Overrun  */
    if (USART3->SR & USART_SR_ORE)
    {
        (void)USART3->DR;   /* dummy read clears ORE (along with SR read above) */
    }

    // check if Reciever has data keep it
    if (USART3->SR & USART_SR_RXNE)
    {
    	 rx_data = (uint8_t)(USART3->DR & 0xFF);  /* reading DR clears RXNE */

    	 // push data to FIFO
    	ring_buff_push(&ringbuff, rx_data) ;

    }
}

/*
 *
 * Initialize uart
 */
COM_Status_t uart_init()
{
	GPIO_Init(); // configure GPIO's
	USART3_Init(); // Configure USART3
	NVIC_Init(); // Configure interrupt
	return COM_OK ;
}


// returns number of bytes read
uint16_t uart_read(uint8_t *data,uint16_t length)
{
	if(data == NULL || length == 0 ) return 0 ;
	 uint16_t count = 0;
	    while (count < length)
	    {
	    	// pop bytes read bytes and store it in data buffer
	        if (!ring_buff_pop(&ringbuff, &data[count]))
	        {
	            break;      // Buffer empty
	        }

	        count++;
	    }
	    // reuturn bytes read
	    return count;
}




void USART3_WriteChar(uint8_t data)
{
	 unsigned int timeout = 0 ;
	// wait until data
    while (!(USART3->SR & USART_SR_TXE) &&  timeout <= MAX_TIMEOUT )
    {
        /* wait until DR is free to accept a new byte */
    	++timeout ;
    }
    if(timeout >= MAX_TIMEOUT) return  ;


    USART3->DR = data;
}


// return bytes written
uint16_t USART3_Write(const uint8_t *data, uint16_t len)
{
	 unsigned int timeout = 0 ;
	uint32_t i  = 0 ;
    for ( i = 0; i < len; i++)
    {
        USART3_WriteChar(data[i]);
    }

    /* Optional: block until the very last byte has fully left the shift
     * register (TC = Transmit Complete), */
    while (!(USART3->SR & USART_SR_TC) && timeout <= MAX_TIMEOUT )
    {
    	++timeout ;
    }
    // bytes written
    return i ;
}


void USART3_WriteString(const char *str)
{
    while (*str)
    {
        USART3_WriteChar((uint8_t)*str++);
    }
}

// use this to call static functions
const com_t com = {.open = uart_init , .close = uart_deinit , .read =  uart_read , .write = USART3_Write } ;

