/*
 * retarget.c
 *
 *  Created on: Apr 9, 2026
 *      Author: Muhmmad Salman
 */


#include "retarget.h"
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <stdint.h>
#include <sys/stat.h>

static UART_HandleTypeDef *uartGlobal = NULL ;

/*
 *	to learn about this explore this link
 * https://www.peak-system.com/produktcd/Develop/Microcontroller%20hardware/PEAK-DevPack/Compiler/10%202020-q4-major/share/doc/gcc-arm-none-eabi/html/libc/Stubs.html#Stubs
 *
 */

void retarget_init(UART_HandleTypeDef *uart_x )
{
	if(uartGlobal  == NULL )
	{
		uartGlobal = uart_x ;
	}

	// disable I/O buffering : means character are flashed out
	// immediately
	setvbuf(stdout, NULL, _IONBF, 0);


}


 int _isatty(int fd) {
    // newlib calls this to check if fd is a terminal
    // Must return 1 (yes, it's a terminal) for fd 0,1,2
    // Otherwise: buffering behavior changes unexpectedly
    if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
        return 1;   // fd 0,1,2 = stdin,stdout,stderr = "terminal"
    errno = EBADF;  // Other fds: bad file descriptor error
    return 0;
}

 int _write(int fd, char* ptr, int len) {
     HAL_StatusTypeDef hstatus;

     // Only handle stdout (fd=1) and stderr (fd=2):
     if (fd == STDOUT_FILENO || fd == STDERR_FILENO) {
         hstatus = HAL_UART_Transmit(
        		 uartGlobal,           // Which UART (huart2 = VCP)
             (uint8_t*)ptr,    // Buffer to send
             len,              // Number of bytes
             HAL_MAX_DELAY     // Wait forever (blocking)
         );
        //  ITM_SendChar(ch) // using itm software trace
         if (hstatus == HAL_OK)
             return len;       // Success: return bytes written
         else
             return EIO;       // I/O error
     }
     errno = EBADF;
     return -1;                // Error: bad file descriptor
 }


 int _read(int fd, char* ptr, int len) {
     HAL_StatusTypeDef hstatus;

     // Only handle stdin (fd=0):
     if (fd == STDIN_FILENO) {
         hstatus = HAL_UART_Receive(
        		 uartGlobal,           // Which UART
             (uint8_t*)ptr,    // Receive buffer
             1,                // Receive exactly 1 byte
             HAL_MAX_DELAY     // Wait forever for keypress
         );
         // Note: reads 1 byte at a time
         // scanf() calls _read() repeatedly building input
         if (hstatus == HAL_OK)
             return 1;         // 1 byte received
         else
             return EIO;
     }
     errno = EBADF;
     return -1;
 }

 int _close(int fd) {
     // For stdin/stdout/stderr: close does nothing (they're always open)
     if (fd >= STDIN_FILENO && fd <= STDERR_FILENO)
         return 0;             // Success (nothing to do)
     errno = EBADF;
     return -1;
 }


 int _fstat(int fd, struct stat* st) {
     // Called by newlib to get file status
     // Must report stdin/stdout/stderr as character devices:
     if (fd >= STDIN_FILENO && fd <= STDERR_FILENO) {
         st->st_mode = S_IFCHR;  // Character device (not file)
         return 0;
     }
     errno = EBADF;
     return 0;  // Note: should be -1 for error but common practice
 }



