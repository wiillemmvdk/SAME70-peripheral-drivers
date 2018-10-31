/**
 * \file
 *
 * \brief Syscalls for SAM (GCC).
 *
 * Copyright (c) 2011-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 *
 *
 * syscalls.c
 * the source of this file is ASF but the file has been made to work without the software framework for use with STDIO
 * See the linkage of the USART to stdout in USART.c
 *
 * helpfull links
 * https://balau82.wordpress.com/2010/12/16/using-newlib-in-arm-bare-metal-programs/
 * http://ww1.microchip.com/downloads/en/DeviceDoc/Frequently-Asked-Questions-4.9.3.26.txt
 * 
 */


#include <stdio.h>
#include <stdarg.h>
#include <sys/types.h>
#include <sys/stat.h>

#define WEAK __attribute__ ((weak))

#ifdef __cplusplus
extern "C" {
#endif


#undef errno
extern int errno;
extern int _end;
extern int __ram_end__;		/* Defined in linker as '__ram_end__ = ORIGIN(ram) + LENGTH(ram) - 4;' */

extern caddr_t _sbrk(int incr);
extern int link(char *old, char *new);
extern int _close(int file);
extern int _fstat(int file, struct stat *st);
extern int _isatty(int file);
extern int _lseek(int file, int ptr, int dir);
extern void _exit(int status);
extern void _kill(int pid, int sig);
extern int _getpid(void);

extern caddr_t _sbrk(int incr)
{
	static unsigned char *heap = NULL;
	unsigned char *prev_heap;
	int ramend = (int)&__ram_end__;

	if (heap == NULL) {
		heap = (unsigned char *)&_end;
	}
	prev_heap = heap;

	if (((int)prev_heap + incr) > ramend) {
		return (caddr_t) -1;	
	}

	heap += incr;

	return (caddr_t) prev_heap;
}

extern int link(char *old, char *new)
{
	return -1;
}

extern int _close(int file)
{
	return -1;
}

extern int _fstat(int file, struct stat *st)
{
	st->st_mode = S_IFCHR;

	return 0;
}

extern int _isatty(int file)
{
	return 1;
}

extern int _lseek(int file, int ptr, int dir)
{
	return 0;
}

extern void _exit(int status)
{
	asm("BKPT #0");
	for (;;);
}

extern void _kill(int pid, int sig)
{
	return;
}

extern int _getpid(void)
{
	return -1;
}


/* syscalls for printf */

volatile void *volatile stdio_base;

/* syscall _write */
int (*ptr_put)(void volatile*, char);

int __attribute__((weak))
_write (int file, const char *ptr, int len);

int __attribute__((weak))
_write (int file, const char *ptr, int len)
{
	int nChars = 0;

	if ((file != 1) && (file != 2) && (file!=3)) {
		return -1;
	}
	for (; len != 0; --len) {
		if (ptr_put(stdio_base, *ptr++) < 0) {
			return -1;
		}
		++nChars;
	}

	return nChars;
}


/* syscall _read */

void (*ptr_get)(void volatile*, char*);

int __attribute__((weak))
_read (int file, char * ptr, int len)
{
	int nChars = 0;

	if (file != 0) {
		return -1;
	}

	for (; len > 0; --len) {
		ptr_get(stdio_base, ptr);
		ptr++;
		nChars++;
	}
	return nChars;
}



#ifdef __cplusplus
}
#endif
