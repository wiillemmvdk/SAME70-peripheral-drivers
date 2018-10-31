/*
 * USART.c
 *
 * Created: 05-Oct-18 14:04:49
 *  Author: Willem van der Kooij
 *
 * CM style driver based on the ISR driven AVR UART driver by JDB
 */ 

#include "sam.h"
#include "usart.h"

#include <stdio.h>
#include <string.h>

#define F_MCK		150000000
#define UART_BAUD	230400

#define TXBUF_DEPTH_CTRL	750
#define RXBUF_DEPTH_CTRL	80

static inline int Ctrl_putchar(void volatile* usart, const char c);

static volatile uint8_t tx_ctrl_wridx, tx_ctrl_rdidx, tx_ctrl_buf[TXBUF_DEPTH_CTRL];
static volatile uint8_t rx_ctrl_wridx, rx_ctrl_rdidx, rx_ctrl_buf[RXBUF_DEPTH_CTRL];

extern volatile void *volatile stdio_base;
extern int (*ptr_put)(void volatile*, char) = &Ctrl_putchar;	//function pointer to putc for syscalls


void USARTinit(void)
{
	//PA21 USART1 Rx
	PIOA -> PIO_PUER	|=	PIO_PUER_P21;
	PIOA -> PIO_PDR		|=	PIO_PDR_P21;
	
	PIOA -> PIO_ABCDSR[0] &= ~(PIO_ABCDSR_P21);	//Rx @ PA21 to peripheral A
	PIOA -> PIO_ABCDSR[1] &= ~(PIO_ABCDSR_P21);
	
	//PB4 USART1 Tx
	PIOB -> PIO_PUER	|=	PIO_PUER_P4;		//enable pullup
	PIOB -> PIO_PDR		|=	PIO_PDR_P4;
	
	PIOB -> PIO_ABCDSR[0]	|=  (PIO_ABCDSR_P4);	//TX @ PB4  / peripheral_D
	PIOB -> PIO_ABCDSR[1]	|=  (PIO_ABCDSR_P4);
	
	MATRIX -> CCFG_SYSIO |= CCFG_SYSIO_SYSIO4; //use PB4 as IO -> NOT as TDI
	
	//enable peripheral clock
	PMC -> PMC_PCER0 = (0x01 << 14);			//PID14 = USART1 
	
	USART1 -> US_WPMR	&= ~(US_WPMR_WPEN);
	USART1 -> US_WPMR	= US_WPMR_WPKEY_PASSWD;
	
	//setup baud		manual calc shows BRGR=40.69 so CD=40 & FP = 6
	//USART1 -> US_BRGR	= US_BRGR_CD( (F_MCK / (16 * UART_BAUD)) ); //CLKDIV	
	USART1 -> US_BRGR	= US_BRGR_CD(40) //CLKDIV
						| US_BRGR_FP(6);
	
	USART1 -> US_MR		= US_MR_USART_MODE_NORMAL
						| US_MR_USCLKS_MCK
						| US_MR_CHRL_8_BIT
						| US_MR_PAR_NO
						| US_MR_NBSTOP_1_BIT
						| US_MR_CHMODE_NORMAL;
						
	USART1 -> US_IER	= US_IER_RXRDY
						| US_IER_TXEMPTY;
						
	
	NVIC_ClearPendingIRQ(USART1_IRQn);
	NVIC_SetPriority(USART1_IRQn, 7);
	NVIC_EnableIRQ(USART1_IRQn);
							
	USART1 -> US_CR		= US_CR_TXEN
						| US_CR_RXEN;	
	
	/* Setup stream for printf (link syscalls) */
	stdio_base = (void *)USART1;
	setbuf(stdout, NULL);	//dont buffer stdout
	setbuf(stdin, NULL);	//dont buffer stdin	
}


void UART_Puts(const char *str)
{
	for (uint8_t i = 0; i < strlen(str); i++) {
		Ctrl_putchar(USART1, str[i]);
	}
}

#define TOHEX(x) (((x) & 0x0F) <= 9 ? ('0' + ((x) & 0x0F)) : ('A' + ((x) & 0x0F) - 10))

static inline int Ctrl_putchar(void volatile* usart, const char c)
{
	static uint8_t checksum;
	
	WriteByte_Ctrl((uint8_t) c);
	
	if(c == '>') {
		checksum = 0;
	}
	else if(c == '<') {
		WriteByte_Ctrl(TOHEX((uint8_t) checksum >> 4));
		WriteByte_Ctrl(TOHEX((uint8_t) checksum));
	}
	else {
		checksum ^= (uint8_t) c;
	}
	
	return 0;
	
}


uint8_t CanRead_Ctrl(void)
{
	uint8_t wridx = rx_ctrl_wridx, rdidx = rx_ctrl_rdidx;
	
	if(wridx >= rdidx)
	return wridx - rdidx;
	else
	return wridx - rdidx + RXBUF_DEPTH_CTRL;
	
} /* CanRead_Ctrl */


uint8_t ReadByte_Ctrl(void)
{
	uint8_t res, curSlot, nextSlot;
	
	curSlot = rx_ctrl_rdidx;
	/* Busy-wait for a byte to be available. Should not be necessary if the caller calls CanRead_xxx() first */
	while(!CanRead_Ctrl()) ;
		
	res = rx_ctrl_buf[curSlot];
	nextSlot = curSlot + 1;
	if(nextSlot >= RXBUF_DEPTH_CTRL)
		nextSlot = 0;
		
	rx_ctrl_rdidx = nextSlot;
	return res;
} /* ReadByte_Ctrl */


uint8_t CanWrite_Ctrl(void)
{
	uint8_t wridx1 = tx_ctrl_wridx + 1, rdidx = tx_ctrl_rdidx;
			
	if(wridx1 >= TXBUF_DEPTH_CTRL)
		wridx1 -= TXBUF_DEPTH_CTRL;
	if(rdidx >= wridx1)
		return rdidx - wridx1;
	else
		return rdidx - wridx1 + TXBUF_DEPTH_CTRL;
			
} /* CanWrite_Ctrl */


void WriteByte_Ctrl(uint8_t data)
{
	uint8_t curSlot, nextSlot;
				
	/* Busy-wait for a byte to be available. Should not be necessary if the caller calls CanWrite_xxx() first */
	while(!CanWrite_Ctrl())
		USART1 -> US_IER |= US_IER_RXRDY | US_IER_TXEMPTY;
				
	curSlot = tx_ctrl_wridx;
	tx_ctrl_buf[curSlot] = data;
				
	nextSlot = curSlot + 1;
	if(nextSlot >= TXBUF_DEPTH_CTRL)
		nextSlot = 0;

	NVIC_DisableIRQ(USART1_IRQn);
	tx_ctrl_wridx = nextSlot;
	USART1 -> US_IER |= US_IER_RXRDY | US_IER_TXEMPTY;
	NVIC_EnableIRQ(USART1_IRQn);

} /* WriteByte_Ctrl */


void USART1_Handler(void)
{
	uint8_t curSlotTx, nextSlotTx, lastSlotTx, curSlotRx, nextSlotRx;
	
	
	if ((USART1 -> US_CSR & US_CSR_TXEMPTY) == US_CSR_TXEMPTY) {
		nextSlotTx = curSlotTx = tx_ctrl_rdidx;
		lastSlotTx = tx_ctrl_wridx;
		
		if(curSlotTx != lastSlotTx) {
			USART1 -> US_THR = tx_ctrl_buf[curSlotTx];
			nextSlotTx = curSlotTx + 1;
			if(nextSlotTx >= TXBUF_DEPTH_CTRL)
			nextSlotTx = 0;
		}
		
		if(nextSlotTx == lastSlotTx)
			USART1 -> US_IDR = US_IDR_TXEMPTY;
		
		tx_ctrl_rdidx = nextSlotTx;

	}
	
	if ((USART1 -> US_CSR & US_CSR_RXRDY) == US_CSR_RXRDY) {
		curSlotRx = rx_ctrl_wridx;
		rx_ctrl_buf[curSlotRx] = USART1 -> US_RHR;
		
		nextSlotRx = curSlotRx + 1;
		if(nextSlotRx >= RXBUF_DEPTH_CTRL)
		nextSlotRx = 0;
		
		if(nextSlotRx != rx_ctrl_rdidx)
		rx_ctrl_wridx = nextSlotRx;

	}
		
}