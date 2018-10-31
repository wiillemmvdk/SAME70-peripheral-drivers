/*
 * SDR_Drivers.c
 *
 * Created: 12-Oct-18 13:29:29
 * Author : Willem van der Kooij
 */ 

#include "sam.h"
#include <stdio.h>

#include "PMC.h"
#include "USART.h"
#include "I2SC.h"
#include "FPU.h"
#include "HSMCI.h"
#include "XDMAC.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "arm_common_tables.h"

static void BoardInit(void);
static void InitIO(void);

COMPILER_WORD_ALIGNED uint8_t readbuffer[HSMCI_SD_BLOCKSZ];		//div 4 since the buffer is 4 bytes
COMPILER_WORD_ALIGNED uint8_t writebuffer[HSMCI_SD_BLOCKSZ];

COMPILER_WORD_ALIGNED static I2S_data_pingpong I2S_buffers;
uint32_t I2S_tx_left_ping[I2SC_BUFFSZ], I2S_tx_left_pong[I2SC_BUFFSZ], I2S_rx_left_ping[I2SC_BUFFSZ], I2S_rx_left_pong[I2SC_BUFFSZ], I2S_tx_right_ping[I2SC_BUFFSZ], I2S_tx_right_pong[I2SC_BUFFSZ], I2S_rx_right_ping[I2SC_BUFFSZ], I2S_rx_right_pong[I2SC_BUFFSZ];


int main(void)
{
    /* Initialize the SAM system */
    SystemInit();
  	BoardInit();
	  
	  
	uint32_t block[(HSMCI_SD_BLOCKSZ / 4)];		// div 4 because of word size reg for byte size data
	
	for (uint32_t i = 0; i < (HSMCI_SD_BLOCKSZ / 4); i++) {
		block[i] = 0;
	}
	
	block[0] = 'h';
	block[1] = 'e';
	block[2] = 'l';
	block[3] = 'l';
	block[4] = 'o';
	block[5] = ' ';
	block[6] = 'w';
	block[7] = 'o';
	block[8] = 'r';
	block[9] = 'l';
	block[10] = 'd';
	
	SD_writeBlocks(block, 1, (312*512));
	
	for (uint32_t i = 0; i < (HSMCI_SD_BLOCKSZ / 4); i++) {
		block[i] = 0;
	}	
	
	SD_readBlocks(block, 1, (312*512));
	
	for (uint32_t i = 0; i < (HSMCI_SD_BLOCKSZ / 4); i++) {
		printf("%c", (char)block[i]);
	}

	while (1) {
		
		PIOC -> PIO_SODR	|= PIO_SODR_P8; //pull led low (will turn on)
		
		if (CanRead_Ctrl()) {
			char ctrlByte = ReadByte_Ctrl();
			switch (ctrlByte) {
				case 0x1B:
					NVIC_SystemReset();
					break;
				default:
					printf("%c",ctrlByte);
					break;
			}
		}
    }
}


static void BoardInit(void)
{
	/* Configure board */
	InitClock();
	WDT -> WDT_MR = WDT_MR_WDDIS;
	
	#if (__FPU_USED == 1)
		fpu_enable();		//enable FPU if present
	#endif
	
	InitPMC();
	InitIO();

	//green led @ pin C8
	PIOC -> PIO_PER		|= PIO_PER_P8;	//PIOC enable P8
	PIOC -> PIO_OER		|= PIO_OER_P8;	//PIOC Output enable P8
	PIOC -> PIO_SODR	|= PIO_CODR_P8; //pull led low (will turn on)

	/* Configure peripherals */
	USARTinit();

	//connect buffer to struct	
	I2S_buffers.data_out_left[0]	= (uint32_t)I2S_tx_left_ping;
	I2S_buffers.data_out_left[1]	= (uint32_t)I2S_tx_left_pong;
	I2S_buffers.data_in_left[0]		= (uint32_t)I2S_rx_left_ping;
	I2S_buffers.data_in_left[1]		= (uint32_t)I2S_rx_left_pong;
	I2S_buffers.data_out_right[0]	= (uint32_t)I2S_tx_right_ping;
	I2S_buffers.data_out_right[1]	= (uint32_t)I2S_tx_right_pong;
	I2S_buffers.data_in_right[0]	= (uint32_t)I2S_rx_right_ping;
	I2S_buffers.data_in_right[1]	= (uint32_t)I2S_rx_right_pong;
	
	for(uint32_t i = 0; i < I2SC_BUFFSZ; i++){
		I2S_tx_left_ping[i] = 0x1;
		I2S_tx_left_pong[i] = 0x1;
		I2S_tx_right_ping[i] = 0xFF;
		I2S_tx_right_pong[i] = 0xFF;
	}
	
	I2SC_Init();	I2SC_StartWithDMA(&I2S_buffers, I2SC_BUFFSZ);
	SD_CardInit();

}

/* \brief Init IO port registers passwords
 * IO's used for peripherals are redirected in the Init 
 */
static void InitIO (void) 
{
	/* PIOA */
	PIOA -> PIO_WPMR &= ~(PIO_WPMR_WPEN);
	PIOA -> PIO_WPMR = PIO_WPMR_WPKEY_PASSWD;	

	//GPSDO interrupt triggered by rising edge
	PMC -> PMC_PCER0	|= (0x01 << 10);	//pioA		
 	PIOA -> PIO_PUER	= PIO_PUER_P11;		//enable pullup
 	PIOA -> PIO_ESR		= PIO_ELSR_P11;		//ISR is a edge detect event
 	PIOA -> PIO_FELLSR	= PIO_FELLSR_P11;	//low level detect
	//PIOA -> PIO_REHLSR	= PIO_REHLSR_P11;	//rising edge detect 
 	PIOA -> PIO_AIMER	= PIO_AIMER_P11;	//interrupt source event is described in ELSR and FRLHSR
	PIOA -> PIO_ODR		= PIO_ODR_P11;		//make pin input (disable output)
	PIOA -> PIO_ISR;	//dummy read to clear status reg
 	PIOA -> PIO_IER		= PIO_IER_P11;		//enable input change ISR

	NVIC_ClearPendingIRQ(PIOA_IRQn);
	NVIC_SetPriority(PIOA_IRQn, 1);
	NVIC_EnableIRQ(PIOA_IRQn);			
	
	
	/* PIOB */
	PIOB -> PIO_WPMR &= ~(PIO_WPMR_WPEN);
	PIOB -> PIO_WPMR = PIO_WPMR_WPKEY_PASSWD;	
	
	/* PIOC */
	PIOC -> PIO_WPMR &= ~(PIO_WPMR_WPEN);
	PIOC -> PIO_WPMR = PIO_WPMR_WPKEY_PASSWD;
	
	/* PIOD */
	PIOD -> PIO_WPMR &= ~(PIO_WPMR_WPEN);
	PIOD -> PIO_WPMR = PIO_WPMR_WPKEY_PASSWD;
	
	/* PIOE */
	PIOE -> PIO_WPMR &= ~(PIO_WPMR_WPEN);
	PIOE -> PIO_WPMR = PIO_WPMR_WPKEY_PASSWD;	
	
}


/* 
 * ALWAYS READ PIO_ISR to clear interrupt flag!!!!
 */
void PIOA_Handler(void)
{
	static int blink = 0;

	//PA11 switch0
	if ((PIOA -> PIO_ISR & PIO_ISR_P11) == PIO_ISR_P11)	{
		if (blink) {
			PIOC -> PIO_CODR	|= PIO_CODR_P8; //pull led low (will turn on)
			blink = 0;
		} else {
			PIOC -> PIO_SODR	|= PIO_SODR_P8; //pull led low (will turn on)
			blink = 1;
		}
			
	}
}