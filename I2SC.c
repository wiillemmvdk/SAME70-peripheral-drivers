/*
 * I2SC.c
 *
 * Created: 12-Oct-18 15:40:06
 *  Author: Willem van der Kooij
 */ 

//PING-PONG HOWTO?????????????	http://ww1.microchip.com/downloads/en/DeviceDoc/90003172A.pdf
	

#include "sam.h"
#include "I2SC.h"
#include "XDMAC.h"

#include <stdio.h>

static void I2SC_PioSetup(void);

#define I2SC_MASTER
//#define I2SC_SLAVE

#define NUM_BUFF_PER_CHANNEL	2

#define PING	0
#define PONG	1

volatile uint8_t pingpong = PING;

COMPILER_WORD_ALIGNED static lld_view1 lld_L_Tx[NUM_BUFF_PER_CHANNEL];
COMPILER_WORD_ALIGNED static lld_view1 lld_L_Rx[NUM_BUFF_PER_CHANNEL];
COMPILER_WORD_ALIGNED static lld_view1 lld_R_Tx[NUM_BUFF_PER_CHANNEL];
COMPILER_WORD_ALIGNED static lld_view1 lld_R_Rx[NUM_BUFF_PER_CHANNEL];



/* Use I2SC1 */
void I2SC_Init(void)
{
	I2SC_PioSetup();
	
	/* setup generic clock for I2S */
	PMC -> PMC_PCR		=	PMC_PCR_CMD
						|	PMC_PCR_GCLKCSS_MAIN_CLK
						|	PMC_PCR_GCLKDIV(0xFF)		
						|	PMC_PCR_PID(70)			//I2SC1
						|	PMC_PCR_GCLKEN
						|	PMC_PCR_EN;
	
	//MATRIX -> CCFG_PCCR &= ~(CCFG_PCCR_I2SC1CC);		//use periperal clock instead of programmable clock
		
	I2SC1 -> I2SC_CR	=	I2SC_CR_SWRST;
	
	#ifdef I2SC_MASTER
	I2SC1 -> I2SC_MR	=	I2SC_MR_MODE_MASTER
						|	I2SC_MR_DATALENGTH_24_BITS
						|	I2SC_MR_TXSAME
						|	I2SC_MR_TXDMA
						|	I2SC_MR_RXDMA
						//|	I2SC_MR_RXLOOP
						|	I2SC_MR_IMCKDIV(5)			//combined with IMCKFS(31) / 1024_Val gives 128fs master clock
						|	I2SC_MR_IMCKFS(I2SC_MR_IMCKFS_M2SF512_Val)	//combined with IMCKDIV(7) gives 128fs master clock
						|	I2SC_MR_IMCKMODE;
	#endif
	
	#ifdef I2SC_SLAVE
	I2SC1 -> I2SC_MR	=	I2SC_MR_MODE_SLAVE
						|	I2SC_MR_TXDMA
						|	I2SC_MR_RXDMA
						//|	I2SC_MR_RXLOOP
						|	I2SC_MR_DATALENGTH_24_BITS
						|	I2SC_MR_TXSAME;
	#endif

}

void I2SC_Start(void)
{
	I2SC1 -> I2SC_CR	=	0;
	I2SC1 -> I2SC_CR	|=	I2SC_CR_RXEN;	//enable Rx
	I2SC1 -> I2SC_CR	|=	I2SC_CR_TXEN;	//enable Tx
	I2SC1 -> I2SC_CR	|=	I2SC_CR_CKEN;	//enable clock
}


/*
 * Linked list desciptor is available in ASF example XDMAC_EXAMPLE1
 *
*/
void I2SC_StartWithDMA(I2S_data_pingpong *buffersetup, uint32_t DMAbuffLen)
{	
	/* setup DMA */
	DMA_configChannel_I2SC1(0, XDMAC_I2SC1_PERID_L_TX, XDMAC_PER_DATADIR_TX, buffersetup->data_out_left[0], NULL, DMAbuffLen);
	DMA_configChannel_I2SC1(1, XDMAC_I2SC1_PERID_L_RX, XDMAC_PER_DATADIR_RX, NULL, buffersetup->data_in_left[0], DMAbuffLen);
	DMA_configChannel_I2SC1(2, XDMAC_I2SC1_PERID_R_TX, XDMAC_PER_DATADIR_TX, buffersetup->data_out_right[0], NULL, DMAbuffLen);
	DMA_configChannel_I2SC1(3, XDMAC_I2SC1_PERID_R_RX, XDMAC_PER_DATADIR_RX, NULL, buffersetup->data_in_right[0], DMAbuffLen);


	/* enable linked lists */
   	DMA_LinkedListSetupI2SC1(0, XDMAC_PER_DATADIR_TX, lld_L_Tx, buffersetup->data_out_left[0], buffersetup->data_out_left[1], NULL, NULL, DMAbuffLen);
	DMA_LinkedListSetupI2SC1(1, XDMAC_PER_DATADIR_RX, lld_L_Rx, NULL, NULL, buffersetup->data_in_left[0], buffersetup->data_in_left[0], DMAbuffLen);
    DMA_LinkedListSetupI2SC1(2, XDMAC_PER_DATADIR_TX, lld_R_Tx, buffersetup->data_out_right[0], buffersetup->data_out_right[1], NULL, NULL, DMAbuffLen);
 	DMA_LinkedListSetupI2SC1(3, XDMAC_PER_DATADIR_RX, lld_R_Rx, NULL, NULL, buffersetup->data_in_right[0], buffersetup->data_in_right[0], DMAbuffLen);

	DMA_enableInterrupt(0, XDMAC_CIE_BIE);
	DMA_enableInterrupt(1, XDMAC_CIE_BIE);
	DMA_enableInterrupt(2, XDMAC_CIE_BIE);
	DMA_enableInterrupt(3, XDMAC_CIE_BIE);

	//enable interrupts for XDMAC
	NVIC_ClearPendingIRQ(XDMAC_IRQn);
	NVIC_SetPriority(XDMAC_IRQn, 3);
	NVIC_EnableIRQ(XDMAC_IRQn);
	
	/* enable DMA channels */
 	DMA_enableChannel(0x0F);
	
	I2SC_Start();			
											
}

/* Pinout
 * MCK		PA19	@ peripheral D
 * SCK		PA20	@ peripheral D
 * WS		PE0		@ peripheral C
 * SDI		PE2		@ peripheral C
 * SDO		PE1		@ peripheral C
 */
static void I2SC_PioSetup(void)
{
	PIOA -> PIO_PDR	|=	PIO_PDR_P19
					|	PIO_PDR_P20;
					
	//to peripheral D
	PIOA -> PIO_ABCDSR[0] |= PIO_ABCDSR_P19 | PIO_ABCDSR_P20;
	PIOA -> PIO_ABCDSR[1] |= PIO_ABCDSR_P19 | PIO_ABCDSR_P20;			
					
	PIOE -> PIO_PDR		|=	PIO_PDR_P0
						|	PIO_PDR_P1
						|	PIO_PDR_P2;
									
	//to peripheral C
	PIOE -> PIO_ABCDSR[0] &= ~	(PIO_ABCDSR_P0 | PIO_ABCDSR_P1 | PIO_ABCDSR_P2);
	PIOE -> PIO_ABCDSR[1] |=	PIO_ABCDSR_P0 | PIO_ABCDSR_P1 | PIO_ABCDSR_P2;					 
} 

/*
 *	returns 0 is pongpong = PING
 *	returns 1 if pingpong = PONG
 */
uint8_t I2SC_DMA_getPingPong(void)
{
	return pingpong;
}

void XDMAC_Handler(void)
{
	//read status registers to clear interrupt flag
	XDMAC -> XDMAC_CHID[0].XDMAC_CIS;
	XDMAC -> XDMAC_CHID[1].XDMAC_CIS;
	XDMAC -> XDMAC_CHID[2].XDMAC_CIS;

	//only switch buffers when one of the channels completion to overcome switching 4 times when interrupts dont fire at the sime time
	if (XDMAC -> XDMAC_CHID[3].XDMAC_CIS) {
		if (pingpong == PING) {
			pingpong = PONG;
		} else { //must be PONG
			pingpong = PING;
		}
	}


	
}