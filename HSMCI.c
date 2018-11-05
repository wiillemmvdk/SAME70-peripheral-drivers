/*
 * HSMCI.c
 *
 * Created: 16-Oct-18 19:09:04
 *  Author: Willem van der Kooij
 */ 

//http://web.mit.edu/freebsd/head/sys/boot/arm/at91/libat91/sd-card.c
//http://www.at91.com/viewtopic.php?t=22990

#include "sam.h"
#include "HSMCI.h"
#include "XDMAC.h"
#include "sd_mmc_protocol.h"
#include "FatFS/ff.h"
#include "./FatFS/diskio.h"

#include <stdio.h>

static volatile DSTATUS Stat = STA_NOINIT;	//disk status

//#define PRINT_CARD_DEBUG

void HSMCI_Init(void);
void HSMCI_sendClock(void);
uint8_t SD_getOperatingCondition(uint8_t v2);
uint16_t SD_getDeviceAddr(void);
uint32_t SD_getMaxTransferSpeed(void);
uint32_t SD_getCapacity(void);
uint32_t SD_getVersion(void);
uint8_t SD_SetHighSpeed(void);
uint32_t HSMCI_getResponse(void);
void HSMCI_getResponse_128(uint8_t *response);
uint32_t HSMCI_sendCommand(uint32_t cmdr,uint32_t cmd, uint32_t arg);
uint32_t HSMCI_WaitEndBlockTransfer(void);
uint8_t HSMCI_sendAppCommand(uint32_t cmd_App, uint32_t arg);
void HSMCI_adtc_start(uint32_t cmd, uint32_t arg, uint16_t blocksize, uint16_t num_blocks, uint8_t use_DMA);
uint8_t HSMCI_SD_WaitReady(void);

void HSMCI_readBlocks(void *dest, uint16_t num_blocks);
void HSMCI_writeBlocks(const void *src, uint16_t num_blocks);

/* Card data */
struct {
	uint32_t	relative_addr;
	uint32_t	CID[4];
	uint32_t	capacity;
	uint8_t		csd[CSD_REG_BSIZE];		//csd = card specific data 
	uint8_t		version;
	uint32_t	max_clk_speed;
	uint8_t		type;
} sd_card_info;


//variables used between functions for transfers
static uint32_t hsmci_transferPos;
static uint16_t hsmci_blocksize, hsmci_numBlocks;

void HSMCI_Init(void)
{
	PIOA -> PIO_PDR	|=	PIO_PDR_P25		// CLK		peripheral D
					|	PIO_PDR_P26		// DAT2		peripheral C
					|	PIO_PDR_P27		// DAT3		peripheral C
					|	PIO_PDR_P28		// CMD		peripheral C
					|	PIO_PDR_P30		// DAT0		peripheral C
					|	PIO_PDR_P31;	// DAT1		peripheral C
	
	PIOA -> PIO_ABCDSR[0] &= ~	(PIO_ABCDSR_P26 | PIO_ABCDSR_P27 | PIO_ABCDSR_P28 | PIO_ABCDSR_P30 | PIO_ABCDSR_P31);
	PIOA -> PIO_ABCDSR[0] |=	PIO_ABCDSR_P25;
	PIOA -> PIO_ABCDSR[1] |=	PIO_ABCDSR_P25 | PIO_ABCDSR_P26 | PIO_ABCDSR_P27 | PIO_ABCDSR_P28 | PIO_ABCDSR_P30 | PIO_ABCDSR_P31;
	
	// make card detect input
	//P... used for card detect
	
	//enable peripheral clock
	PMC -> PMC_PCER0 = PMC_PCER0_PID18;			//PID18 = MMC interface

	if ((PMC -> PMC_PCSR1 & PMC_PCSR1_PID58) != PMC_PCSR1_PID58) {	//clock is not enabled
		PMC -> PMC_PCER1 |= PMC_PCER1_PID58;						//enable XDMAC peripheral clock
	}
	
	//unlock registers
	HSMCI -> HSMCI_WPMR &= ~HSMCI_WPMR_WPEN;
	HSMCI -> HSMCI_WPMR =	HSMCI_WPMR_WPKEY_PASSWD;
		
	//perform software reset
	HSMCI -> HSMCI_CR	=	HSMCI_CR_SWRST;
	
	//setup timeouts
	HSMCI -> HSMCI_DTOR = HSMCI_DTOR_DTOMUL_1048576 | HSMCI_DTOR_DTOCYC(2);			// Set the Data Timeout Register to 2 Mega Cycles
	HSMCI -> HSMCI_CSTOR = HSMCI_CSTOR_CSTOMUL_1048576 | HSMCI_CSTOR_CSTOCYC(2);	// Set Completion Signal Timeout to 2 Mega Cycles

	//set config
	HSMCI -> HSMCI_CFG	= HSMCI_CFG_FIFOMODE				// start write as soon as data is written to fifo
						| HSMCI_CFG_FERRCTRL;
	
	HSMCI -> HSMCI_BLKR = HSMCI_BLKR_BLKLEN(HSMCI_SD_BLOCKSZ);	// setup block length to 512 until the exact size is known
	
	HSMCI -> HSMCI_SDCR	= HSMCI_SDCR_SDCSEL_SLOTA			// there only is slotA so thats easy
						| HSMCI_SDCR_SDCBUS_1;				// 1-bit wide bus; use 1-bit until we know for sure card can handle 4bit mode
	
	HSMCI -> HSMCI_MR	= HSMCI_MR_CLKDIV( ((150000000 / 2) / (SDMMC_CLOCK_INIT)) - 2 )		// setup clock div	(150000000 = MCK freq usually )
						| HSMCI_MR_CLKODD					// LSB of clkdiv
						| HSMCI_MR_PWSDIV(0xFFF);			// max value for power save
						
	//enable the multi-media interface
	HSMCI -> HSMCI_CR	=	HSMCI_CR_MCIEN | HSMCI_CR_PWSEN;
	
	
	HSMCI -> HSMCI_MR	|=  HSMCI_MR_RDPROOF		// enable RDPROOF and stop clock if fifo is full to guarantee data integrity
						|	HSMCI_MR_WRPROOF;		// enable WRPROOF and stop clock if fifo is full to guarantee data integrity
}

void HSMCI_sendClock(void)
{
	HSMCI -> HSMCI_MR &= ~(HSMCI_MR_WRPROOF | HSMCI_MR_RDPROOF | HSMCI_MR_FBYTE);						//configure command
	HSMCI->HSMCI_ARGR = 0;																				//write arg	
	HSMCI->HSMCI_CMDR = HSMCI_CMDR_RSPTYP_NORESP | HSMCI_CMDR_SPCMD_INIT | HSMCI_CMDR_OPDCMD_OPENDRAIN;	// Write and start initialization command

	while (!(HSMCI->HSMCI_SR & HSMCI_SR_CMDRDY));														// Wait end of initialization command
}

uint32_t HSMCI_getResponse(void)
{
	return (HSMCI -> HSMCI_RSPR[0]);
}

void HSMCI_getResponse_128(uint8_t *response)
{
	uint32_t response_32;

	for (uint8_t i = 0; i < 4; i++) {
		response_32 = HSMCI -> HSMCI_RSPR[0];	//read response reg [0] auto increments upon consecutive reads of register DO NOT DO THIS MANUALLY!
		
		*response = (response_32 >> 24) & 0xFF;	//write 32 bit value in 8bit chunks to *response
		response++;
		*response = (response_32 >> 16) & 0xFF;
		response++;
		*response = (response_32 >>  8) & 0xFF;
		response++;
		*response = (response_32 >>  0) & 0xFF;
		response++;
	}	
}

/* Multiple command types
 * bc	: broadcast command, no response
 * br	: broadcast command with response
 * ac	: addressed command (point to point)	No data transfer on DAT
 * adtc : addressed data transfer command	Data transfer on DAT
*/
uint32_t HSMCI_sendCommand(uint32_t cmdr, uint32_t cmd, uint32_t arg)
{
	
	cmdr	|=	HSMCI_CMDR_CMDNB(cmd)	//set command
			|	HSMCI_CMDR_SPCMD_STD;	//standard command

	//check command response and command definitions
	if (cmd & SDMMC_RESP_PRESENT) {
		cmdr |= HSMCI_CMDR_MAXLAT;
		
		if (cmd & SDMMC_RESP_136) {
			cmdr |= HSMCI_CMDR_RSPTYP_136_BIT;
		} else if (cmd & SDMMC_RESP_BUSY) {
			cmdr |= HSMCI_CMDR_RSPTYP_R1B;
		} else {
			cmdr |= HSMCI_CMDR_RSPTYP_48_BIT;
		}
	}
	
	if (cmd & SDMMC_CMD_OPENDRAIN) {
		cmdr |= HSMCI_CMDR_OPDCMD_OPENDRAIN;
	}
	
	//write arg
	HSMCI -> HSMCI_ARGR = arg;
	
	//write and start command
	HSMCI -> HSMCI_CMDR = cmdr;
	
	while (! (HSMCI -> HSMCI_SR & HSMCI_SR_CMDRDY) );			// wait for command to be send
	
	if (cmd & SDMMC_RESP_BUSY) {
		while (! (HSMCI -> HSMCI_SR & HSMCI_SR_NOTBUSY) );		//ready for new data transfer
	}

	return 1;		//return cmd send
}

uint8_t HSMCI_sendAppCommand(uint32_t cmd_App, uint32_t arg)
{
	HSMCI_sendCommand(0, SDMMC_CMD55_APP_CMD, ((uint32_t)(sd_card_info.relative_addr << 16)));	//indicate special command is comming
	HSMCI_sendCommand(0, cmd_App, arg);		
	
	return 1;													
}

uint8_t SD_getOperatingCondition(uint8_t v2)
{
	uint32_t response;
	uint32_t retry = 2100;
	
	do {
		if (v2)	{
			HSMCI_sendAppCommand(SD_MCI_ACMD41_SD_SEND_OP_COND, (OCR_VDD_27_28 | OCR_VDD_28_29 | OCR_VDD_29_30 | OCR_VDD_30_31 | OCR_VDD_31_32 | OCR_VDD_32_33 | SD_ACMD41_HCS));
		} else {
			HSMCI_sendAppCommand(SD_MCI_ACMD41_SD_SEND_OP_COND, (OCR_VDD_27_28 | OCR_VDD_28_29 | OCR_VDD_29_30 | OCR_VDD_30_31 | OCR_VDD_31_32 | OCR_VDD_32_33));
		}
		
		response = HSMCI_getResponse();
		if (response & OCR_POWER_UP_BUSY) {
			if ((response & OCR_CCS) != 0) {
				sd_card_info.type |= CARD_TYPE_HC;
			}
			break;		//card is ready
		}

		if (retry-- == 0) {
			return 0;
		}
	} while (1);	
	
	return 1;
}

uint16_t SD_getDeviceAddr(void)
{
	HSMCI_sendCommand(0, SDMMC_CMD2_ALL_SEND_CID, 0);			//read CID number to identify card, so this by sending CMD2 (page 1047 datasheet table 40-4 and 40-5)
	sd_card_info.CID[0] = HSMCI_getResponse();
	sd_card_info.CID[1] = HSMCI_getResponse();
	sd_card_info.CID[2] = HSMCI_getResponse();
	sd_card_info.CID[3] = HSMCI_getResponse();
	HSMCI_sendCommand(0, SD_CMD3_SEND_RELATIVE_ADDR, 0);		//ask the card to publish new relative addr
	return ((HSMCI_getResponse() >> 16) & 0xFFFF);				//save relative addr		
}

uint32_t SD_getMaxTransferSpeed(void)
{
	/* setup cards max transfer speed */
	uint32_t clk_transfer_speed, clk_unit_transfer, clk_multiplier;

	clk_transfer_speed = CSD_TRAN_SPEED(sd_card_info.csd);
	clk_unit_transfer = sd_mmc_trans_units[clk_transfer_speed & 0x07];
	clk_multiplier = sd_trans_multipliers[(clk_transfer_speed >> 3) & 0x0F];
	
	return clk_unit_transfer * clk_multiplier * 1000;	
}

uint32_t SD_getCapacity(void)
{
	if (CSD_STRUCTURE_VERSION(sd_card_info.csd) >= SD_CSD_VER_2_0) {
		return (SD_CSD_2_0_C_SIZE( sd_card_info.csd ) + 1) * 512;
	} else {
		uint32_t blocknr = ((SD_CSD_1_0_C_SIZE( sd_card_info.csd ) + 1) * (1 << (SD_CSD_1_0_C_SIZE_MULT(sd_card_info.csd) + 2)));
		return blocknr * (1 << SD_CSD_1_0_READ_BL_LEN(sd_card_info.csd)) / 1024;
	}	
}

uint8_t scr[SD_SCR_REG_BSIZE] = {0,0,0,0,0,0,0,0};
	
uint32_t SD_getVersion(void)
{
	uint32_t cardversion = 0;
	
	HSMCI_sendCommand(0, SDMMC_CMD55_APP_CMD, (uint32_t)sd_card_info.relative_addr << 16);
	HSMCI_adtc_start(SD_ACMD51_SEND_SCR, 0, SD_SCR_REG_BSIZE, 1, 1);
	
 	HSMCI_readBlocks(scr, 1);
	HSMCI_WaitEndBlockTransfer();
	
	switch(SD_SCR_SD_SPEC(scr)) {
		case SD_SCR_SD_SPEC_1_0_01:
			cardversion = CARD_VER_SD_1_0;
			break;
		case SD_SCR_SD_SPEC_1_10:
			cardversion = CARD_VER_SD_1_10;
			break;
		case SD_SCR_SD_SPEC_2_00:
			if (SD_SCR_SD_SPEC(scr) == SD_SCR_SD_SPEC_3_00) {
				cardversion = CARD_VER_SD_3_0;
			} else {
				cardversion = CARD_VER_SD_2_0;
			}
			break;
			
		default:
			cardversion = CARD_VER_SD_1_0;
		break;
	}	
		
	return cardversion;
}

uint8_t switch_status[SD_SW_STATUS_BSIZE] = {0};

uint8_t SD_SetHighSpeed(void)
{

	HSMCI_adtc_start( SD_CMD6_SWITCH_FUNC, SD_CMD6_MODE_SWITCH | SD_CMD6_GRP6_NO_INFLUENCE | SD_CMD6_GRP5_NO_INFLUENCE | SD_CMD6_GRP4_NO_INFLUENCE 
												| SD_CMD6_GRP3_NO_INFLUENCE | SD_CMD6_GRP2_DEFAULT | SD_CMD6_GRP1_HIGH_SPEED, SD_SW_STATUS_BSIZE, 1 ,1);
	HSMCI_readBlocks(switch_status, 1);
	HSMCI_WaitEndBlockTransfer();								
	

	//give Card time to switch over by sending clock cycles					
	HSMCI_sendClock();
	
	sd_card_info.max_clk_speed *= 2;	//double the max speed
	
	//configure HSMCI CLK
	uint32_t clkdiv = (((150000000 / 2) / (sd_card_info.max_clk_speed)) - 2);
	HSMCI -> HSMCI_MR	= HSMCI_MR_CLKDIV(clkdiv)			// setup clock div
						| HSMCI_MR_CLKODD					// LSB of clkdiv
						| HSMCI_MR_PWSDIV_Msk;				// power save
	
	return 1;
}

DSTATUS SD_CardInit(void)
{	
	sd_card_info.type = CARD_TYPE_SD;			// assume card is SD
	sd_card_info.version = CARD_VER_UNKNOWN;	// of unknown version
	sd_card_info.relative_addr = 0;				// and unknown address
	
	uint8_t v2 = 0;
	uint32_t response;
	
	HSMCI_Init();
	
	HSMCI_sendClock();	//send 74 clock cycles to start card
	
	HSMCI_sendCommand(0, SDMMC_MCI_CMD0_GO_IDLE_STATE, 0);									// reset cards to idle state

	HSMCI_sendCommand(0, SD_CMD8_SEND_IF_COND,	SD_CMD8_PATTERN | SD_CMD8_HIGH_VOLTAGE);	// test for SD v2	
	response = HSMCI_getResponse();
	if (response == 0xFFFFFFFF || ((response & (SD_CMD8_MASK_PATTERN | SD_CMD8_MASK_VOLTAGE)) != (SD_CMD8_PATTERN | SD_CMD8_HIGH_VOLTAGE)) ) {
		v2 = 0;
	} else v2 = 1;
	
	if (sd_card_info.type & CARD_TYPE_SD) {
		SD_getOperatingCondition(v2);						// get operating conditions, check if card is high capacity
	}
	
	sd_card_info.relative_addr = SD_getDeviceAddr();	// ask card for CID number and relative card address 
	
	if (sd_card_info.relative_addr == 0) {	//card not responding or none found stop function if true
		Stat |= STA_NODISK;
		return Stat;
	}

	/* ****************************************************************
	///////////////////////////////////////////////////////////////////
	Initialization mode of card is over here, card now in standby state 
	\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
	**************************************************************** */
	
	/* Read card specific data */
	HSMCI_sendCommand(0, SDMMC_MCI_CMD9_SEND_CSD, ((uint32_t)(sd_card_info.relative_addr << 16)));
	HSMCI_getResponse_128(sd_card_info.csd);
	
	sd_card_info.max_clk_speed = SD_getMaxTransferSpeed();
	sd_card_info.capacity = SD_getCapacity();
	
	
	/* Get SD version by reading config reg*/
	HSMCI_sendCommand(0, SDMMC_CMD7_SELECT_CARD_CMD, ((uint32_t)sd_card_info.relative_addr << 16));		//select the card to put it in data transfer mode
	sd_card_info.version = SD_getVersion();
	
	/* enable bus width of 4bits */
	HSMCI_sendAppCommand(SD_ACMD6_SET_BUS_WIDTH, 0x02);		//hex 2 = 4 bits bus
	HSMCI -> HSMCI_SDCR	|= HSMCI_SDCR_SDCBUS_4;				//switch HSMCI to 4 bit wide bus
	
	
	/* enable high speed mode if possible */
	if ((sd_card_info.version > CARD_VER_SD_1_0) && (sd_card_info.type & CARD_TYPE_SD)) {
		SD_SetHighSpeed();
	}
	
	//set default blocksize
	HSMCI_sendCommand(0, SDMMC_CMD16_SET_BLOCKLEN, HSMCI_SD_BLOCKSZ);
	
	
	/*****************************************************************
	///////////////////////////////////////////////////////////////////
	Card is initialized and should be ready for data transfers	
	\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
	**************************************************************** */

#ifdef PRINT_CARD_DEBUG
	//print card info
	printf("Card info\r\n");
	
	if (sd_card_info.type & CARD_TYPE_HC) {			
		printf("    type : SDHC\r\n");
	} else printf("    type : SD\r\n");
		
	printf("    CID  : %lu%lu%lu%lu\r\n", sd_card_info.CID[0],sd_card_info.CID[1],sd_card_info.CID[2],sd_card_info.CID[3]);
	printf("    RCA  : %lu\r\n",sd_card_info.relative_addr);
	printf("    v2   : %d\r\n", v2);
	printf("    Size : %lu MB\r\n", (sd_card_info.capacity / 1024));
	printf("    CLK  : %lu Hz\r\n", (sd_card_info.max_clk_speed));
	
	switch(sd_card_info.version) {
		case SD_SCR_SD_SPEC_1_0_01:
			printf("    versn: 1\r\n");
			break;
		case SD_SCR_SD_SPEC_1_10:
			printf("    versn: 1.1\r\n");
			break;
		case SD_SCR_SD_SPEC_2_00:
			printf("    versn: 2\r\n");
			break;
	}
		
// 	for (uint8_t i = 0; i < SD_SCR_REG_BSIZE; i++) {
// 		printf("scr[%d] : %d\r\n", i, scr[i]);
// 	}
// 	
// 	for (uint8_t i = 0; i < SD_SW_STATUS_BSIZE; i++) {
// 		printf("switch_stat[%d] : %d\r\n", i, switch_status[i]);
// 	}
	
#endif	
	Stat &= ~STA_NOINIT;	
	return Stat;
}

uint32_t HSMCI_WaitEndBlockTransfer(void)
{
	XDMAC -> XDMAC_CHID[HSMCI_XDMAC_CH].XDMAC_CIS; //clear flag
	uint32_t timeout = 0xFFFFF;
	
	do {
		if ( (HSMCI -> HSMCI_SR) & (HSMCI_SR_UNRE | HSMCI_SR_OVRE | HSMCI_SR_DTOE | HSMCI_SR_DCRCE) ) {		//also do a hardware timout check
			XDMAC -> XDMAC_GD = (XDMAC_GD_DI0 << HSMCI_XDMAC_CH);	//disable DMA channel, because no point in finishing after error is given
			return 0;
		}
		if ( ((uint32_t)hsmci_blocksize * hsmci_numBlocks) > hsmci_transferPos ) {	// This was not the last block of data
			if (XDMAC -> XDMAC_CHID[HSMCI_XDMAC_CH].XDMAC_CIS & XDMAC_CIS_BIS) {
				return 1;
			}	
		}
		if (--timeout == 0) {
			break;
		}
	} while ( ! ( (HSMCI -> HSMCI_SR) & HSMCI_SR_XFRDONE) );
	
	if (!timeout) {	//timeout reached
		return 0;
	} else return 1;
}

//start data transfer command
void HSMCI_adtc_start(uint32_t cmd, uint32_t arg, uint16_t blocksize, uint16_t num_blocks, uint8_t use_DMA)
{
	uint32_t cmdr;
	
	if (use_DMA) {
		HSMCI->HSMCI_DMA = HSMCI_DMA_DMAEN;
	} else {
		HSMCI->HSMCI_DMA = 0;
	}
	
	// Enabling Read/Write Proof allows to stop the HSMCI Clock during
	// read/write  access if the internal FIFO is full.
	// This will guarantee data integrity, not bandwidth.
	HSMCI->HSMCI_MR |= HSMCI_MR_WRPROOF | HSMCI_MR_RDPROOF;
	// Force byte transfer if needed
	if (blocksize & 0x3) {
		HSMCI->HSMCI_MR |= HSMCI_MR_FBYTE;
		} else {
		HSMCI->HSMCI_MR &= ~HSMCI_MR_FBYTE;
	}
	
	
	if (cmd & SDMMC_CMD_WRITE) {
		cmdr = HSMCI_CMDR_TRCMD_START_DATA | HSMCI_CMDR_TRDIR_WRITE;
		} else {
		cmdr = HSMCI_CMDR_TRCMD_START_DATA | HSMCI_CMDR_TRDIR_READ;
	}	
	
	if (cmd & SDMMC_CMD_SDIO_BYTE) {
		cmdr |= HSMCI_CMDR_TRTYP_BYTE;
		// Value 0 corresponds to a 512-byte transfer
		HSMCI->HSMCI_BLKR = ((blocksize % 512) << HSMCI_BLKR_BCNT_Pos);
		} else {
		HSMCI->HSMCI_BLKR = (blocksize << HSMCI_BLKR_BLKLEN_Pos) |	(num_blocks << HSMCI_BLKR_BCNT_Pos);
		if (cmd & SDMMC_CMD_SDIO_BLOCK) {
			cmdr |= HSMCI_CMDR_TRTYP_BLOCK;
		} else if (cmd & SDMMC_CMD_STREAM) {
			cmdr |= HSMCI_CMDR_TRTYP_STREAM;
		} else if (cmd & SDMMC_CMD_SINGLE_BLOCK) {
			cmdr |= HSMCI_CMDR_TRTYP_SINGLE;
		} else if (cmd & SDMMC_CMD_MULTI_BLOCK) {
			cmdr |= HSMCI_CMDR_TRTYP_MULTIPLE;
		}
	}
	
	hsmci_transferPos = 0;			//reset transferpos
	hsmci_blocksize = blocksize;
	hsmci_numBlocks = num_blocks;
	
	HSMCI_sendCommand(cmdr, cmd, arg);
	
}

void HSMCI_readBlocks(void *dest, uint16_t num_blocks)
{
	uint32_t datasize = num_blocks * hsmci_blocksize;
	
	XDMAC -> XDMAC_GD = (XDMAC_GD_DI0 << HSMCI_XDMAC_CH);	//disable DMA channel
	XDMAC -> XDMAC_CHID[HSMCI_XDMAC_CH].XDMAC_CBC = XDMAC_CBC_BLEN(0);
	XDMAC -> XDMAC_CHID[HSMCI_XDMAC_CH].XDMAC_CDS_MSP = 0;
	XDMAC -> XDMAC_CHID[HSMCI_XDMAC_CH].XDMAC_CSUS = 0;
	XDMAC -> XDMAC_CHID[HSMCI_XDMAC_CH].XDMAC_CDUS = 0;
	
	
	if((uint32_t)dest & 3) {
		XDMAC -> XDMAC_CHID[HSMCI_XDMAC_CH].XDMAC_CUBC = datasize;
		HSMCI->HSMCI_MR |= HSMCI_MR_FBYTE;
		XDMAC -> XDMAC_CHID[HSMCI_XDMAC_CH].XDMAC_CC	= XDMAC_CC_TYPE_PER_TRAN
														| XDMAC_CC_MBSIZE_SINGLE
														| XDMAC_CC_DSYNC_PER2MEM
														| XDMAC_CC_CSIZE_CHK_1
														| XDMAC_CC_DWIDTH_BYTE
														| XDMAC_CC_SIF_AHB_IF1
														| XDMAC_CC_DIF_AHB_IF0
														| XDMAC_CC_SAM_FIXED_AM
														| XDMAC_CC_DAM_INCREMENTED_AM
														| XDMAC_CC_PERID(0);
	} else {
		XDMAC -> XDMAC_CHID[HSMCI_XDMAC_CH].XDMAC_CUBC = datasize / 4;
		HSMCI->HSMCI_MR &= ~HSMCI_MR_FBYTE;
		XDMAC -> XDMAC_CHID[HSMCI_XDMAC_CH].XDMAC_CC	= XDMAC_CC_TYPE_PER_TRAN
														| XDMAC_CC_MBSIZE_SINGLE
														| XDMAC_CC_DSYNC_PER2MEM
														| XDMAC_CC_CSIZE_CHK_1
														| XDMAC_CC_DWIDTH_WORD	
														| XDMAC_CC_SIF_AHB_IF1
														| XDMAC_CC_DIF_AHB_IF0
														| XDMAC_CC_SAM_FIXED_AM
														| XDMAC_CC_DAM_INCREMENTED_AM
														| XDMAC_CC_PERID(0);
	}
		
	XDMAC -> XDMAC_CHID[HSMCI_XDMAC_CH].XDMAC_CSA = (uint32_t)&(HSMCI->HSMCI_FIFO[0]);
	XDMAC -> XDMAC_CHID[HSMCI_XDMAC_CH].XDMAC_CDA = (uint32_t)dest;	
	
	XDMAC -> XDMAC_CHID[HSMCI_XDMAC_CH].XDMAC_CIS;	//read interrupt reg to clear any flags prior to enabling channel
	XDMAC -> XDMAC_GE = ( XDMAC_GE_EN0 << HSMCI_XDMAC_CH );

	//block written increment counter
	hsmci_transferPos += datasize;		
		
}			

void HSMCI_writeBlocks(const void *src, uint16_t num_blocks)
{
	uint32_t datasize = num_blocks * hsmci_blocksize;
	
	XDMAC -> XDMAC_GD = (XDMAC_GD_DI0 << HSMCI_XDMAC_CH);	//disable DMA channel
	XDMAC -> XDMAC_CHID[HSMCI_XDMAC_CH].XDMAC_CBC = XDMAC_CBC_BLEN(0);
	XDMAC -> XDMAC_CHID[HSMCI_XDMAC_CH].XDMAC_CDS_MSP = 0;
	XDMAC -> XDMAC_CHID[HSMCI_XDMAC_CH].XDMAC_CSUS = 0;
	XDMAC -> XDMAC_CHID[HSMCI_XDMAC_CH].XDMAC_CDUS = 0;
	
	
	if((uint32_t)src & 3) {
		XDMAC -> XDMAC_CHID[HSMCI_XDMAC_CH].XDMAC_CUBC = datasize;
		HSMCI->HSMCI_MR |= HSMCI_MR_FBYTE;
		XDMAC -> XDMAC_CHID[HSMCI_XDMAC_CH].XDMAC_CC	= XDMAC_CC_TYPE_PER_TRAN
														| XDMAC_CC_MBSIZE_SINGLE
														| XDMAC_CC_DSYNC_MEM2PER
														| XDMAC_CC_CSIZE_CHK_1
														| XDMAC_CC_DWIDTH_BYTE
														| XDMAC_CC_SIF_AHB_IF0
														| XDMAC_CC_DIF_AHB_IF1
														| XDMAC_CC_SAM_INCREMENTED_AM
														| XDMAC_CC_DAM_FIXED_AM
														| XDMAC_CC_PERID(0);
		} else {
		XDMAC -> XDMAC_CHID[HSMCI_XDMAC_CH].XDMAC_CUBC = datasize / 4;
		HSMCI->HSMCI_MR &= ~HSMCI_MR_FBYTE;
		XDMAC -> XDMAC_CHID[HSMCI_XDMAC_CH].XDMAC_CC	= XDMAC_CC_TYPE_PER_TRAN
														| XDMAC_CC_MBSIZE_SINGLE
														| XDMAC_CC_DSYNC_MEM2PER
														| XDMAC_CC_CSIZE_CHK_1
														| XDMAC_CC_DWIDTH_WORD
														| XDMAC_CC_SIF_AHB_IF0
														| XDMAC_CC_DIF_AHB_IF1
														| XDMAC_CC_SAM_INCREMENTED_AM
														| XDMAC_CC_DAM_FIXED_AM
														| XDMAC_CC_PERID(0);
	}
	
	XDMAC -> XDMAC_CHID[HSMCI_XDMAC_CH].XDMAC_CSA = (uint32_t)src;	
	XDMAC -> XDMAC_CHID[HSMCI_XDMAC_CH].XDMAC_CDA = (uint32_t)&(HSMCI->HSMCI_FIFO[0]);
	
	XDMAC -> XDMAC_CHID[HSMCI_XDMAC_CH].XDMAC_CIS;	//read interrupt reg to clear any flags prior to enabling channel
	XDMAC -> XDMAC_GE = ( XDMAC_GE_EN0 << HSMCI_XDMAC_CH );
	
	//block written increment counter
	hsmci_transferPos += datasize;
	
}

uint8_t HSMCI_SD_WaitReady(void)
{
	uint32_t retry = 200000;

	//wait for data ready status
	do {
		HSMCI_sendCommand(0, SDMMC_MCI_CMD13_SEND_STATUS, (uint32_t)sd_card_info.relative_addr << 16);
		if (HSMCI_getResponse() & CARD_STATUS_READY_FOR_DATA) {
			break;
		}
		
		if (retry-- == 0) {
			return 0;
		}
	} while (1);	
	
	return 1;
}

uint8_t HSMCI_RW_Test(void)
{
	uint32_t num_blocks_to_write = 4;
	char blockwrite[HSMCI_SD_BLOCKSZ * num_blocks_to_write] , blockread[HSMCI_SD_BLOCKSZ * num_blocks_to_write];
	
	for (uint32_t i = 0; i < HSMCI_SD_BLOCKSZ * num_blocks_to_write; i++) {
		blockwrite[i] = 0;
		blockread[i] = 0;
	}
	
	sprintf(blockwrite, "Hello world!!!\r\n");
	sprintf(blockwrite + HSMCI_SD_BLOCKSZ, "joepraktischhoi\r\n");
	sprintf(blockwrite + 2*HSMCI_SD_BLOCKSZ, "Heel praktisch\r\n");
	sprintf(blockwrite + 3*HSMCI_SD_BLOCKSZ, "meautorbeaut\r\n");
	
	SD_writeBlocks(blockwrite, num_blocks_to_write, (312*512));
	SD_readBlocks(blockread, num_blocks_to_write, (312*512));
	
	for (uint32_t i = 0; i < HSMCI_SD_BLOCKSZ * num_blocks_to_write; i++) {
		printf("%c", blockread[i]);
	}
	
	return 1;
	
}


DRESULT SD_readBlocks(void *dst, uint16_t num_blocks, uint32_t startaddr)
{
	uint32_t cmd, arg;
	uint32_t retry = 200000;

	//wait for data ready
	do {
		HSMCI_sendCommand(0, SDMMC_MCI_CMD13_SEND_STATUS, (uint32_t)sd_card_info.relative_addr << 16);
		if (HSMCI_getResponse() & CARD_STATUS_READY_FOR_DATA) {
			break;
		}
		
		if (retry-- == 0) {
			return RES_ERROR;
		}
	} while (1);
	
	if (num_blocks > 1) {
		cmd = SDMMC_CMD18_READ_MULTIPLE_BLOCK;
	} else {
		cmd = SDMMC_CMD17_READ_SINGLE_BLOCK;
	}
	
	if (sd_card_info.type & CARD_TYPE_HC) {
		arg = startaddr;
	} else {
		arg = (startaddr * HSMCI_SD_BLOCKSZ);
	}	

	HSMCI_adtc_start(cmd, arg, HSMCI_SD_BLOCKSZ, num_blocks, 1);
	
	//check for error code?
	
	HSMCI_readBlocks(dst, num_blocks);		
	HSMCI_WaitEndBlockTransfer();

	HSMCI_sendCommand(0, SDMMC_CMD12_STOP_TRANSMISSION, 0);
	
	return RES_OK;
}

DRESULT SD_writeBlocks(const void *src, uint16_t num_blocks, uint32_t startaddr)
{
	uint32_t cmd, arg;

	//wait for data ready status
	if (!HSMCI_SD_WaitReady()) {
		return RES_ERROR;
	}
		
	
	if (num_blocks > 1) {
		cmd = SDMMC_CMD25_WRITE_MULTIPLE_BLOCK;
	} else {
		cmd = SDMMC_CMD24_WRITE_BLOCK;
	}	
	
	if (sd_card_info.type & CARD_TYPE_HC) {
		arg = startaddr;
	} else {
		arg = (startaddr * HSMCI_SD_BLOCKSZ);
	}	
	
	HSMCI_adtc_start(cmd, arg, HSMCI_SD_BLOCKSZ, num_blocks, 1);		//address counter is reset here
	
	//check for error code?

	HSMCI_writeBlocks(src, num_blocks);
	HSMCI_WaitEndBlockTransfer();
	
	HSMCI_sendCommand(0, SDMMC_CMD12_STOP_TRANSMISSION, 0);


	return RES_OK;	
}

DRESULT SD_getStatus(void)
{
	return Stat;
}

DWORD get_fattime (void)
{
	uint32_t timestamp	=	((1998 - 1980) << 25)	//year
						|	(2 << 21)				//month
						|	(16 << 16);				//day		
	
	return timestamp;
}

DRESULT SD_IOctl(uint8_t cmd, void *buff)
{
	DRESULT res;
	if (Stat & STA_NOINIT) return RES_NOTRDY;	//no point if card isn't ready
	
	res = RES_ERROR;
	
	switch(cmd) {
		
		case CTRL_SYNC:		//make sure all data has been written on the media
			if (HSMCI_SD_WaitReady()) {
				res = RES_OK;
			}
			break;
		}
		
	return res;
}