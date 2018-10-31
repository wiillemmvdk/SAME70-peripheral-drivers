/*
 * XDMAC.c
 *
 * Created: 17-Oct-18 13:05:14
 *  Author: Willem van der Kooij
 */ 

#include "sam.h"
#include "XDMAC.h"

/* bascicly the same as the prepare_dma_channel() in ASF */
void DMA_prepareChannel(uint32_t channel)
{
	if ((PMC -> PMC_PCSR1 & PMC_PCSR1_PID58) != PMC_PCSR1_PID58) {	//clock is not enabled
		PMC -> PMC_PCER1 |= PMC_PCER1_PID58;						//enable XDMAC peripheral clock
	}

	/* Clear status */
	XDMAC -> XDMAC_GS;							//read dummy channel status
	XDMAC -> XDMAC_GIS;
	XDMAC -> XDMAC_CHID[channel].XDMAC_CIS;		//get interrupt status
	
	/* disable interrupts for channel */
	XDMAC -> XDMAC_GID = (XDMAC_GID_ID(channel));		//disable global interrupts
	XDMAC -> XDMAC_CHID[channel].XDMAC_CID = 0x7F;		//disable all interrupts for channel
	
	/* disable channel */
	XDMAC -> XDMAC_GD = XDMAC_GD_DI(channel);						//disable channel
	XDMAC -> XDMAC_CHID[channel].XDMAC_CSA	= 0;					//set source addr
	XDMAC -> XDMAC_CHID[channel].XDMAC_CDA	= 0;					//set dst addr
	XDMAC -> XDMAC_CHID[channel].XDMAC_CBC	= XDMAC_CBC_BLEN(0);	//block control
	XDMAC -> XDMAC_CHID[channel].XDMAC_CC	= 0;					//ASF writes 0x20 here but thats an empty bit ????		//set config
	XDMAC -> XDMAC_CHID[channel].XDMAC_CNDA = 0;					//descriptor addr
	XDMAC -> XDMAC_CHID[channel].XDMAC_CNDC = 0;					//descriptor ctrl
}


void DMA_configChannel_I2SC1(uint32_t channel, uint32_t PERID, uint32_t datadir, uint32_t *srcbuf, uint32_t *dstbuf, uint32_t bufsize)
{
	DMA_prepareChannel(channel);	//prepare channel for config
	
	XDMAC -> XDMAC_CHID[channel].XDMAC_CIS;			//clear pending interrupt status bit
	
	XDMAC -> XDMAC_CHID[channel].XDMAC_CSA	= (datadir ? (uint32_t)&I2SC1->I2SC_RHR	: (uint32_t)srcbuf);			//set source addr
	XDMAC -> XDMAC_CHID[channel].XDMAC_CDA	= (datadir ? (uint32_t)dstbuf			: (uint32_t)&I2SC1->I2SC_THR);	//set dst addr
	
	XDMAC -> XDMAC_CHID[channel].XDMAC_CUBC = XDMAC_CUBC_UBLEN(bufsize);	//uBlock Size
	
	XDMAC -> XDMAC_CHID[channel].XDMAC_CC	= XDMAC_CC_TYPE_PER_TRAN
											| XDMAC_CC_MBSIZE_SINGLE
											| (datadir ? XDMAC_CC_SAM_FIXED_AM		 : XDMAC_CC_SAM_INCREMENTED_AM)
											| (datadir ? XDMAC_CC_DAM_INCREMENTED_AM : XDMAC_CC_DAM_FIXED_AM)
											| (datadir ? XDMAC_CC_DSYNC_PER2MEM		 : XDMAC_CC_DSYNC_MEM2PER)
											| XDMAC_CC_CSIZE_CHK_1
											| XDMAC_CC_DWIDTH_WORD
											| (datadir ? XDMAC_CC_SIF_AHB_IF1		: XDMAC_CC_SIF_AHB_IF0)
											| (datadir ? XDMAC_CC_DIF_AHB_IF0		: XDMAC_CC_DIF_AHB_IF1)
											| XDMAC_CC_PERID(PERID)
											| XDMAC_CC_SWREQ_HWR_CONNECTED;
								
	XDMAC -> XDMAC_CHID[channel].XDMAC_CBC	= XDMAC_CBC_BLEN(0); 
	
	XDMAC -> XDMAC_CHID[channel].XDMAC_CNDC		= 0;
	XDMAC -> XDMAC_CHID[channel].XDMAC_CDS_MSP	= 0;
	XDMAC -> XDMAC_CHID[channel].XDMAC_CSUS		= 0;
	XDMAC -> XDMAC_CHID[channel].XDMAC_CDUS		= 0;
	
}

/* 
 * The descriptors point to each other for pingpong buffering
 *
 * buffer 1 for ping setup, buffer 2 for pong?
 *
 */
void DMA_LinkedListSetupI2SC1(uint32_t channel, uint32_t datadir, lld_view1 *lld, uint32_t *srcbuf1, uint32_t *srcbuf2, uint32_t *dstbuf1 ,uint32_t *dstbuf2, uint32_t bufsize)
{
	
	lld[0].mbr_nda	= (uint32_t)&lld[1];
	lld[0].mbr_da	= (datadir ? (uint32_t)&I2SC1->I2SC_RHR : (uint32_t)srcbuf1);
	lld[0].mbr_da	= (datadir ? (uint32_t)dstbuf1			: (uint32_t)&I2SC1->I2SC_THR);
	lld[0].mbr_ubc	= XDMAC_UBC_NVIEW_NDV1
					| XDMAC_UBC_NDE_FETCH_EN
					| (datadir ? XDMAC_UBC_NSEN_UNCHANGED	: XDMAC_UBC_NSEN_UPDATED)
					| (datadir ? XDMAC_UBC_NDEN_UPDATED		: XDMAC_UBC_NDEN_UNCHANGED)
					| XDMAC_UBC_UBLEN(bufsize);
	
	
	lld[1].mbr_nda	= (uint32_t)&lld[0];
	lld[1].mbr_sa	= (datadir ? (uint32_t)&I2SC1->I2SC_RHR : (uint32_t)srcbuf2);
	lld[1].mbr_da	= (datadir ? (uint32_t)dstbuf2 			: (uint32_t)&I2SC1->I2SC_THR);
	lld[1].mbr_ubc	= XDMAC_UBC_NVIEW_NDV1
					| XDMAC_UBC_NDE_FETCH_EN
					| (datadir ? XDMAC_UBC_NSEN_UNCHANGED	: XDMAC_UBC_NSEN_UPDATED)
					| (datadir ? XDMAC_UBC_NDEN_UPDATED		: XDMAC_UBC_NDEN_UNCHANGED)
					| XDMAC_UBC_UBLEN(bufsize);
								
	/* Write next descriptor addr, since the linked listt should be wordt aligned the last bits aren't used
	 * The macro atmel uses (XDMAC_CNDA_NDA) cant be used since here the upper 2 bits will be shifted of as well
	 */
	XDMAC -> XDMAC_CHID[channel].XDMAC_CNDA	= ((uint32_t)&lld[0] & 0xFFFFFFFC);

	XDMAC -> XDMAC_CHID[channel].XDMAC_CNDC	= XDMAC_CNDC_NDVIEW_NDV1
											| XDMAC_CNDC_NDE_DSCR_FETCH_EN
											| (datadir ? XDMAC_CNDC_NDSUP_SRC_PARAMS_UNCHANGED	: XDMAC_CNDC_NDSUP_SRC_PARAMS_UPDATED) 
											| (datadir ? XDMAC_CNDC_NDDUP_DST_PARAMS_UPDATED	: XDMAC_CNDC_NDDUP_DST_PARAMS_UNCHANGED );
	
}


void DMA_enableInterrupt(uint32_t channel, uint32_t interruptMask)
{
	//clear pending interrupts
	XDMAC -> XDMAC_CHID[channel].XDMAC_CIS;
	XDMAC -> XDMAC_GIS;
	
	//channel specific interrupt
	XDMAC -> XDMAC_CHID[channel].XDMAC_CIE = interruptMask;	
	
	//global interrupt enable for channel
	XDMAC -> XDMAC_GIE = XDMAC_GIE_IE(channel);
}

void DMA_enableChannel(uint32_t channel)
{
	XDMAC -> XDMAC_GE = XDMAC_GE_EN(channel);	//enable channel
}