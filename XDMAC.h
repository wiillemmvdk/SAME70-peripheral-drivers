/*
 * XDMAC.h
 *
 * Created: 17-Oct-18 13:06:39
 *  Author: Willem van der Kooij
 */ 


#ifndef XDMAC_H_
#define XDMAC_H_

/* USED CHANNELS
 * 0 I2S
 * 1 I2S
 * 2 I2S
 * 3 I2S
 * 4 HSMCI
*/
#define I2SC1_XDMAC_CH_L_RX
#define I2SC1_XDMAC_CH_L_TX
#define I2SC1_XDMAC_CH_R_RX
#define I2SC1_XDMAC_CH_R_TX
#define	HSMCI_XDMAC_CH			4


#define XDMAC_PER_DATADIR_TX	0
#define XDMAC_PER_DATADIR_RX	1


#define	XDMAC_I2SC1_PERID_L_TX 46
#define	XDMAC_I2SC1_PERID_L_RX 47
#define	XDMAC_I2SC1_PERID_R_TX 50
#define	XDMAC_I2SC1_PERID_R_RX 51

#define COMPILER_WORD_ALIGNED    __attribute__((__aligned__(4)))


/* Linked List descriptor view 1 style */
typedef struct 
{
	uint32_t mbr_nda;	// addr of next descriptor
	uint32_t mbr_ubc;	// control settings of MBR_UBC
	uint32_t mbr_sa;	// //next source addr
	uint32_t mbr_da;	// next destination addr
} lld_view1;


void DMA_prepareChannel(uint32_t channel);
void DMA_configChannel_I2SC1(uint32_t channel, uint32_t PERID, uint32_t datadir, uint32_t *srcbuf, uint32_t *dstbuf, uint32_t bufsize);
void DMA_LinkedListSetupI2SC1(uint32_t channel, uint32_t datadir, lld_view1 *lld, uint32_t *srcbuf1, uint32_t *srcbuf2, uint32_t *dstbuf1 ,uint32_t *dstbuf2, uint32_t bufsize);

void DMA_enableInterrupt(uint32_t channel, uint32_t interruptMask);
void DMA_enableInterrupt(uint32_t channel, uint32_t interruptMask);
void DMA_enableChannel(uint32_t channel);



/* XDMA_MBR_UBC */
#define   XDMAC_UBC_NDE            (0x1u << 24)
#define   XDMAC_UBC_NDE_FETCH_DIS  (0x0u << 24)
#define   XDMAC_UBC_NDE_FETCH_EN   (0x1u << 24)
#define   XDMAC_UBC_NSEN           (0x1u << 25)
#define   XDMAC_UBC_NSEN_UNCHANGED (0x0u << 25)
#define   XDMAC_UBC_NSEN_UPDATED   (0x1u << 25)
#define   XDMAC_UBC_NDEN           (0x1u << 26)
#define   XDMAC_UBC_NDEN_UNCHANGED (0x0u << 26)
#define   XDMAC_UBC_NDEN_UPDATED   (0x1u << 26)
#define   XDMAC_UBC_NVIEW_Pos      27
#define   XDMAC_UBC_NVIEW_Msk      (0x3u << XDMAC_UBC_NVIEW_Pos)
#define   XDMAC_UBC_NVIEW_NDV0     (0x0u << XDMAC_UBC_NVIEW_Pos)
#define   XDMAC_UBC_NVIEW_NDV1     (0x1u << XDMAC_UBC_NVIEW_Pos)
#define   XDMAC_UBC_NVIEW_NDV2     (0x2u << XDMAC_UBC_NVIEW_Pos)
#define   XDMAC_UBC_NVIEW_NDV3     (0x3u << XDMAC_UBC_NVIEW_Pos)
#define   XDMAC_UBC_UBLEN_Pos 0
#define   XDMAC_UBC_UBLEN_Msk (0xffffffu << XDMAC_UBC_UBLEN_Pos)
#define   XDMAC_UBC_UBLEN(value) ((XDMAC_UBC_UBLEN_Msk & ((value) << XDMAC_UBC_UBLEN_Pos)))



#endif /* XDMAC_H_ */