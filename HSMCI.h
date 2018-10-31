/*
 * HSMCI.h
 *
 * Created: 20-Oct-18 22:50:32
 *  Author: Willem van der Kooij
 */ 


#ifndef HSMCI_H_
#define HSMCI_H_

#define HSMCI_DMA_PERID	0
#define HSMCI_BLOCKACTION_READ		1
#define HSMCI_BLOCKACTION_WRITE		0

#define HSMCI_SD_BLOCKSZ	512


uint8_t SD_CardInit(void);
void test_sd_mem(uint8_t *writebuffer, uint8_t *readbuffer);
uint8_t SD_writeBlocks(uint32_t *src, uint16_t num_blocks, uint32_t startaddr);
uint8_t SD_readBlocks(uint32_t *dst, uint16_t num_blocks, uint32_t startaddr);

#endif /* HSMCI_H_ */