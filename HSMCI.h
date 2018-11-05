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

uint8_t HSMCI_RW_Test(void);


#endif /* HSMCI_H_ */