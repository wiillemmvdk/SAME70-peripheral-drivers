/*
 * PMC.c
 *
 * Created: 12-Oct-18 15:43:05
 *  Author: Willem van der Kooij
 */ 

#include "sam.h"
#include "PMC.h"

#define F_CPU			300000000
#define F_XOSC			12000000

#define XPLD_BOARD


void InitPMC(void)
{
	PMC -> PMC_WPMR &= ~(PMC_WPMR_WPEN);
	PMC -> PMC_WPMR = PMC_WPMR_WPKEY_PASSWD;
	
	MATRIX -> MATRIX_WPMR &= ~(MATRIX_WPMR_WPEN);
	MATRIX -> MATRIX_WPMR = MATRIX_WPMR_WPKEY_PASSWD;
}





#define PMC_TIMEOUT		2048

/* Clock setup 
 *	source external 13MHz OSC input
 *
 * Clocks explained (see page 283 of datasheet)
 *		Main Clock			= Input clock, can be used as input for the PLL
 *		Master Clock		= Output clock for system, either derived from slow clock, main clock or PLL clock 
 *		Programmable clock	= PCK[x] clocks used for some peripherals can have multiple input clocks and use its own prescaler
 *		Peripheral clock	= derived from master clock
 *
 *
 *		PLLA output is 300MHz
 *		Master clock = PLLA / 2
 *
 */
uint8_t InitClock(void)
{
	//set flash wait state
	EFC->EEFC_FMR	=	EEFC_FMR_FWS(5)
					|	EEFC_FMR_CLOE;
	
#ifdef XPLD_BOARD		//enable PLL0 source

	PMC -> CKGR_MOR =	(PMC->CKGR_MOR & ~CKGR_MOR_MOSCXTBY)
					|	CKGR_MOR_KEY_PASSWD
					|	CKGR_MOR_MOSCXTEN				//enable main XOSC
					|	CKGR_MOR_MOSCXTST(0x3E);		//main OSC startup time multiplied by 8 in clock cycles
						
	
	//wait for XOSC to stabilize
	while (!(PMC->PMC_SR & PMC_SR_MOSCXTS));
		
	PMC -> CKGR_MOR |=	CKGR_MOR_KEY_PASSWD
					|	CKGR_MOR_MOSCSEL;				//select main crystal OSC
	
	//wait for OSC to be ready
	while(!(PMC->PMC_SR & PMC_SR_MOSCSELS));
	
	//configure and enable PLL
	PMC -> CKGR_PLLAR = CKGR_PLLAR_ONE | CKGR_PLLAR_MULA(0);	//always stop PLL clock first
	PMC -> CKGR_PLLAR = CKGR_PLLAR_ONE | CKGR_PLLAR_MULA(((F_CPU / F_XOSC) - 1)) | CKGR_PLLAR_DIVA_BYPASS | CKGR_PLLAR_PLLACOUNT(0x3FU); //setup multiplier (F_CPU / F_XOSC) and DIV (bypass = 1)
	while (! (PMC -> PMC_SR & PMC_SR_LOCKA) );	//wait for PLL lock		
	
	//set MCK div
	PMC -> PMC_MCKR = (PMC -> PMC_MCKR & (~PMC_MCKR_MDIV_Msk)) | PMC_MCKR_MDIV_PCK_DIV2;
	while (!(PMC->PMC_SR & PMC_SR_MCKRDY));
	
	//set MCK to PLL
	uint32_t ul_timeout;

	PMC->PMC_MCKR = (PMC -> PMC_MCKR & (~PMC_MCKR_PRES_Msk));
	for (ul_timeout = PMC_TIMEOUT; !(PMC->PMC_SR & PMC_SR_MCKRDY); --ul_timeout) {
		if (ul_timeout == 0) {
			return 0;
		}
	}

	PMC->PMC_MCKR	=	(PMC->PMC_MCKR & (~PMC_MCKR_CSS_Msk)) |	PMC_MCKR_CSS_PLLA_CLK;
	
	for (ul_timeout = PMC_TIMEOUT; !(PMC->PMC_SR & PMC_SR_MCKRDY); --ul_timeout) {
		if (ul_timeout == 0) {
			return 0;
		}
	}
#endif
	
	return 1;
}