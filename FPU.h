/*
 * FPU.h
 *
 * Created: 08-Oct-18 20:00:16
 *  Author: Willem van der Kooij
 */ 

// http://ww1.microchip.com/downloads/en/AppNotes/Atmel-42144-SAM4E-FPU-and-CMSIS-DSP-Library_AP-Note_AT03157.pdf
// http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0646a/BEHBJHIG.html

#ifndef FPU_H_
#define FPU_H_


/** Address for ARM CPACR */
#define ADDR_CPACR 0xE000ED88
/** CPACR Register */
#define REG_CPACR (*((volatile uint32_t *)ADDR_CPACR))


void fpu_enable(void)
{
 REG_CPACR |= (0xFu << 20);
 __DSB();
 __ISB();
}

uint32_t fpu_is_enabled(void)
{
	return (REG_CPACR & (0xFu << 20)); 
}


#endif /* FPU_H_ */