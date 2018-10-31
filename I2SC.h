/*
 * I2SC.h
 *
 * Created: 12-Oct-18 15:40:33
 *  Author: Willem van der Kooij
 */ 


#ifndef I2SC_H_
#define I2SC_H_


#define I2SC_BUFFSZ 1024


//2D buffers for pingpong
typedef struct 
{
	uint32_t data_out_left[2];
	uint32_t data_in_left[2];
	uint32_t data_out_right[2];
	uint32_t data_in_right[2];
} I2S_data_pingpong;


void I2SC_Init(void);
void I2SC_Start(void);
void I2SC_StartWithDMA(I2S_data_pingpong *buffersetup, uint32_t DMAbuffLen);
uint8_t I2SC_DMA_getPingPong(void);


#endif /* I2SC_H_ */