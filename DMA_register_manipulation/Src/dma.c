/*
 * dma.c
 *  Created on: Dec 18, 2025
 *      Author: Tushar
 */

#include "dma.h"

#define DMA2EN              (1U<<22)
#define DMA1EN (1<<21)
#define ALL_Stream_bits (0<31)
#define CH_SEL (0<<0) //Since we are selecting channel 0(ADC1)
#define NDTR_VAL      25000
#define PL_16 (1<<16)
#define PL_17 (1<<17)
#define P2M ((0U<<7) | (0U<<6)) //PERIPHERAL TO MEMORY
#define M2P ((0U<<7) | (1U<<6)) //Memory to peripheral
#define M2M ((1U<<7) | (0U<<6)) //memory to memory
#define PINC (1U<<9) //9th Bit PINC- peripheral increment mode bit, if set to 1 increments
#define MINC (1U<<10) //10th bit MINC - memory increment mode bit
#define CIRC_MODE (1U<<8)
#define ENABLE_DMA_BIT (1U<<0)
#define FLOW_CTRL (1<<5) //0:DMA is the flow controller 1: peripheral is the FC

uint32_t raw_adc_value;
void dma1_init(){
	 RCC->AHB1ENR |= DMA2EN;
	 /*Disable DMA stream*/
	 DMA2_Stream0->CR &=~ENABLE_DMA_BIT;
	 /*Wait till DMA is disabled*/
	 while((DMA2_Stream0->CR & ENABLE_DMA_BIT)){}

	/*Setting the peripheral port register
	the peripheral data register is ADC1_DR*/
	DMA2_Stream0->PAR=(uint32_t)(&(ADC1->DR));
	/*Setting the destination address*/
	DMA2_Stream0->M0AR=(uint32_t)(&raw_adc_value);

	/*Configure the data items to be transferred SxNDTR*/
	DMA2_Stream0->NDTR|=(uint16_t)NDTR_VAL;

	/*5. Selecting the DMA channel request */
	DMA2_Stream0->CR |=CH_SEL;

	/*Setting the stream priority*/
	DMA2_Stream0->CR|=PL_16;
	DMA2_Stream0->CR|=PL_17;

	/*Configuring data transfer direction*/
	DMA2_Stream0->CR|=P2M;

	/* Setting memory data width*/
	DMA2_Stream0->CR|=(1<<13);
	DMA2_Stream0->CR&=~(1<<14); // when you are setting make sure you AND that specific bit

	/* Setting peripheral data width*/
	DMA2_Stream0->CR|=(1<<12);
	DMA2_Stream0->CR&=~(1<<11);

	/*Configuring pointer increment of peripheral and memory*/
	DMA2_Stream0->CR&=~PINC;
	DMA2_Stream0->CR&=~MINC;

	/*Setting the flow control*/
	DMA2_Stream0->CR&=FLOW_CTRL;

	/*Enable circular mode*/
	DMA2_Stream0->CR|=CIRC_MODE;

	DMA2->LISR |= ALL_Stream_bits;
	DMA2->HISR |= ALL_Stream_bits;
    /*Enable DMA stream*/
	DMA2_Stream0->CR|=ENABLE_DMA_BIT;
}
