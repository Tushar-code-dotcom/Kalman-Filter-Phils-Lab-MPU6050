/*
 * adc.c
 *
 *  Created on: Dec 14, 2025
 *      Author: Tushar
 */
#include "adc.h"
#define GPIOAEN        (1U<<0)
#define ADC1EN         (1U<<8)
#define ADC_CH1        (1U<<0)
#define ADC_SEQ_LEN_1   0x00
#define CR1_SCAN    (1U<<8)

#define CR2_ADCON      (1U<<0)
#define CR2_CONT       (1U<<1)
#define CR2_SWSTART    (1U<<30)
#define SR_EOC         (1U<<1)
#define ADC_DMA_EN     (1U<<8)
#define ADC_DMA_DDS    (1U<<9)
#define ADC_OVR        (1U<<5)
void pa1_adc_init(void)
{
    /****Configure the ADC GPIO Pin**/
    /*Enable clock access to GPIOA*/
    RCC->AHB1ENR |= GPIOAEN;
    /*Set PA1 mode to analog mode*/
    GPIOA->MODER |=(1U<<2);
    GPIOA->MODER |=(1U<<3);
    /****Configure the ADC Module**/
    /*Enable clock access to the ADC module*/
    RCC->APB2ENR |=ADC1EN;
    /*Set sequence length*/
        ADC1->SQR1 &= ~(1U<<20);
        ADC1->SQR1 &= ~(1U<<21);
        ADC1->SQR1 &= ~(1U<<22);
        ADC1->SQR1 &= ~(1U<<23);
    /*Set conversion sequence start*/
    ADC1->SQR3 = ADC_CH1;
    /*Enable scan mode*/
    ADC1->CR1 = CR1_SCAN;
    /*Setting the DMA and DDS bit in ADC_CR2 */
    ADC1->CR2|=ADC_DMA_EN;
    ADC1->CR2|=ADC_DMA_DDS;
    /*Set conversion sequence length*/
    ADC1->SQR1 = ADC_SEQ_LEN_1;
    /*Enable ADC module*/
    ADC1->CR2 |=CR2_ADCON;
}
void start_conversion(void)
{
    /*Enable continuous conversion*/
    ADC1->CR2 |=CR2_CONT;
    /*Start ADC conversion*/
    ADC1->CR2 |=CR2_SWSTART;
}

uint32_t adc_read(void)
{
	/*If OVR bit is set, we try resetting the bit*/
	if((ADC1->SR & ADC_OVR)){
		ADC1->SR&=~(ADC_OVR);
	}
	/*Wait for conversion to be complete*/
	while(!(ADC1->SR & SR_EOC)){}

	/*Read converted value*/
	return (ADC1->DR);
}




