/*
 * adc.h
 *
 *  Created on: Dec 11, 2025
 *      Author: Tushar
 */

#ifndef ADC_H_
#define ADC_H_

#include "stm32f4xx.h"
#include "stdint.h"

void pa1_adc_init(void);
void start_conversion(void);
uint32_t adc_read(void);

#endif /* ADC_H_ */
