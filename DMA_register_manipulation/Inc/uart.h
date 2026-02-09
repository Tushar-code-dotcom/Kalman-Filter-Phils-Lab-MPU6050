/*
 * uart.h
 *
 *  Created on: Dec 19, 2025
 *      Author: Tushar
 */

#ifndef UART_H_
#define UART_H_

#include "stm32f4xx.h"
#include <stdint.h>

void uart_init();
static void uart_write(int ch);
static uint16_t compute_uart_bd(uint32_t periph_clk,uint32_t baudrate);
static void uart_set_baudrate(uint32_t periph_clk,uint32_t baudrate);

#endif /* UART_H_ */
