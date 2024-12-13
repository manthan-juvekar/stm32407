/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>
#include<stdio.h>

#define ADC_BASE_ADDR				0x40012000UL
#define ADC_CR1_REG_OFFSET			0X04UL
#define ADC_CR1_REG_ADDR			(ADC_BASE_ADDR + ADC_CR1_REG_OFFSET)
#define RCC_BASE_ADDR       		0x40023800UL
#define RCC_APB2_ENR_OFFSET         0X44UL
#define RCC_APB2_ENR_ADDR           (RCC_APB2_ENR_OFFSET + RCC_BASE_ADDR )
int main(void)
{
	uint32_t *pAdcCrlReg= (uint32_t*) ADC_CR1_REG_ADDR;

	uint32_t *pApbAdcClkEnReg = (uint32_t*) RCC_APB2_ENR_ADDR;
	 //1.Enble the peripheral clock for ADC1
	*pApbAdcClkEnReg |= (1<<8);
	 //2.modify the ADC crl register
	*pAdcCrlReg |= (1<<8);
    /* Loop forever */
	for(;;);
}