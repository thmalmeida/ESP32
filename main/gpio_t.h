/*
 * gpio.h
 *
 *  Created on: 22 de fev de 2017
 *      Author: titi
 */

#ifndef HARDWARE_GPIO_H_
#define HARDWARE_GPIO_H_

#include <stdio.h>
//#include <stdint.h>	// is that for uint8_t
//#include "sdmmc_types.h"
//#include "stdint-gcc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include <stdint.h>
#include "sdkconfig.h"

#define BLINK_GPIO_1 2
#define BLINK_GPIO_2 3

class thmGPIO {
public:
	void gateConfig(uint8_t pin, uint8_t dir);
	void gateSet(uint8_t pin, uint8_t status);
	void gateToggle(uint8_t pin);
	uint8_t gateRead(uint8_t pin, uint8_t reg);
};


void thmGPIO::gateConfig(uint8_t pin, uint8_t dir)
{
//	gpio_pad_select_gpio(pin);

	switch (dir)
	{
		case 0:
			gpio_set_direction((gpio_num_t)BLINK_GPIO_1, GPIO_MODE_INPUT);
			break;

		case 1:
			gpio_set_direction((gpio_num_t)BLINK_GPIO_1, GPIO_MODE_OUTPUT);
			break;

		default:
			break;
	}
//
//		case 29:
//			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
//			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
//			GPIO_Init(GPIOB, &GPIO_InitStructure);
//			break;
}
void thmGPIO::gateSet(uint8_t pin, uint8_t status)
{
	gpio_set_level((gpio_num_t)pin, (uint32_t) status);
}
//void GPIO::gateToggle(uint8_t pin)
//{
//	switch (pin)
//	{
//		case 1:
//			GPIOC -> ODR ^= (1<<13);
//			break;
//
//		case 2:
//			GPIOC -> ODR ^= (1<<14);
//			break;
//
//		case 3:
//			GPIOC -> ODR ^= (1<<15);
//			break;
//
//		case 4:
//			GPIOA -> ODR ^= (1<<0);
//			break;
//
//		case 5:
//			GPIOA -> ODR ^= (1<<1);
//			break;
//
//		case 6:
//			GPIOA -> ODR ^= (1<<2);
//			break;
//
//		case 7:
//			GPIOA -> ODR ^= (1<<3);
//			break;
//
//		case 8:
//			GPIOA -> ODR ^= (1<<4);
//			break;
//
//		case 9:
//			GPIOA -> ODR ^= (1<<5);
//			break;
//
//		case 10:
//			GPIOA -> ODR ^= (1<<6);
//			break;
//
//		case 11:
//			GPIOA -> ODR ^= (1<<7);
//			break;
//
//		case 30:
//			GPIOB -> ODR ^= (1<<7);
//			break;
//	}
//
//}
uint8_t thmGPIO::gateRead(uint8_t pin, uint8_t reg)	// reg: read register input IDR (0) or output ODR (1)
{
	return 0;
}
//	uint8_t status = 0;
//
//	switch (pin)
//	{
//		case 1:
//			if(reg)
//				status = GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13);
//			else
//				status = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13);
//
////			status =  (uint8_t) (((GPIOC -> ODR) & (1 << 13)) >> 13);
//			break;
//
//		case 2:
//			if(reg)
//				status = GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_14);
//			else
//				status = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14);
//			break;
//
//		case 3:
//			if(reg)
//				status = GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_15);
//			else
//				status = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15);
//			break;
//
//		case 4:
//			if(reg)
//				status = GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_0);
//			else
//				status = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);
//			break;
//
//		case 5:
//			if(reg)
//				status = GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_1);
//			else
//				status = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1);
//			break;
//
//		case 6:
//			if(reg)
//				status = GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_2);
//			else
//				status = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2);
//			break;
//
//		case 7:
//			if(reg)
//				status = GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_3);
//			else
//				status = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3);
//			break;
//
//		case 16:
//			if(reg)
//				status = GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_12);
//			else
//				status = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12);
//			break;
//
//		case 17:
//			if(reg)
//				status = GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_13);
//			else
//				status = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13);
//			break;
//
//		case 18:
//			if(reg)
//				status = GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_14);
//			else
//				status = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14);
//			break;
//
//		case 19:
//			if(reg)
//				status = GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_15);
//			else
//				status = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15);
//			break;
//
//		case 24:
//			if(reg)
//				status = GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_12);
//			else
//				status = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12);
//			break;
//
//		case 25:
//			if(reg)
//				status = GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_15);
//			else
//				status = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15);
//			break;
//
//		case 26:
//			if(reg)
//				status = GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_3);
//			else
//				status = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3);
//			break;
//
//		case 27:
//			if(reg)
//				status = GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_4);
//			else
//				status = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4);
//			break;
//
//		case 28:
//			if(reg)
//				status = GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_5);
//			else
//				status = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5);
//			break;
//
//		case 29:
//			if(reg)
//				status = GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_6);
//			else
//				status = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6);
//			break;
//
//		case 30:
//			if(reg)
//				status = GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_7);
//			else
//				status = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7);
//			break;
//
//		case 31:
//			if(reg)
//				status = GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_8);
//			else
//				status = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8);
//			break;
//
//		case 32:
//			if(reg)
//				status = GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_9);
//			else
//				status = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9);
//			break;
//	}
//
//	return status;
//}
//
#endif /* HARDWARE_GPIO_H_ */
