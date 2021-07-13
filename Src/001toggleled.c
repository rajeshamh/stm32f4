/*
 * 001toggleled.c
 *
 *  Created on: Jun 11, 2021
 *      Author: rajes
 */


#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

void delay(int data)
{   int i;
	for(i=0;i<4000*data;i++);
}


int main()
{

	GPIO_Handle_t GPIOled;
	GPIOled.pGPIOx=GPIOD;
    GPIOled.GPIO_PinConfig.GPIO_Pinnumber= GPIO_PIN_NO_12|GPIO_PIN_NO_13|GPIO_PIN_NO_14|GPIO_PIN_NO_15;
    GPIOled.GPIO_PinConfig.GPIO_Pinmode= GPIO_MODE_OUT;
    GPIOled.GPIO_PinConfig.GPIO_Pin_Outputtype=GPIO_OP_TYPE_PP;
    GPIOled.GPIO_PinConfig.GPIO_PuPdControl=GPIO_NO_PUPD;
    GPIOled.GPIO_PinConfig.GPIO_Pinspeed=GPIO_SPEED_FAST;
    GPIO_PeriClockControl(GPIOD,ENABLE);
    GPIO_Init(&GPIOled);



while(1)
{
	GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_NO_12|GPIO_PIN_NO_13|GPIO_PIN_NO_14|GPIO_PIN_NO_15);
	delay(500);
}

}
























