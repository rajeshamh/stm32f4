/*
 * 002buttontoggle.c
 *
 *  Created on: Jun 11, 2021
 *      Author: rajes
 */



#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

void delay(int data)
{   int i;
	for(i=0;i<(data*data*2)/2;i++);
}



void ledinit()
{
	    GPIO_Handle_t GPIOled;
		GPIOled.pGPIOx=GPIOD;
	    GPIOled.GPIO_PinConfig.GPIO_Pinnumber= GPIO_PIN_NO_12;
	    GPIOled.GPIO_PinConfig.GPIO_Pinmode= GPIO_MODE_OUT;
	    GPIOled.GPIO_PinConfig.GPIO_Pin_Outputtype=GPIO_OP_TYPE_PP;
	    GPIOled.GPIO_PinConfig.GPIO_PuPdControl=GPIO_NO_PUPD;
	    GPIOled.GPIO_PinConfig.GPIO_Pinspeed=GPIO_SPEED_FAST;
	    GPIO_PeriClockControl(GPIOD,ENABLE);
	    GPIO_Init(&GPIOled);
}

void toggleonpress()
{

	        GPIO_Handle_t buttonled;
	        buttonled.pGPIOx=GPIOA;
	        buttonled.GPIO_PinConfig.GPIO_Pinnumber= GPIO_PIN_NO_0;
	        buttonled.GPIO_PinConfig.GPIO_Pinmode= GPIO_MODE_IN;
	        buttonled.GPIO_PinConfig.GPIO_PuPdControl=GPIO_NO_PUPD;
	        buttonled.GPIO_PinConfig.GPIO_Pinspeed=GPIO_SPEED_FAST;
		    GPIO_PeriClockControl(GPIOA,ENABLE);
		    GPIO_Init(&buttonled);

}

int main()
{

	ledinit();
	toggleonpress();

while(1)
{
	if(GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0)==SET )
		{
		GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_NO_12);
		delay(500);
		}
}

}





