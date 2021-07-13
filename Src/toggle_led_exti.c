/*
 * toggle_led_exti.c
 *
 *  Created on: Jun 12, 2021
 *      Author: rajes
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

void delay(int data)
{   int i;
	for(i=0;i<data*data*2;i++);
}

void led(void)
{


	GPIO_Handle_t GPIOled;
    memset(&GPIOled,0,sizeof(GPIOled));
	GPIOled.pGPIOx=GPIOD;
    GPIOled.GPIO_PinConfig.GPIO_Pinnumber= GPIO_PIN_NO_12;
    GPIOled.GPIO_PinConfig.GPIO_Pinmode= GPIO_MODE_OUT;
    GPIOled.GPIO_PinConfig.GPIO_Pin_Outputtype=GPIO_OP_TYPE_PP;
    GPIOled.GPIO_PinConfig.GPIO_PuPdControl=GPIO_NO_PUPD;
    GPIOled.GPIO_PinConfig.GPIO_Pinspeed=GPIO_SPEED_FAST;
    GPIO_PeriClockControl(GPIOD,ENABLE);
    GPIO_Init(&GPIOled);


}

void buttonc()
{
	//button is connected to a gpio pin so i have to configure this pi as it_ft mode

	GPIO_Handle_t button;
	button.pGPIOx=GPIOA;
	button.GPIO_PinConfig.GPIO_Pinnumber=GPIO_PIN_NO_0;
	button.GPIO_PinConfig.GPIO_Pinmode=GPIO_MODE_IT_FT;
	button.GPIO_PinConfig.GPIO_Pin_Outputtype=GPIO_OP_TYPE_PP;
	button.GPIO_PinConfig.GPIO_Pinspeed=GPIO_SPEED_FAST;
	button.GPIO_PinConfig.GPIO_PuPdControl=GPIO_PD;


	GPIO_PeriClockControl(GPIOA,ENABLE);
	GPIO_Init(&button);// we have to sen handle variable to the init fuction
	GPIO_IRQ_PriorityConfig(IRQ_NO_EXTI0,IRQ_NO_PRI1);
	GPIO_IRQ_IT_Config(IRQ_NO_EXTI0,ENABLE);
}
int main()
{
   led();
   buttonc();

while(1)

	{
		GPIO_WriteToOutputPin(GPIOD,GPIO_PIN_NO_12,GPIO_PIN_SET);
		delay(1000);
		GPIO_WriteToOutputPin(GPIOD,GPIO_PIN_NO_12,GPIO_PIN_RESET);
		delay(1000);
	}

}
void EXTI0_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NO_0);
	GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_NO_12);
	delay(500);
	GPIO_WriteToOutputPin(GPIOD,GPIO_PIN_NO_12,GPIO_PIN_SET);
	delay(1000);
}
