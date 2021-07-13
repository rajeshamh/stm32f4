/*
 * 006spi_send.c
 *
 *  Created on: 19-Jun-2021
 *      Author: rajes
 */


#include "stm32f407xx.h"
#include <string.h>
/*
 * PB14--->SPI2_MISO
 * PB15--->SPI2_MOSI
 * PB13--->SPI2_SCLK
 * PB12--->SPI2_NSS
 */

/*
 * define handle variables
 */

GPIO_Handle_t gpio_con;
SPI_Handle_t  spi_con;


/*
 * define variables
 */

char Tx_Data[]="Hello World";

void delay(int data)
{
	int i;
	for(i=0;i<data*data*2;i++);
}



void  SPI_GPIOinit()
{

	gpio_con.pGPIOx=GPIOB;
	gpio_con.GPIO_PinConfig.GPIO_Pinmode=GPIO_MODE_ALTFN;
	gpio_con.GPIO_PinConfig.GPIO_PinAltfunmode=5;//AF5
	gpio_con.GPIO_PinConfig.GPIO_Pin_Outputtype=GPIO_OP_TYPE_PP;
	gpio_con.GPIO_PinConfig.GPIO_PuPdControl=GPIO_NO_PUPD;
	gpio_con.GPIO_PinConfig.GPIO_Pinspeed=GPIO_SPEED_FAST;

	//MOSI
	gpio_con.GPIO_PinConfig.GPIO_Pinnumber=GPIO_PIN_NO_15;
	GPIO_Init(&gpio_con);
	//MISO
	gpio_con.GPIO_PinConfig.GPIO_Pinnumber=GPIO_PIN_NO_14;
	GPIO_Init(&gpio_con);
	//SCLK
	gpio_con.GPIO_PinConfig.GPIO_Pinnumber=GPIO_PIN_NO_13;
	GPIO_Init(&gpio_con);
	//NSS
	gpio_con.GPIO_PinConfig.GPIO_Pinnumber=GPIO_PIN_NO_12;
	GPIO_Init(&gpio_con);


}

void SPI2_InIt()
{
	spi_con.pSPIx=SPI2;
	spi_con.SPI_Pinconfig.SPI_BusConfig=SPI_Bus_Config_FD;
	spi_con.SPI_Pinconfig.SPI_DeviceMode=SPI_DEVICE_MODE_MASTER;
	spi_con.SPI_Pinconfig.SPI_SclkSpeed=SPI_SCLK_DIV2;
	spi_con.SPI_Pinconfig.SPI_DFF=SPI_DFF_8BITS;
	spi_con.SPI_Pinconfig.SPI_CPHA=SPI_CPHA_LOW;
	spi_con.SPI_Pinconfig.SPI_CPOL=SPI_CPOL_LOW;
	spi_con.SPI_Pinconfig.SPI_SSM=SPI_SSM_DI;

	SPI_Init(&spi_con);

}



void toggleonpress()
{
	GPIO_Handle_t buttonled;
	buttonled.pGPIOx=GPIOA;
	buttonled.GPIO_PinConfig.GPIO_Pinnumber= GPIO_PIN_NO_0;
	buttonled.GPIO_PinConfig.GPIO_Pinmode= GPIO_MODE_IN;
	buttonled.GPIO_PinConfig.GPIO_PuPdControl=GPIO_NO_PUPD;
	buttonled.GPIO_PinConfig.GPIO_Pinspeed=GPIO_SPEED_FAST;

	GPIO_Init(&buttonled);


}


int main()
{




	toggleonpress();
    //This Function is used to INitialise the GPIO as SPI
	SPI_GPIOinit();
	//This Function is used to Configure the Peripheral
	SPI2_InIt();

	SPI_SSOEConfig(SPI2,ENABLE );
	while(1)
	{
		//wait till button to be pressed
		while(! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0));
		delay(500);

		//This Function is used to Enable the SPI Peripheral
		SPI_PeripheralControl(SPI2,ENABLE);

		uint8_t datalen=strlen(Tx_Data);
		SPI_Transmit(SPI2,&datalen,1);



		//this function is used to send the data
		SPI_Transmit(SPI2,(uint8_t *)Tx_Data,strlen(Tx_Data));

		//before disabling check for busy flag,if not busy then disable it
	    while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));

		SPI_PeripheralControl(SPI2,DISABLE);
	}

}
