/*
 * SPI_FULL_DUPLEX.c
 *
 *  Created on: 23-Jun-2021
 *      Author: rajes
 */


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
 *
 * Command codes
 */
#define COMMAND_LED_CNTRL	0X50
#define COMMAND_SENSOR_READ 0X51
#define	COMMAND_LED_READ	0X52
#define	COMMAND_PRINT		0X53
#define	COMMAND_ID_READ		0X54

#define LED_ON               1
#define LED_OFF				 0

#define	ANALOG_PIN0          0
#define	ANALOG_PIN1          1
#define	ANALOG_PIN2          2
#define	ANALOG_PIN3          3
#define	ANALOG_PIN4          4

#define	LED_PIN          9

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

uint8_t	SPI_Verifyslave_Response(uint8_t data)
{
	if(data==0xF5)
	{
		//for ack
		return 0;

	}
	else return 0;
}
int main()
{


	uint8_t dummy_byte=0xff;
	uint8_t dummy_Read;
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

		//commnad_cntrl  pin_no  value
		uint8_t ack_bbyte;
		uint8_t args[2];
	    uint8_t command_code=COMMAND_LED_CNTRL;
	    SPI_Transmit(SPI2,&command_code,1);
	    //To clear the RXNE_FLAg
	    SPI_Receive(SPI2,&dummy_Read,1);
	    //send some dummy data to fetch the slave response
	    SPI_Transmit(SPI2,&dummy_byte,1);

	    //read the ack byte
	    SPI_Receive(SPI2,&ack_bbyte,1);

	    //verify the slave response
	    if(SPI_Verifyslave_Response(ack_bbyte))
	    		{
	    	     args[0]=LED_PIN;
	    	     args[1]=LED_ON;
	    	     SPI_Transmit(SPI2,&args,2);
	    		}

		//this function is used to send the data
		SPI_Transmit(SPI2,(uint8_t *)Tx_Data,strlen(Tx_Data));

		//before disabling check for busy flag,if not busy then disable it
	    while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));

		SPI_PeripheralControl(SPI2,DISABLE);
	}

}










