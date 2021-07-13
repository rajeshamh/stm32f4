/*
 * stm32f4xx_gpio_driver.h
 *  mcu specific header file
 *  Created on: Jun 8, 2021
 *      Author: rajes
 *
 *
 *      This header contains the different function prototypes for the APIs
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_



#include "stm32f407xx.h"



/*
 structure contains the pin configuration setting
*/

typedef struct
{
	uint8_t GPIO_Pinnumber;
	uint8_t GPIO_Pinmode;
	uint8_t GPIO_Pinspeed;
	uint8_t GPIO_PuPdControl;//PUSH PULL REGISTER
	uint8_t GPIO_Pin_Outputtype;
	uint8_t GPIO_PinAltfunmode;

}GPIO_PinConfig_t ;


/*
 this structure to access the different ports and port settings
*/

typedef struct
{
    GPIO_Regdef_t		*pGPIOx;			/*|>This is the pointer to holds the base address of specific SPI port*/
	GPIO_PinConfig_t	 GPIO_PinConfig;	/*|>SPI_PinConfig is structure variable holds the configuration setting */

}GPIO_Handle_t;

/*
 	 GPIO PIN NUMBERS
*/

#define GPIO_PIN_NO_0			0
#define GPIO_PIN_NO_1			1
#define GPIO_PIN_NO_2			2
#define GPIO_PIN_NO_3			3
#define GPIO_PIN_NO_4			4
#define GPIO_PIN_NO_5			5
#define GPIO_PIN_NO_6			6
#define GPIO_PIN_NO_7			7
#define GPIO_PIN_NO_8			8
#define GPIO_PIN_NO_9			9
#define GPIO_PIN_NO_10			10
#define GPIO_PIN_NO_11			11
#define GPIO_PIN_NO_12			12
#define GPIO_PIN_NO_13			13
#define GPIO_PIN_NO_14			14
#define GPIO_PIN_NO_15			15


/*
  POSSIBLE GPIO PIN MODES
*/

#define GPIO_MODE_IN			0
#define GPIO_MODE_OUT			1
#define GPIO_MODE_ALTFN			2
#define GPIO_MODE_ANALOG		3
#define GPIO_MODE_IT_FT			4                 /* GPIO PIN CONFIG AS INTRUPT SO TO SET INPUT PIN FOR FALLING EDGE TRIGGER                            */
#define GPIO_MODE_IT_RT			5
#define GPIO_MODE_IT_RFT		6

/*
 POSSIBLE GPIO PIN OUTPUT TYPES
*/

#define GPIO_OP_TYPE_PP         0
#define GPIO_OP_TYPE_OP			1

/*
 *  POSSIBLE GPIO PIN OUTPUT SPEED TYPES
*/

#define GPIO_SPEED_LOW 			0
#define	GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define	GPIO_SPEED_HIGH			3

/*
 	 	 POSSIBLE GPIO PIN PULL UP AND PULL DOWN CONFIGURARION
*/

#define GPIO_NO_PUPD            0
#define GPIO_PU					1
#define GPIO_PD					2



/*
  ********************************  Peripheral function prototype
*/

/*  Peripheral clock setup
 *
 * this function will receives the base address passed from the app.c, also receives the info weather to enable specific port ot not ,
 * in source we call this function and pass the base address so we initialized a pointer of type regdef which is sturucture which holds
 *  the different base addresses
 *
 */

void GPIO_PeriClockControl(GPIO_Regdef_t *pGPIOx,uint8_t EnorDi);

/*

 gpio   init() and deinit()
-->>    This function takes parameter as pointer,user application should send a pointer variable of type GPIO_handle_t,
        because this function is used to initialize the  specific port  and pins so using above structure we can do this.
  -->>  Deint is used to uninitialize the peripheral port so we should receive the base address of the port and pass it to the structure using the pointer
        so the variable is of the type GPIO_Regdef_t *GPIOx
*/

void 	GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void	GPIO_DeInit(GPIO_Regdef_t *pGPIOx);

/*
 *
 Data resd and write
*/

uint8_t	 GPIO_ReadFromInputPin(GPIO_Regdef_t *pGPIOx,uint8_t Pinnumber);						/*|<uint8_t because return value will be either ture or false*/
uint16_t GPIO_ReadFromInputPort(GPIO_Regdef_t *pGPIOx);											/*|<uint16_t means port has 16 pins */
void	 GPIO_WriteToOutputPin(GPIO_Regdef_t *pGPIOx,uint8_t Pinnumber,uint8_t value);			/*|<value may be set or reset */
void	 GPIO_WriteToOutputPort(GPIO_Regdef_t *pGPIOx,uint16_t value);
void 	 GPIO_ToggleOutputPin(GPIO_Regdef_t *pGPIOx,uint8_t Pinnumber);
/*
 *
iRQ configuration and handling
*/

void GPIO_IRQ_IT_Config(uint8_t IRQNumber,uint8_t EnorDi);
void GPIO_IRQ_PriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t Pinnumber);












#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
