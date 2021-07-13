/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: 15-Jun-2021
 *      Author: rajes
 */

#include "stm32f407xx.h"

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

//SPI Macros for Status Flag

#define SPI_TXE_FLAG	(1<<SPI_SR_TXE)
#define SPI_RXNE_FLAG   (1<<SPI_SR_RXNE)
#define SPI_BSY_FLAG    (1<<SPI_SR_BSY)



typedef struct
{

	uint8_t	SPI_DeviceMode;//to set master or slave user must configure this bit field in cr1
	uint8_t	SPI_BusConfig;//
	uint8_t	SPI_SclkSpeed;//what is the maxmun communiction speed
	uint8_t	SPI_DFF;//user must selsect the frame format 8 bit or 16 bit
	uint8_t	SPI_CPOL;//polarity is for edge trigger  ex rising edgw or falling
	uint8_t	SPI_CPHA;//is to tell at which clk  edge data should be proccesse
	uint8_t	SPI_SSM;//slave software elect mode



}SPI_PinConfig_t;



typedef struct
{
	SPI_Regdef_t 		*pSPIx;
	SPI_PinConfig_t      SPI_Pinconfig;
	uint8_t				*pTxBuffer;         /*|>pointer variable is to store the application Tx buffer address */
	uint8_t				*pRxBuffer;			/*|>pointer variable is to store the application Rx buffer address */
	uint32_t			 TXLEN;				/*|> to store the application Tx buffer len */
	uint32_t  			 RXLEN;             /*|> to store the application Rx buffer len */
	uint8_t				 TxState;           /*|> to store the application Tx State  */
	uint8_t				 RxSTATE;           /*|> to store the application Rx State*/


}SPI_Handle_t;

/*
* @DeviceMode
*/
#define SPI_DEVICE_MODE_MASTER	1
#define SPI_DEVICE_MODE_SLAVE	0


/*
*@BusConfig
*/
#define SPI_Bus_Config_FD				1
#define SPI_Bus_Config_HD				2
#define SPI_Bus_Config_SIMPLX_RX_ONLY	3

/*
*@SCLK_SPEEd
*/
#define SPI_SCLK_DIV2					0
#define SPI_SCLK_DIV4					1
#define SPI_SCLK_DIV8					2
#define SPI_SCLK_DIV16					3
#define SPI_SCLK_DIV32					4
#define SPI_SCLK_DIV64					5
#define SPI_SCLK_DIV128					6
#define SPI_SCLK_DIV256				    7

/*
*@DFF
*/

#define SPI_DFF_8BITS					0   //by default it will  be 8bit
#define SPI_DFF_16BITS					1

/*
*@CPOL
*/
#define SPI_CPOL_HIGH					1
#define SPI_CPOL_LOW					0
/*
*@CPHA
*/
#define SPI_CPHA_HIGH					1
#define SPI_CPHA_LOW					0


/*
*@CSSM
*/

#define SPI_SSM_EN						1
#define SPI_SSM_DI						0






/*
	peripheral clock setup
*/

void SPI_PeriClockControl(SPI_Regdef_t *pSPIx,uint8_t EnorDi);//base address and set or reset values


/*
	SPI Init and Deinit
*/
void 	SPI_Init(SPI_Handle_t *pSPIHandle);
void	SPI_DeInit(SPI_Regdef_t *pSPIx);
void    SPI_PeripheralControl(SPI_Regdef_t *pSPIx,uint8_t EnorDi);
void    SPI_SSIConfig(SPI_Regdef_t *pSPIx,uint8_t EnorDi);
void    SPI_SSOEConfig(SPI_Regdef_t *pSPIx,uint8_t EnorDi);
uint8_t SPI_GetFlagStatus(SPI_Regdef_t *pSPIx,uint8_t flagnme);
/*
    Data send and receive
*/
void    SPI_Transmit(SPI_Regdef_t* pSPIx,uint8_t *pTxbuffer,uint32_t size);
void    SPI_Receive(SPI_Regdef_t *pSPIx,uint8_t *pRxbuffer,uint32_t size);


uint8_t    SPI_Transmit_IT(SPI_Handle_t *pSPIHandle,uint8_t *pTxbuffer,uint32_t size);
uint8_t    SPI_Receive_IT(SPI_Handle_t *pSPIHandle,uint8_t *pRxbuffer,uint32_t size);


/*
 	IRQ Configuring and ISR Handling
*/

void SPI_IRQITConfig(uint8_t IRQNumber,uint8_t EnorDi);//to enable the interrupt for the irq number
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint8_t Priority);//setting the priority for the irq
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);// to handle the isr
void SPI_Close_Transmit(SPI_Handle_t *pSPIHandle);
void SPI_Close_Reception(SPI_Handle_t *pSPIHandle);
void SPI_Clear_OVRERR(SPI_Handle_t *pSPIHandle);
void SPI_event_Callback(SPI_Handle_t* pSPIHandle,uint8_t evflag);



#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */




