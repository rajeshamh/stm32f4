/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: 16-Jun-2021
 *      Author: rajes
 */


#include "stm32f407xx.h"


static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_errer_interrupt_handle(SPI_Handle_t *pSPIHandle);

/********************************************************************************************************************************************
 * @fn 			-- 		SPI_PeriClockControl
 *
 *@breaf		--  	This function is to enable or disable the peripheral clock for the corresponding SPI port
 *
 *
 *@param[01] 	--      This Function takes base address as the first parameter,
 *			            this parameter is of the type  SPIx_Regdef_t
 *@param[02]    --		Is SET Or RESET value
 *@param[in]
 *
 *
 *@return		--		This Function will not return anything
 *
 *
 *@note
 *
 *
*/

void SPI_PeriClockControl(SPI_Regdef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi==ENABLE)
	{
		if(pSPIx==SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if(pSPIx==SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSPIx==SPI3)
		{
			SPI3_PCLK_EN();
		}
		else if(pSPIx==SPI3)
		{
			SPI4_PCLK_EN();
		}
	}
	else
	{
		if(pSPIx==SPI1)
			{
			SPI1_PCLK_DI();
			}
		else if(pSPIx==SPI2)
			{
			SPI2_PCLK_DI();
			}
		else if(pSPIx==SPI3)
			{
			SPI3_PCLK_DI();
			}
		else if(pSPIx==SPI3)
			{
			SPI4_PCLK_DI();
			}
	}
}

/*******************************
* @fn 			-- 		SPI_Init
 *
 *@breaf		--  	This function is to Initialize the SPI Peripheral
 *
 *
 *@param[01] 	--
 *@param[02]    --
 *
 *
 *@return		--		This Function will not return anything
 *
 *
 *@note
 *
 *
*********************************/
void 	SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//enable the clock
	SPI_PeriClockControl(pSPIHandle->pSPIx,ENABLE);
	//to collect the bit field
	uint32_t tempreg=0;
    //configure mode
    tempreg|=pSPIHandle->SPI_Pinconfig.SPI_DeviceMode<<SPI_CR1_MSTR;
	//pSPIHandle->pSPIx->CR1|=(pSPIHandle->SPI_Pinconfig.SPI_DeviceMode<<2);

    //bus config
	if (pSPIHandle->SPI_Pinconfig.SPI_BusConfig ==SPI_Bus_Config_FD)
		{
		//clear the bidi bit in the cr1;
		tempreg &=~(1<<SPI_CR1_BIDIMODE)	;
		//pSPIHandle->pSPIx->CR1 &= ~(1<<15);//
		}
	else if(pSPIHandle->SPI_Pinconfig.SPI_BusConfig==SPI_Bus_Config_HD)
		{
		//set the bidi bit in the cr1
		tempreg |=(1<<SPI_CR1_BIDIMODE)	;
		//	pSPIHandle->pSPIx->CR1|=1<<15;
		}
	else if(pSPIHandle->SPI_Pinconfig.SPI_BusConfig==SPI_Bus_Config_SIMPLX_RX_ONLY)
		{
		//bidi mode must be cleared
		tempreg &=~(1<<SPI_CR1_BIDIMODE)	;
		//pSPIHandle->pSPIx->CR1 &= ~(1<<15);
		//set the rx only bit in the cr1 register
		tempreg |=(1<<SPI_CR1_BIDIMODE);
		//pSPIHandle->pSPIx->CR1|=1<<10;
		}

	//configure the spi clock speed
	tempreg|=pSPIHandle->SPI_Pinconfig.SPI_SclkSpeed<<SPI_CR1_BR;
	//pSPIHandle->pSPIx->CR1|=(pSPIHandle->SPI_Pinconfig.SPI_SclkSpeed<<3);


	//configure the dff
	tempreg|=(pSPIHandle->SPI_Pinconfig.SPI_DFF<<SPI_CR1_DFF);
	//pSPIHandle->pSPIx->CR1|=(pSPIHandle->SPI_Pinconfig.SPI_DFF<<11);

	//configure the  cpol
	tempreg|=(pSPIHandle->SPI_Pinconfig.SPI_CPOL<<SPI_CR1_CPOL);
	//pSPIHandle->pSPIx->CR1|=(pSPIHandle->SPI_Pinconfig.SPI_CPOL<<1);

	//configure the cpha
	tempreg|=(pSPIHandle->SPI_Pinconfig.SPI_CPHA<<SPI_CR1_CPHA);
	//pSPIHandle->pSPIx->CR1|=(pSPIHandle->SPI_Pinconfig.SPI_CPHA<<0);

	//configure the ssm
	tempreg|=(pSPIHandle->SPI_Pinconfig.SPI_SSM<<SPI_CR1_SSM);
	//pSPIHandle->pSPIx->CR1|=(pSPIHandle->SPI_Pinconfig.SPI_SSM<<9);

	pSPIHandle->pSPIx->CR1=tempreg;
}
/*******************************
* @fn 			-- 		GPIO_PeriClockControl
 *
 *@breaf		--  	To Enable or Disable the corresponding  SPI peripheral
 *
 *
 *@param[01] 	--      Base Address of the Peripheral whis of tyoe GPIO_Regdef_t
 *@param[02]    --		Set or RESET value
 *@param[in]	--		none
 *
 *
 *@return		--		This Function will not return nothig
 *
 *
 *@note			--		To achieve the communication we should enable it after the all configuration
 *
 *
*********************************/
void    SPI_PeripheralControl(SPI_Regdef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi==ENABLE)
	{
		pSPIx->CR1|=(1<<SPI_CR1_SPE);
	}
	else
	{

		pSPIx->CR1&=~(1<<SPI_CR1_SPE);

	}
}
/*******************************
* @fn 			-- 		SPI_SSOEConfig
 *
 *@breaf		--  	*This Function Manages The NSS pin during Hardware Slave Select Mode
 *
 *
 *@param[01] 	--      This Function takes base address as the  parameter this parameter is of the type of structure SPI_Regdef_t
 *@param[02]    --		Is a Set Or Reset value
 *@param[in]	--		none
 *
 *
 *@return		--		This Function will not return nothing
 *
 *
 *@note			--		When Enabled SSOE ,It keeps NSS pin HIGH until SPE is Enabled,ie
 *@note			--      ---->SPE=1  NSS=0;
 *@note			--		---->SPE=0  NSS=1,so it help to manage nss pin
 *
 *
*********************************/

void  SPI_SSOEConfig(SPI_Regdef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi==ENABLE)
	{
		pSPIx->CR2|=(1<<SPI_CR2_SSOE);//enabling the slave select output enable,
	}
	else
	{

		pSPIx->CR2&=~(1<<SPI_CR2_SSOE);

	}
}


/*******************************
* @fn 			-- 		SPI_SSIConfig
 *
 *@breaf		--  	To manage the NSS PIN when software slave management is enabled,we should configure this function
 *
 *
 *@param[01] 	--      This Function takes base address as the  parameter this parameter is of the type of structure SPI_Regdef_t
 *@param[02]    --		Is a Set Or Reset value
 *@param[in]	--		none
 *
 *
 *@return		--	    none
 *
 *
 *@note			--		When Enabled SSI ,It keeps NSS pin HIGH
 *@note			--      ---->SPE=1  NSS=1   SSM=1;
 *@note			--		IF SSM=0 then no need to configure this bit
 *
 *
*********************************/

void  SPI_SSIConfig(SPI_Regdef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi==ENABLE)
	{
		pSPIx->CR1|=(1<<SPI_CR1_SSI);
	}
	else
	{

		pSPIx->CR1&=~(1<<SPI_CR1_SSI);

	}
}

/*******************************
* @fn 			-- 		SPI_DeInit
 *
 *@breaf		--  	This function resets the corresponding spi port
 *
 *
 *@param[01] 	--      This Function takes base address as the  parameter this parameter is of the type of strucutr GPIO_Regdef_t
 *@param[02]    --		none
 *@param[in]	--		none
 *
 *
 *@return		--		None
 *
 *
 *@note
 *
 *
*********************************/

void	SPI_DeInit(SPI_Regdef_t *pSPIx)
{
	if(pSPIx==SPI1)
	{
		SPI1_REG_RESET();
	}
	else if(pSPIx==SPI2)
	{
		SPI2_REG_RESET();
	}
	else if(pSPIx==SPI3)
	{
		SPI3_REG_RESET();
	}
	else if(pSPIx==SPI4)
	{
		SPI4_REG_RESET();
	}

}

/*******************************
* @fn 			-- 		SPI_GetFlagStatus
 *
 *@breaf		--  	This function is to check the  status flags if spi peripherals
 *
 *
 *@param[01] 	--      This Function takes base address as the  parameter this parameter is of the type of strucutr GPIO_Regdef_t
 *@param[02]    --		FLAG To check the Status
 *@param[in]	--		none
 *
 *
 *@return		--		Will return boolean values ie:SET OR RESET values
 *
 *
 *@note
 *
 *
*********************************/

uint8_t SPI_GetFlagStatus(SPI_Regdef_t *pSPIx,uint8_t flagnme)
{
	if(pSPIx->SR & flagnme)
	{
		return FLAG_SET ;
	}
	return FLAG_RESET;
}

/*******************************
* @fn 			-- 		SPI_Transmit
 *
 *@breaf		--  	Performs the data transmission
 *
 *
 *@param[01] 	--      This Function takes base address as the  parameter this parameter is of the type  GPIO_Regdef_t
 *@param[02]    --		User buffer name
 *@param[in]	--		Size of the buffer
 *
 *
 *@return		--		None
 *
 *
 *@note
 *
 *****/
void SPI_Transmit(SPI_Regdef_t *pSPIx,uint8_t *pTxbuffer,uint32_t size)
{
	while(size>0)
	{

	    //wait till tx buffer gets empty

	   while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG)  == FLAG_RESET );
		//while(!(pSPIx->SR  &  1<<SPI_SR_TXE));
	  if(pSPIx->CR1 & (1<<SPI_CR1_DFF))
		{
			pSPIx->DR=*((uint16_t *)pTxbuffer);
			size--;
			size--;
			(uint16_t *)pTxbuffer++;
		}else
		{
			pSPIx->DR=*pTxbuffer;
			size--;
			pTxbuffer++;
		}

	}
}
/*******************************
* @fn 			-- 		SPI_Transmit
 *
 *@breaf		--  	Performs the data transmission
 *
 *
 *@param[01] 	--      This Function takes base address as the  parameter this parameter is of the type  GPIO_Regdef_t
 *@param[02]    --		User buffer name
 *@param[in]	--		Size of the buffer
 *
 *
 *@return		--		None
 *
 *
 *@note
 *
 *****/
void SPI_Receive(SPI_Regdef_t *pSPIx,uint8_t *pRxbuffer,uint32_t size)
{
	while(size>0)
	{

	    //wait till tx buffer gets empty

		 while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG)==FLAG_RESET);
		//while(!(pSPIx->SR  &  1<<SPI_SR_RXNE));
		if(pSPIx->CR1 & (1<<SPI_CR1_DFF))
		{   //load the data from the DR
			*((uint16_t *)pRxbuffer)=pSPIx->DR;
			size--;
			size--;
			(uint16_t *)pRxbuffer++;

		}
		else
		{
			*(pRxbuffer)=pSPIx->DR;
			size--;
			pRxbuffer++;
		}


	}
}

/*******************************
* @fn 			-- 		SPI_Transmit
 *
 *@breaf		--  	Performs the data transmission
 *
 *
 *@param[01] 	--      This Function takes base address as the  parameter this parameter is of the type  GPIO_Regdef_t
 *@param[02]    --		User buffer name
 *@param[in]	--		Size of the buffer
 *
 *
 *@return		--		None
 *
 *
 *@note
 *
 *****/
void SPI_IRQ_IT_Config(uint8_t IRQNumber,uint8_t EnorDi)
{
		if(EnorDi==ENABLE)
		{
			if(IRQNumber<32)
			{
				//enable the NVIC_ISER0
				*NVIC_ISER0 |= (1<<IRQNumber);
			}
			else if(IRQNumber>=32&&IRQNumber<64)
			{
				//enable the NVIC_ISER1
				*NVIC_ISER1 |= (1<<IRQNumber%32);
			}
			else if(IRQNumber>=64&&IRQNumber<95)
			{
				//enable the NVIC_ISER2
				*NVIC_ISER1 |= (1<<IRQNumber%64);//WE ARE USING MOD OPERATO TO ASSIGN THE EACH BIT TO CORRESPONDING IRQ NO EX 64%64=0,BIT 0WL BE ASGND
			}
		}
		else
		{
                //interupt clear enable register
			if(IRQNumber<32)
			{
				//configure the NVIC_ICER0
				*NVIC_ICER0 |=(1<<IRQNumber);
			}
			else if(IRQNumber>=32&&IRQNumber<64)
			{
				//configure the NVIC_ICER1
				*NVIC_ICER0 |=(1<<IRQNumber%32);
			}
			else if(IRQNumber>=64&&IRQNumber<96)
			{
				//configure the NVIC_ICER2
				*NVIC_ICER0 |=(1<<IRQNumber%64);
			}
		}

}

/*@fn 			--  	GPIO_IRQHandling
 *
 *@breaf		--
 *
 *
 *@param[in]
 *@param[in]
 *@param[in]
 *
 *
 *@return
 *
 *
 *@note
 *
 *
*/

void SPI_IRQ_PriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{

	uint8_t iprx=IRQNumber/4;//to find in which register the irq no belngs

	uint8_t iprx_section=IRQNumber%4;//to select particular section in a particular register

	uint8_t shift_section=(iprx_section*8)+(8-NO_PR_BITS_IMPLEMENTED);

	*(NVIC_IPR_BASEADDR+iprx)|=(IRQPriority<<shift_section);//(NVIC_IPR_BASEADDR+iprx) is a pointer arithmetic operation performed to move

}



uint8_t   SPI_Transmit_IT(SPI_Handle_t *pSPIHandle,uint8_t *pTxbuffer,uint32_t size)
{
	    uint8_t state=pSPIHandle->TxState;
	    if(state !=SPI_BUSY_IN_TX)
	    {
		/*Save the Buffer address in some Global variables	*/
		pSPIHandle->pTxBuffer= pTxbuffer;

		/*Save the length information in a global variable*/
		pSPIHandle->TXLEN=size;

		/*Mark The SPI state as Busy */
		pSPIHandle->TxState=SPI_BUSY_IN_TX;
		/* Set the TXIE control bit */
		pSPIHandle->pSPIx->CR2|=(1<<SPI_CR2_TXEIE);


	    }
	    return state;




}
uint8_t    SPI_Receive_IT(SPI_Handle_t *pSPIHandle,uint8_t *pRxbuffer,uint32_t size)
{
	        uint8_t state=pSPIHandle->RxSTATE;
		    if(state !=SPI_BUSY_IN_RX)
		    {
			/*Save the Buffer address in some Global variables	*/
			pSPIHandle->pTxBuffer=pRxbuffer;

			/*Save the length information in a global variable*/
			pSPIHandle->RXLEN=size;

			/*Mark The SPI state as Busy */
			pSPIHandle->TxState=SPI_BUSY_IN_RX;
			/* Set the RXNEIE control bit */
			pSPIHandle->pSPIx->CR2|=(1<<SPI_CR2_RXNEIE);


		    }
		    return state;
}


void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)// to handle the isr
{
	uint8_t temp1;
	uint8_t temp2;

	//check for the TXE Flag

	temp1=pSPIHandle->pSPIx->SR&(1<<SPI_SR_TXE);
	temp2=pSPIHandle->pSPIx->CR2&(1<<SPI_CR2_TXEIE);

	if(temp1==temp2)
	{
		//handle the txe
		spi_txe_interrupt_handle(pSPIHandle);
	}


	//check for the RXNE Flag

	temp1=pSPIHandle->pSPIx->SR&(1<<SPI_SR_RXNE);
	temp2=pSPIHandle->pSPIx->CR2&(1<<SPI_CR2_RXNEIE);

		if(temp1==temp2)
		{
			//handle the txe
			spi_rxne_interrupt_handle(pSPIHandle);
		}
	//check for the overrun error
	temp1=pSPIHandle->pSPIx->SR&(1<<SPI_SR_OVR);
	temp2=pSPIHandle->pSPIx->CR2&(1<<SPI_CR2_ERRIE);

		if(temp1==temp2)
		{
			//handle the txe
			//spi_errer_interrupt_handle();

		spi_errer_interrupt_handle(pSPIHandle);
		}

}

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if(pSPIHandle->pSPIx->CR1 & (1<<SPI_CR1_DFF))
			{
		pSPIHandle->pSPIx->DR=*((uint16_t *)pSPIHandle->pTxBuffer);
		pSPIHandle->TXLEN--;
		pSPIHandle->TXLEN--;
		pSPIHandle->pTxBuffer++;
			}
	else
			{
				pSPIHandle->pSPIx->DR=*((uint16_t *)pSPIHandle->pTxBuffer);
				pSPIHandle->TXLEN--;
				pSPIHandle->pTxBuffer++;
			}

   if(! pSPIHandle->TXLEN)
   {
	   //
	   SPI_Close_Transmit(pSPIHandle);
	   SPI_event_Callback(pSPIHandle,SPI_EVENT_TX_CMPLT);

   }
}
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if(pSPIHandle->pSPIx->CR1 & (1<<SPI_CR1_DFF))
			{   //load the data from the DR
				*((uint16_t *)pSPIHandle->pRxBuffer)=pSPIHandle->pSPIx->DR;
				pSPIHandle->RXLEN--;
				pSPIHandle->RXLEN--;
				pSPIHandle->pRxBuffer--;
				pSPIHandle->pRxBuffer--;
			}
	else
			{
				*(pSPIHandle->pRxBuffer)=pSPIHandle->pSPIx->DR;
				pSPIHandle->RXLEN--;
				pSPIHandle->pRxBuffer--;
			}
	if(!pSPIHandle->RXLEN)
	{
		SPI_Close_Reception(pSPIHandle);
		SPI_event_Callback(pSPIHandle,SPI_EVENT_RX_CMPLT);
	}
}
static void spi_errer_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	if(pSPIHandle->TxState|=SPI_BUSY_IN_TX)
	{
		 temp=pSPIHandle->pSPIx->DR;
		 temp=pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	SPI_event_Callback(pSPIHandle,SPI_EVENT_RX_CMPLT);

}

void SPI_Close_Transmit(SPI_Handle_t *pSPIHandle)
{
	 pSPIHandle->pSPIx->CR2 &=~(1<<SPI_CR2_TXEIE);
	 pSPIHandle->pTxBuffer=0U;
	 pSPIHandle->TXLEN=0U;
	 pSPIHandle->TxState=SPI_READY;
}
void SPI_Clear_OVRERR(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	temp=pSPIHandle->pSPIx->DR;
	temp=pSPIHandle->pSPIx->SR;
	(void)temp;
}





void SPI_Close_Reception(SPI_Handle_t *pSPIHandle)
{

	*(uint16_t*)pSPIHandle->pSPIx->CR2 &= ~(1<<SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer=0U;
	pSPIHandle->RXLEN=0U;
	pSPIHandle->RxSTATE=SPI_READY;

}

__weak  void SPI_event_Callback(SPI_Handle_t* pSPIHandle,uint8_t evflag)
{




}






























