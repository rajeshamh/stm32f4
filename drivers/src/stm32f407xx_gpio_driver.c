/*
 * stm32f4xx_gpio_driveer.c
 *
 *  Created on: Jun 8, 2021
 *      Author: rajes
 */


#include "stm32f407xx_gpio_driver.h"

/********************************************************************************************************************************************
 * @fn 			-- 		GPIO_PeriClockControl
 *
 *@breaf		--  	This function enables or disables the peripheral clock for the given gpio port
 *
 *
 *@param[01] 	--      This Function takes base address as the first parameter this parameter is of the type of strucutr GPIO_Regdef_t
 *@param[02]    --		This is about the enabling the port priph clock or disable it,
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

void GPIO_PeriClockControl(GPIO_Regdef_t *pGPIOx,uint8_t EnorDi)
{
	 if(EnorDi == ENABLE)
		 {
			if(pGPIOx == GPIOA)
			{
				GPIOA_PCLK_EN() ;
			}

			else if(pGPIOx == GPIOB)
			{
				GPIOB_PCLK_EN() ;
			}

			else if(pGPIOx == GPIOC)
			{
				GPIOC_PCLK_EN() ;
			}
			else if(pGPIOx == GPIOD)
			{
				GPIOD_PCLK_EN() ;
			}
			else if(pGPIOx == GPIOE)
			{
				GPIOE_PCLK_EN() ;
			}
			else if(pGPIOx == GPIOF)
			{
				GPIOF_PCLK_EN() ;
			}
			else if(pGPIOx == GPIOH)
			{
				GPIOH_PCLK_EN() ;
			}
			else if(pGPIOx == GPIOI)
			{
				GPIOI_PCLK_EN() ;
			}
		 }
	 else
	 	 {
		 	 if(pGPIOx == GPIOA)
		 	 {
		 		 GPIOA_PCLK_DI() ;
		 	 }

		 	 else if(pGPIOx == GPIOB)
		 	 {
		 		 GPIOB_PCLK_DI() ;
		 	 }

		 	 else if(pGPIOx == GPIOC)
		 	 {
		 		 GPIOC_PCLK_DI() ;
		 	 }
		 	 else if(pGPIOx == GPIOD)
		 	 {
		 		 GPIOD_PCLK_DI() ;
		 	 }
		 	 else if(pGPIOx == GPIOE)
		 	 {
		 		 GPIOE_PCLK_DI() ;
		 	 }
		 	 else if(pGPIOx == GPIOF)
		 	 {
		 		 GPIOF_PCLK_DI() ;
		 	 }
		 	 else if(pGPIOx == GPIOH)
		 	 {
		 		 GPIOH_PCLK_DI() ;
		 	 }
		 	 else if(pGPIOx == GPIOI)
		 	 {
		 		 GPIOI_PCLK_DI() ;
		 	 }
		}


}

/*******************************************************************************************************
 *
 * @fn 			--  	GPIO_InIt
 *
 *@breaf		--      This function is to initialize or to configure  the gpio port,This structure will contain two member elements one to
 *@breaf		        access the gpio port through base address,this element is of the type GPIO_Regdef_t
 *@breaf		        another member to configure the specific pin ex:= output type,mode etc
 *
 *
 *@param[in]    --      this will take a port base address so we have pointeer to hold that address
 *@param[in]
 *@param[in]
 *
 *
 *@return      --		none
 *
 *
 *@note
 *
 *
*/

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	//enable th eclock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx,ENABLE);




	uint32_t temp=0;
    /* FIRST WE NEDD CONFIGURE THE PIN MODE  */
	if(pGPIOHandle->GPIO_PinConfig.GPIO_Pinmode<=GPIO_MODE_ANALOG)	//here wr are accessing the mode value enter by the user,and we are checking it ,=3
	{   //non interrupted mode
	    temp=((pGPIOHandle->GPIO_PinConfig.GPIO_Pinmode)<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_Pinnumber));//each pin corresponds to 2 bits in moder registeer
	    pGPIOHandle->pGPIOx->MODER&=~(0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_Pinnumber);
		pGPIOHandle->pGPIOx->MODER|=temp;
	}
	else
	{
		//interrupted mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_Pinmode==GPIO_MODE_IT_RT)
		{
			//set rtsr register
			EXTI->RTSR|=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_Pinnumber);
			//clear corresponding ftsr bit
			EXTI->FTSR&=~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_Pinnumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_Pinmode==GPIO_MODE_IT_FT)
		{
			//set FTSR register
			EXTI->FTSR|=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_Pinnumber);
			//Clear the RTSR
			EXTI->RTSR&=~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_Pinnumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_Pinmode==GPIO_MODE_IT_RFT)
		{
			EXTI->RTSR|=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_Pinnumber);
			EXTI->FTSR|=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_Pinnumber);
		}
		//configure the gpio port slection in syscfg
		 uint8_t temp=pGPIOHandle->GPIO_PinConfig.GPIO_Pinnumber/4;//to determine which control regitser to assign
		 uint8_t temp1=pGPIOHandle->GPIO_PinConfig.GPIO_Pinnumber%4;//to determine which section the pin or port to be assigned
		 uint8_t portcode=GPIOX_BASEEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		 SYSCFG->EXTICR[temp]=portcode<<(4*temp1);//we are left shifting 4*temp1 times the port code ,determines the which pin of the CR to be configure to a port



		//enable the interrupt delivery using the IMR register

		EXTI->IMR|=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_Pinnumber);//when we enable the



	}

	 /* FIRST WE NEDD CONFIGURE THE PIN speed MODE  */

	temp=0;
	temp=((pGPIOHandle->GPIO_PinConfig.GPIO_Pinspeed)<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_Pinnumber));
	pGPIOHandle->pGPIOx->OSPEEDR  &= ~(0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_Pinnumber);
	pGPIOHandle->pGPIOx->OSPEEDR |=temp;

	 temp=0;
	/* OUTPUT TYPE */
    temp=((pGPIOHandle->GPIO_PinConfig.GPIO_Pin_Outputtype)<<(pGPIOHandle->GPIO_PinConfig.GPIO_Pinnumber));
    pGPIOHandle->pGPIOx->OTYPER &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_Pinnumber);
    pGPIOHandle->pGPIOx->OTYPER |=temp;

    /*output pushpull contol register*
     */
    temp=0;
    temp=((pGPIOHandle->GPIO_PinConfig.GPIO_PuPdControl)<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_Pinnumber));
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_Pinnumber);//clearing the corresponding bits
    pGPIOHandle->pGPIOx->PUPDR |=temp;                                               //setting the corresponding bits


    temp=0;
    //Alternate function mode
    if(pGPIOHandle->GPIO_PinConfig.GPIO_Pinmode==GPIO_MODE_ALTFN)
    {
    	uint8_t temp1,temp2;
    	temp1=pGPIOHandle->GPIO_PinConfig.GPIO_Pinnumber/8;
    	temp2=pGPIOHandle->GPIO_PinConfig.GPIO_Pinnumber%8;
    	pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF<<(4*temp2));
    	pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltfunmode<<(4*temp2));

    }

 }

/*@fn 			--  		GPIO_De_Init
 *
 *@breaf		--      	this is to de initialize the port
 *
 *
 *@param[in]	--			this will take the base address of the register ie reset register address rstr
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





void	GPIO_DeInit(GPIO_Regdef_t *pGPIOx)
{

   if(pGPIOx == GPIOA)
	{
	   GPIOA_REG_RESET() ;
	}

	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET() ;
	}

	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET() ;
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET() ;
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET() ;
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOG_REG_RESET() ;
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET() ;
	}
	else if(pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET() ;
	}








}

/*@fn 			--  	GPIO_ReadFromInputPin
 *
 *@breaf		--		To configure the specific pin of the specific port and read the data from the idr for the specific pin
 *@breaf		        each bit in idr corresponds to pin number,we shift the idr data to least significant bit and mask all
 *@breaf		        the other bits
 *
 *
 *@param[in]	--      base address of specific port
 *@param[in]
 *@param[in]	--		pin number
 *
 *
 *@return		--		it will return the true or false
 *
 *
 *@note
 *
 *
*/

uint8_t	 GPIO_ReadFromInputPin(GPIO_Regdef_t *pGPIOx,uint8_t Pinnumber)
{
	uint8_t value;
	value=(uint8_t)((pGPIOx->IDR>>Pinnumber)&(0x00000001));

	return value;
}
/*@fn 			--  		GPIO_ReadFromInputPort
 *
 *@breaf		--			to read the data from the specific port data register
 *
 *
 *@param[in]	--			it will take only base address for the port
 *@param[in]
 *@param[in]
 *
 *
 *@return		--			it returns 16bit data
 *
 *
 *@note
 *
 *
*/
uint16_t GPIO_ReadFromInputPort(GPIO_Regdef_t *pGPIOx)
{

	uint16_t data;
	data=(uint16_t) pGPIOx->IDR;

	return data;

}

/*@fn 			--  		GPIO_WriteToOutputPin
 *
 *@breaf		--			This function is to modify the specific port and set the send data in the ODR
 *
 *
 *@param[in]	--			base address for the port
 *@param[in]	--			pint number of the port
 *@param[in]	--			value ie set or reset to on or off the pin
 *
 *
 *@return
 *
 *
 *@note
 *
 *
*/

void	 GPIO_WriteToOutputPin(GPIO_Regdef_t *pGPIOx,uint8_t Pinnumber,uint8_t value)
{
	if(value==GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1<<Pinnumber);//writing to corresponding bit field in the data register
	}
	else
	{
		pGPIOx->ODR &= ~(1<<Pinnumber);
	}


}
/*@fn 		-  		GPIO_InIt
 *
 *@breaf	-
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



/*@fn 			--  		GPIO_WriteToOutputPort
 *
 *@breaf		--			to set or reset the specific port
 *
 *
 *@param[in]	--			takes base address for the port
 *@param[in]	--			set or reset  value
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
void	 GPIO_WriteToOutputPort(GPIO_Regdef_t *pGPIOx,uint16_t value)
{
		pGPIOx->ODR=value;
}

/*@fn 			--  		GPIO_ToggleOutputPin
 *
 *@breaf		--			to toggle the specific pin in the port
 *
 *
 *@param[in]	--			takes base address
 *@param[in]	--			takes pin number
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


void 	 GPIO_ToggleOutputPin(GPIO_Regdef_t *pGPIOx,uint8_t Pinnumber)
{

	pGPIOx->ODR^=(1<<Pinnumber);


}

/*@fn 			--  		GPIO_IRQConfig
 *
 *@breaf		-		configure the interrupt for the port
 *
 *
 *@param[in]	--		takes irq numbeer
 *@param[in]	--		priority
 *@param[in]	--		set or reset value
 *
 *
 *@return
 *
 *
 *@note
 *
 *
*/
void GPIO_IRQ_IT_Config(uint8_t IRQNumber,uint8_t EnorDi)
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
void GPIO_IRQ_PriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{

	uint8_t iprx=IRQNumber/4;//to find in which register the irq no belngs

	uint8_t iprx_section=IRQNumber%4;//to select particular section in a particular register

	uint8_t shift_section=(iprx_section*8)+(8-NO_PR_BITS_IMPLEMENTED);

	*(NVIC_IPR_BASEADDR+iprx)|=(IRQPriority<<shift_section);//(NVIC_IPR_BASEADDR+iprx) is a pointer arithmetic operation performed to move

}




void GPIO_IRQHandling(uint8_t Pinnumber)
{
	if(EXTI->PR &(1<<Pinnumber))//if Pending register bit corresponding to pin number is set then
     {
	 EXTI->PR |=(1<<Pinnumber);
	 }
}


