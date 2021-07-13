/*
 * stm32f407xx.h
 *
 *  Created on: Jun 6, 2021
 *      Author: rajes
 *      this is a devic specific header file contains the macros for the base address
 */



#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_



#include <stdint.h>
#include<stddef.h>





#define	NO_PR_BITS_IMPLEMENTED	 4
#define __vo                	 volatile

/*
 *************************************************processor specific details**********************************************************************
*
* ARM Cortex M4 Processor  Base addresses for the NVIC Registers
*/

#define	 NVIC_ISER0			((__vo uint32_t *)0xE000E100)
#define	 NVIC_ISER1			((__vo uint32_t *)0xE000E104)
#define	 NVIC_ISER2			((__vo uint32_t *)0xE000E108)
#define	 NVIC_ISER3			((__vo uint32_t *)0xE000E10c)

/*
 *  ARM Cortex M4 Processor  Base addresses for the NVIC_ICERx Registers
 */
#define NVIC_ICER0			((__vo uint32_t *)0XE000E180)
#define NVIC_ICER1			((__vo uint32_t *)0XE000E184)
#define NVIC_ICER2			((__vo uint32_t *)0XE000E188)
#define NVIC_ICER3			((__vo uint32_t *)0XE000E18c)
/*
 *  ARM Cortex M4 Processor  Base addresses for the NVIC_ICERx Registers
 */

#define NVIC_IPR_BASEADDR			((__vo uint32_t *)0xE000E400)


/*
* Base Addresses for the memory  *********      **           ***********      *       ***************          *****************
*/

#define FLASBASE_ADDR	    0x08000000U
#define SRAM1BASEADDR	    0x20000000U//112*1024=1C000
#define SRMA2BASEADDR		0x2001C000U
#define ROM					0x1FFF0000U
#define SRAM 				SRAM1BASEADDR

/*          peripheral BUS base address                      */

#define PERIPHERAL_BASEADDR 0x40000000U
#define APB1PERIPH_BASEADDR	0x40000000U
#define APB2PERIPH_BASEADDR	0x40010000U
#define AHB1PERIPH_BASEADDR	0x40020000U
#define AHB2PERIPH_BASEADDR 0x50000000U

// macros for base addresses of peripherals  connected to AHB1      //

#define	GPIOA_BASEADDR		(AHB1PERIPH_BASEADDR + 0X0000)
#define	GPIOB_BASEADDR		(AHB1PERIPH_BASEADDR + 0X0400)
#define	GPIOC_BASEADDR		(AHB1PERIPH_BASEADDR + 0X0800)
#define	GPIOD_BASEADDR		(AHB1PERIPH_BASEADDR + 0X0C00)
#define GPIOE_BASEADDR		(AHB1PERIPH_BASEADDR + 0X1000)
#define	GPIOF_BASEADDR		(AHB1PERIPH_BASEADDR + 0X1400)
#define	GPIOG_BASEADDR		(AHB1PERIPH_BASEADDR + 0X1800)
#define	GPIOH_BASEADDR		(AHB1PERIPH_BASEADDR + 0X1C00)
#define GPIOI_BASEADDR		(AHB1PERIPH_BASEADDR + 0X2000)
#define GPIOJ_BASEADDR		(AHB1PERIPH_BASEADDR + 0X2400)
#define GPIOK_BASEADDR		(AHB1PERIPH_BASEADDR + 0X2800)
#define RCC_BASEADDR        (AHB1PERIPH_BASEADDR + 0x3800)

/*  macros for the base addresses of peripheral  connected to APB1  */

#define I2C1_BASEADDR       (APB1PERIPH_BASEADDR + 0X5400)
#define I2C2_BASEADDR		(APB1PERIPH_BASEADDR + 0X5800)
#define I2C3_BASEADDR		(APB1PERIPH_BASEADDR + 0X5C00)

#define SPI3_BASEADDR		(APB1PERIPH_BASEADDR + 0X3C00)
#define	SPI2_BASEADDR		(APB1PERIPH_BASEADDR + 0X3800)

#define	UART4_BASEADDR		(APB1PERIPH_BASEADDR + 0X4C00)
#define	UART5_BASEADDR		(APB1PERIPH_BASEADDR + 0X5000)
#define	UART7_BASEADDR		(APB1PERIPH_BASEADDR + 0X7800)
#define	UART8_BASEADDR		(APB1PERIPH_BASEADDR + 0X7C00)

#define USART2_BASEADDR		(APB1PERIPH_BASEADDR + 0X4400)
#define USART3_BASEADDR		(APB1PERIPH_BASEADDR + 0X4800)


/* macros for the base addresses of peripheral  connected to APB2        */

#define EXTI_BASEADDR		(APB2PERIPH_BASEADDR + 0X3C00)

#define ADC1_BASEADDR		(APB2PERIPH_BASEADDR + 0X2000)
#define ADC2_BASEADDR		(APB2PERIPH_BASEADDR + 0X2000)
#define ADC3_BASEADDR		(APB2PERIPH_BASEADDR + 0X2000)

#define SPI1_BASEADDR		(APB2PERIPH_BASEADDR + 0X3000)
#define SPI4_BASEADDR		(APB2PERIPH_BASEADDR + 0X3400)
#define SPI5_BASEADDR		(APB2PERIPH_BASEADDR + 0X5000)
#define SPI6_BASEADDR		(APB2PERIPH_BASEADDR + 0X5400)

#define SYSCFG_BASEADDR		(APB2PERIPH_BASEADDR + 0X3800)

#define USART1_BASEADDR		(APB2PERIPH_BASEADDR + 0X1000)
#define USART6_BASEADDR		(APB2PERIPH_BASEADDR + 0X1400)


/*
 *
*/
typedef struct
{
    __vo uint32_t SR;				/*|<TODO	        Address Offset     0x00    */
    __vo uint32_t DR;				/*|<TODO	        Address Offset     0x04    */
    __vo uint32_t BRR;				/*|<TODO	        Address Offset     0x08    */
    __vo uint32_t CR1;				/*|<TODO	        Address Offset     0x1c    */
    __vo uint32_t CR2;		    	/*|<TODO	        Address Offset     0x10    */
    __vo uint32_t CR3;		    	/*|<TODO	        Address Offset     0x14    */
	__vo uint32_t GPTR;		    	/*|<TODO	        Address Offset     0x18    */


}USART_Regdef_t;

typedef struct
{
    __vo uint32_t CR1;				/*|<TODO	        Address Offset     0x00    */
    __vo uint32_t CR2;				/*|<TODO	        Address Offset     0x04    */
    __vo uint32_t SR;				/*|<TODO	        Address Offset     0x08    */
    __vo uint32_t DR;				/*|<TODO	        Address Offset     0x1c    */
    __vo uint32_t CRCPR;					/*|<TODO	        Address Offset     0x10    */
    __vo uint32_t RXCRCR;			    	/*|<TODO	        Address Offset     0x14    */
	__vo uint32_t TXCRCR;					/*|<TODO	        Address Offset     0x18    */
    __vo uint32_t I2SCFGR;                	/*|<TODO	        Address Offset     0x2c    */
    __vo uint32_t I2SPR;				/*|<TODO	        Address Offset     0x20    */

}SPI_Regdef_t;

/*  structures for Definitions of GPIOxx   Peripheral Register
 *
 *  if You want to access the GPIOA registers then we have assign the base  address to pointer of type GPIO_Regdef_t
 *
 *  so we created another set of macros for the type casted addresses below
 *   */

typedef struct
{
    __vo uint32_t MODER;				/*|<TODO	        Address Offset     0x00    */
    __vo uint32_t OTYPER;				/*|<TODO	        Address Offset     0x04    */
    __vo uint32_t OSPEEDR;				/*|<TODO	        Address Offset     0x08    */
    __vo uint32_t PUPDR;				/*|<TODO	        Address Offset     0x1c    */
    __vo uint32_t IDR;					/*|<TODO	        Address Offset     0x10    */
    __vo uint32_t ODR;			    	/*|<TODO	        Address Offset     0x14    */
	__vo uint32_t BSRR;					/*|<TODO	        Address Offset     0x18    */
    __vo uint32_t LCKR;                	/*|<TODO	        Address Offset     0x2c    */
    __vo uint32_t AFR[2];				/*|<TODO	        Address Offset     0x20    */

}GPIO_Regdef_t;

 /* structures for Definitions of RCC Peripheral Register
  *
  * This structure will have all the rcc register
  *
  * we can access each these member elements through the pointer
  *
  * so user have to assign the required base address from the source
  *
  *
  *
  *  */

typedef struct
{

	__vo uint32_t CR;					/*|<TODO	        Address Offset     0x00    */
	__vo uint32_t PLLCFGR;              /*|<TODO	        Address Offset     0x04    */
	__vo uint32_t CFGR;                 /*|<TODO	        Address Offset     0x08    */
	__vo uint32_t CIR;                  /*|<TODO	        Address Offset     0x0C    */
	__vo uint32_t AHB1RSTR;             /*|<TODO	        Address Offset     0x0C    */
	__vo uint32_t AHB2RSTR;             /*|<TODO	        Address Offset     0x14    */
	__vo uint32_t AHB3RSTR;             /*|<TODO	        Address Offset     0x18    */
	     uint32_t Reserved;             /*|<TODO	        Address Offset     0x1C    */
	__vo uint32_t APB1RSTR;             /*|<TODO	        Address Offset     0x20    */
	__vo uint32_t APB2RSTR;             /*|<TODO	        Address Offset     0x24    */
	     uint32_t RESERVED1;            /*|<TODO	        Address Offset     0x28    */
	     uint32_t RESERVED2;            /*|<TODO	        Address Offset     0x2C   */
	__vo uint32_t AHB1ENR;              /*|<TODO	        Address Offset     0x30    */
	__vo uint32_t AHB2ENR;              /*|<TODO	        Address Offset     0x34    */
	__vo uint32_t AHB3ENR;              /*|<TODO	        Address Offset     0x38    */
	     uint32_t RESERVED3;            /*|<TODO	        Address Offset     0x3C    */
	__vo uint32_t APB1ENR;              /*|<TODO	        Address Offset     0x40    */
	__vo uint32_t APB2ENR;              /*|<TODO	        Address Offset     0x44    */
	     uint32_t RESERVED4;			/*|<TODO	        Address Offset     0x48    */
	     uint32_t RESERVED5;    		/*|<TODO	        Address Offset     0x4C    */
	__vo uint32_t AHB1LPENR;			/*|<TODO	        Address Offset     0x50    */
	__vo uint32_t AHB2LPENR;			/*|<TODO	        Address Offset     0x54    */
	__vo uint32_t AHB3LPENR;			/*|<TODO	        Address Offset     0x58    */
	     uint32_t RESERVED6;			/*|<TODO	        Address Offset     0x5C    */
	__vo uint32_t APB1LPENR;			/*|<TODO	        Address Offset     0x60    */
	__vo uint32_t APB2LPENR;			/*|<TODO	        Address Offset     0x64    */
	     uint32_t RESERVED7;			/*|<TODO	        Address Offset     0x68    */
	     uint32_t RESERVED8;			/*|<TODO	        Address Offset     0x6C    */
	__vo uint32_t BDCR;					/*|<TODO	        Address Offset     0x70    */
	__vo uint32_t CSR;					/*|<TODO	        Address Offset     0x74    */
	     uint32_t RESERVED9;			/*|<TODO	        Address Offset     0x78    */
	     uint32_t RESERVED10;			/*|<TODO	        Address Offset     0x7C    */
	__vo uint32_t SSCGR;				/*|<TODO	        Address Offset     0x80    */
	__vo uint32_t PLLI2SCFGR;			/*|<TODO	        Address Offset     0x84    */
	__vo uint32_t PLLSAICFGR;			/*|<TODO	        Address Offset     0x88    */
	__vo uint32_t DCKCFGR;				/*|<TODO	        Address Offset     0x8C    */
}RCC_Regdef_t;



/*  structures for Definitions of EXTI   Peripheral Register*/

typedef struct
{
    __vo uint32_t IMR;				/*|<TODO	        Address Offset     0x00    */
    __vo uint32_t EMR;				/*|<TODO	        Address Offset     0x04    */
    __vo uint32_t RTSR;				/*|<TODO	        Address Offset     0x08    */
    __vo uint32_t FTSR;				/*|<TODO	        Address Offset     0x1c    */
    __vo uint32_t SWIER;			/*|<TODO	        Address Offset     0x10    */
    __vo uint32_t PR;			    /*|<TODO	        Address Offset     0x14    */

}EXTI_Regdef_t;



/*  structures for Definitions of SYSCFG   Peripheral Register*/

typedef struct
{
    __vo uint32_t MEMRMP;			    /*|<TODO	        Address Offset     0x00    */
    __vo uint32_t PMC;				    /*|<TODO	        Address Offset     0x04    */
    __vo uint32_t EXTICR[4];			/*|<TODO	        Address Offset     0x08-0X14    */
         uint32_t RESERVED1[2] ;		/*|<TODO	        Address Offset     0x18    */
    __vo uint32_t CMPCR;				/*|<TODO	        Address Offset     0x20    */
         uint32_t RESERVED[2]	;		/*|<TODO	        Address Offset     0x24-28   */
    __vo uint32_t CFGR;			   	    /*|<TODO	        Address Offset     0x2C   */

}SYSCFG_Regdef_t;



#define SYSCFG  ((SYSCFG_Regdef_t *) SYSCFG_BASEADDR) /*  SYSCFG peripherals  definition type casted TO SYSCFG_Regdef_t*/

#define EXTI   		((EXTI_Regdef_t *) EXTI_BASEADDR)     /*  EXTI peripherals  definition type casted TO EXTI_Regdef_t*/


#define GPIOA  		((GPIO_Regdef_t*)GPIOA_BASEADDR)      /*  GPIOx peripheral definitions type casted TO GPIO_Regdef_t */
#define GPIOB		((GPIO_Regdef_t*)GPIOB_BASEADDR)
#define GPIOC		((GPIO_Regdef_t*)GPIOC_BASEADDR)
#define GPIOD		((GPIO_Regdef_t*)GPIOD_BASEADDR)
#define GPIOE		((GPIO_Regdef_t*)GPIOE_BASEADDR)
#define GPIOF		((GPIO_Regdef_t*)GPIOF_BASEADDR)
#define GPIOG		((GPIO_Regdef_t*)GPIOG_BASEADDR)
#define GPIOH		((GPIO_Regdef_t*)GPIOH_BASEADDR)
#define GPIOI		((GPIO_Regdef_t*)GPIOI_BASEADDR)

#define RCC    		 ((RCC_Regdef_t *) RCC_BASEADDR)           /* RCC peripheral definitions type casted TO GPIO_Regdef_t */


#define SPI1  		((SPI_Regdef_t*)SPI1_BASEADDR)         	 /*  SPIX peripheral definitions type casted TO GPIO_Regdef_t */
#define SPI2		((SPI_Regdef_t*)SPI2_BASEADDR)
#define SPI3		((SPI_Regdef_t*)SPI3_BASEADDR)
#define SPI4 		((SPI_Regdef_t*)SPI4_BASEADDR)
#define SPI5		((SPI_Regdef_t*)SPI5_BASEADDR)

#define USART1  	((USART_Regdef_t*)USART1_BASEADDR)
#define USART2  	((USART_Regdef_t*)USART2_BASEADDR)
#define USART3  	((USART_Regdef_t*)USART3_BASEADDR)
#define UART4  	    ((USART_Regdef_t*)UART4_BASEADDR)
#define UART5   	((USART_Regdef_t*)UART5_BASEADDR)
#define USART6  	((USART_Regdef_t*)USART6_BASEADDR)


/* clock enable macros for  GPIPx peripherals */

#define    			GPIOA_PCLK_EN()    	 (RCC->AHB1ENR |= (1<<0))	/*|<TODO	 ENABLE THE GPIOA PORT   */
#define				GPIOB_PCLK_EN()		 (RCC->AHB1ENR |= (1<<1))	/*|<TODO	 ENABLE THE GPIOA PORT   */
#define				GPIOC_PCLK_EN()		 (RCC->AHB1ENR |= (1<<2))	/*|<TODO	 ENABLE THE GPIOA PORT   */
#define				GPIOD_PCLK_EN() 	 (RCC->AHB1ENR |= (1<<3))	/*|<TODO	 ENABLE THE GPIOA PORT   */
#define				GPIOE_PCLK_EN()		 (RCC->AHB1ENR |= (1<<4))	/*|<TODO	 ENABLE THE GPIOA PORT   */
#define				GPIOF_PCLK_EN()		 (RCC->AHB1ENR |= (1<<5))	/*|<TODO	 ENABLE THE GPIOA PORT   */
#define				GPIOG_PCLK_EN()		 (RCC->AHB1ENR |= (1<<6))	/*|<TODO	 ENABLE THE GPIOA PORT   */
#define				GPIOH_PCLK_EN()		 (RCC->AHB1ENR |= (1<<7))	/*|<TODO	 ENABLE THE GPIOA PORT   */
#define				GPIOI_PCLK_EN()		 (RCC->AHB1ENR |= (1<<8))	/*|<TODO	 ENABLE THE GPIOA PORT   */


/* clock enable macros for  I2Cx peripherals */

#define   		    I2C1_PCLK_EN()  	 (RCC->APB1ENR |= (1<<21))	/*|<TODO ENABLE THE CLK FOR I2C1  PORT */
#define    		    I2C2_PCLK_EN()  	 (RCC->APB1ENR |= (1<<22))	/*|<TODO ENABLE THE CLK FOR I2C1  PORT */
#define     		I2C3_PCLK_EN()  	 (RCC->APB1ENR |= (1<<23))	/*|<TODO ENABLE THE CLK FOR I2C1  PORT */

/* clock enable macros for  SPIx peripherals */

#define     		SPI1_PCLK_EN()  	 (RCC->APB2ENR |= (1<<12))	/*|<TODO ENABLE THE CLK FOR SPI1  PORT */
#define    			SPI2_PCLK_EN()  	 (RCC->APB1ENR |= (1<<14))	/*|<TODO ENABLE THE CLK FOR SPI2  PORT */
#define     		SPI3_PCLK_EN()  	 (RCC->APB1ENR |= (1<<15))	/*|<TODO ENABLE THE CLK FOR SPI3  PORT */
#define     		SPI4_PCLK_EN()  	 (RCC->APB2ENR |= (1<<13))	/*|<TODO ENABLE THE CLK FOR SPI4  PORT */


/* clock enable macros for  UARTx peripherals */
#define    			UART4_PCLK_EN()  	 (RCC->APB1ENR |= (1<<19))	/*|<TODO ENABLE THE CLK FOR I2C1  PORT */
#define    			UART5_PCLK_EN()  	 (RCC->APB1ENR |= (1<<20))	/*|<TODO ENABLE THE CLK FOR I2C1  PORT */


/* clock enable macros for  USARTx peripherals */

#define    			USART1_PCLK_EN()  	 (RCC->APB2ENR |= (1<<4))	/*|<TODO ENABLE THE CLK FOR I2C1  PORT */
#define   		    USART2_PCLK_EN()  	 (RCC->APB1ENR |= (1<<17))	/*|<TODO ENABLE THE CLK FOR I2C1  PORT */
#define     		USART3_PCLK_EN()  	 (RCC->APB1ENR |= (1<<18))	/*|<TODO ENABLE THE CLK FOR I2C1  PORT */
#define     		USART6_PCLK_EN()  	 (RCC->APB2ENR |= (1<<5))	/*|<TODO ENABLE THE CLK FOR I2C1  PORT */

/* clock enable macros for  SYSCFGx peripherals */
#define     		SYSCFG_PCLK_EN()         (RCC->APB2ENR |= (1<<14))	/*|<TODO ENABLE THE CLK FOR SYSCFG  PORT */

/* clock DISABLE macros for  GPIOx peripherals */
#define				GPIOA_PCLK_DI()      (RCC->AHB1ENR &= ~(1<<0))	/*|<TODO	 DISABLE THE GPIOA PORT   */
#define				GPIOB_PCLK_DI()		 (RCC->AHB1ENR &= ~(1<<1))	/*|<TODO	 DISABLE THE GPIOA PORT   */
#define				GPIOC_PCLK_DI()		 (RCC->AHB1ENR &= ~(1<<2))	/*|<TODO	 DISABLE THE GPIOA PORT   */
#define				GPIOD_PCLK_DI() 	 (RCC->AHB1ENR &= ~(1<<3))	/*|<TODO	 DISABLE THE GPIOA PORT   */
#define				GPIOE_PCLK_DI()		 (RCC->AHB1ENR &= ~(1<<4))	/*|<TODO	 DISABLE THE GPIOA PORT   */
#define				GPIOF_PCLK_DI()		 (RCC->AHB1ENR &= ~(1<<5))	/*|<TODO	 DISABLE THE GPIOA PORT   */
#define				GPIOG_PCLK_DI()		 (RCC->AHB1ENR &= ~(1<<6))	/*|<TODO	 DISABLE THE GPIOA PORT   */
#define				GPIOH_PCLK_DI()		 (RCC->AHB1ENR &= ~(1<<7))	/*|<TODO	 DISABLE THE GPIOA PORT   */
#define				GPIOI_PCLK_DI()		 (RCC->AHB1ENR &= ~(1<<8))	/*|<TODO	 DISABLE THE GPIOA PORT   */

/* clock DISABLE macros for  I2Cx peripherals */

#define     		I2C1_PCLK_DI()  	 (RCC->APB1ENR &= ~ (1<<21))	/*|<TODO DISABLE THE CLK FOR I2C1  PORT */
#define     		I2C2_PCLK_DI()  	 (RCC->APB1ENR &= ~ (1<<22))	/*|<TODO DISABLE THE CLK FOR I2C1  PORT */
#define     		I2C3_PCLK_DI()  	 (RCC->APB1ENR &= ~ (1<<23))	/*|<TODO DISABLE THE CLK FOR I2C1  PORT */

/******************************************clock DISABLE macros for  SPIx peripherals *******************************************************/

#define     		SPI1_PCLK_DI()  	 (RCC->APB2ENR &= ~(1<<12))	/*|<TODO DISABLE THE CLK FOR I2C1  PORT */
#define     		SPI2_PCLK_DI()  	 (RCC->APB1ENR &= ~(1<<14))	/*|<TODO DISABLE THE CLK FOR I2C1  PORT */
#define     		SPI3_PCLK_DI()  	 (RCC->APB1ENR &= ~(1<<15))	/*|<TODO DISABLE THE CLK FOR I2C1  PORT */
#define     		SPI4_PCLK_DI()  	 (RCC->APB1ENR &= ~(1<<16))	/*|<TODO DISABLE THE CLK FOR I2C1  PORT */


/* clock Disable macros for  UARTx peripherals */
#define     		UART4_PCLK_DI()  	 (RCC->APB1ENR &= ~(1<<19))	/*|<TODO ENABLE THE CLK FOR I2C1  PORT */
#define     		UART5_PCLK_DI()  	 (RCC->APB1ENR &= ~(1<<20))	/*|<TODO ENABLE THE CLK FOR I2C1  PORT */


/* clock Disable macros for  USARTx peripherals */

#define     		USART1_PCLK_DI()  	 (RCC->APB2ENR &= ~(1<<4))	/*|<TODO ENABLE THE CLK FOR I2C1  PORT */
#define     		USART2_PCLK_DI()  	 (RCC->APB1ENR &= ~(1<<17))	/*|<TODO ENABLE THE CLK FOR I2C1  PORT */
#define     		USART3_PCLK_DI()  	 (RCC->APB1ENR &= ~(1<<18))	/*|<TODO ENABLE THE CLK FOR I2C1  PORT */
#define     		USART6_PCLK_DI()  	 (RCC->APB2ENR &= ~(1<<5))	/*|<TODO ENABLE THE CLK FOR I2C1  PORT */

/*
 * ****************************************macros to reset the gpiox peripherals***************************************************************
*/

#define				GPIOA_REG_RESET()	 do{(RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define				GPIOB_REG_RESET()	 do{(RCC->AHB1RSTR |= (1<<1)); (RCC->AHB1RSTR &= ~(1<<1));}while(0)
#define				GPIOC_REG_RESET()	 do{(RCC->AHB1RSTR |= (1<<2)); (RCC->AHB1RSTR &= ~(1<<2));}while(0)
#define				GPIOD_REG_RESET()	 do{(RCC->AHB1RSTR |= (1<<3)); (RCC->AHB1RSTR &= ~(1<<3));}while(0)
#define				GPIOE_REG_RESET()	 do{(RCC->AHB1RSTR |= (1<<4)); (RCC->AHB1RSTR &= ~(1<<4));}while(0)
#define				GPIOF_REG_RESET()	 do{(RCC->AHB1RSTR |= (1<<5)); (RCC->AHB1RSTR &= ~(1<<5));}while(0)
#define				GPIOG_REG_RESET()	 do{(RCC->AHB1RSTR |= (1<<6)); (RCC->AHB1RSTR &= ~(1<<6));}while(0)
#define				GPIOH_REG_RESET()	 do{(RCC->AHB1RSTR |= (1<<7)); (RCC->AHB1RSTR &= ~(1<<7));}while(0)
#define				GPIOI_REG_RESET()	 do{(RCC->AHB1RSTR |= (1<<8)); (RCC->AHB1RSTR &= ~(1<<8));}while(0)

/******************************************macros to reset the Spi peripheras******************************************************************/
#define				SPI1_REG_RESET()	 do{(RCC->APB2RSTR |= (1<<12));(RCC->APB2RSTR &= ~(1<<12));}while(0)
#define				SPI2_REG_RESET()	 do{(RCC->APB1RSTR |= (1<<14));(RCC->APB1RSTR &= ~(1<<14));}while(0)
#define				SPI3_REG_RESET()	 do{(RCC->APB1RSTR |= (1<<15));(RCC->APB1RSTR &= ~(1<<15));}while(0)
#define				SPI4_REG_RESET()	 do{(RCC->APB2RSTR |= (1<<13));(RCC->APB2RSTR &= ~(1<<13));}while(0)

/******************************************macros to reset the uart peripheras******************************************************************/
#define				UASRT1_REG_RESET()	 do{(RCC->APB2RSTR |= (1<<4)); (RCC->APB2RSTR &= ~(1<<4));}while(0)
#define				UART2_REG_RESET()	 do{(RCC->APB1RSTR |= (1<<17));(RCC->APB1RSTR &= ~(1<<17));}while(0)
#define				UART3_REG_RESET()	 do{(RCC->APB1RSTR |= (1<<18));(RCC->APB1RSTR &= ~(1<<18));}while(0)
#define				UART4_REG_RESET()	 do{(RCC->APB2RSTR |= (1<<19));(RCC->APB2RSTR &= ~(1<<19));}while(0)
#define				UART5_REG_RESET()	 do{(RCC->APB2RSTR |= (1<<20));(RCC->APB2RSTR &= ~(1<<20));}while(0)
#define				USART6_REG_RESET()	 do{(RCC->APB2RSTR |= (1<<5)); (RCC->APB2RSTR &= ~(1<<5));}while(0)
/*********************************** clock DISABLE macros for  SYSCFG  peripherals**************************************************************/

#define     		SYSCFG_PCLK_DI()         (RCC->APB2ENR &= ~(1<<14))	/*|<TODO DISABLE THE CLK FOR SYSCFG  PORT */
#define     GPIOX_BASEEADDR_TO_CODE(X)			((X==GPIOA)?0:\
											    (X==GPIOB)?1:\
											    (X==GPIOC)?2:\
										        (X==GPIOD)?3:\
											    (X==GPIOE)?4:\
											    (X==GPIOF)?5:\
											    (X==GPIOG)?6:\
											    (X==GPIOH)?7:\
											    (X==GPIOI)?8:0)//TO SELECT THE PORT FOR EXTI CR

/*
 * ********************************************IRQ numbers of STM32F407VG************************************************************************
*/
/*
 * IRQ no for the GPIOs
*/



#define	IRQ_NO_EXTI0						6
#define	IRQ_NO_EXTI1						7
#define	IRQ_NO_EXTI2                        8
#define	IRQ_NO_EXTI3						9
#define	IRQ_NO_EXTI4						10
#define	IRQ_NO_EXTI9_5						23
#define	IRQ_NO_EXTI15_10					40

/*
 IRQ nos for the SPIx
*/

#define IRQ_NO_SPI1						    35
#define IRQ_NO_SPI2							36
#define IRQ_NO_SPI3							51

/*
 IRQ nos for the USARTx
*/

#define IRQ_NO_USART1						44
#define IRQ_NO_USART2						45
#define IRQ_NO_UART4						59
#define IRQ_NO_UART5						60
#define IRQ_NO_USART6						78

/*
macros for the interrupt priorities
*/

#define IRQ_NO_PRI0							0
#define IRQ_NO_PRI1							1
#define IRQ_NO_PRI2							2
#define IRQ_NO_PRI3							3
#define IRQ_NO_PRI4							4
#define IRQ_NO_PRI5							5
#define IRQ_NO_PRI6							6
#define IRQ_NO_PRI7							7
#define IRQ_NO_PRI8							8
#define IRQ_NO_PRI9							9
#define IRQ_NO_PRI10					    10
#define IRQ_NO_PRI11						11
#define IRQ_NO_PRI12						12
#define IRQ_NO_PRI13						13
#define IRQ_NO_PRI14						14
#define IRQ_NO_PRI15						15

/****************************************************generic macros*******************************************************************************/

#define ENABLE	 		 1
#define DISABLE			 0
#define SET				 ENABLE
#define RESET			 DISABLE
#define GPIO_PIN_SET	 SET
#define GPIO_PIN_RESET	 RESET
#define FLAG_SET  		 SET
#define FLAG_RESET		 RESET
#define __weak           __attribute__((weak))
/*
***************************************************************************************************************************************************
****************************************************Bit Position Macros For SPI Peripheral*********************************************************
*/

/*
*BIT Position Definitions for SPI_CR1 Regidster
*/
#define	SPI_CR1_CPHA	 0
#define	SPI_CR1_CPOL     1
#define	SPI_CR1_MSTR   	 2
#define	SPI_CR1_BR     	 3
#define	SPI_CR1_SPE		 6
#define	SPI_CR1_LSBFIRST 7
#define	SPI_CR1_SSI		 8
#define	SPI_CR1_SSM      9
#define	SPI_CR1_RXONLY   10
#define	SPI_CR1_DFF		 11
#define	SPI_CR1_CRCNEXT  12
#define	SPI_CR1_CRCEN	 13
#define	SPI_CR1_BIDIOE   14
#define	SPI_CR1_BIDIMODE 15

/*
*BIT Position Definitions for SPI_CR2 Regidster
*/

#define	SPI_CR2_RXDMAEN		0
#define	SPI_CR2_TXDMAEN		1
#define	SPI_CR2_SSOE		2
#define	SPI_CR2_FRF			4
#define	SPI_CR2_ERRIE		5
#define	SPI_CR2_RXNEIE		6
#define	SPI_CR2_TXEIE		7



/*
*BIT Position Definitions for SPI_SR Regidster
*/
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8

/******************************************************************************************
 *Bit position definitions of USART peripheral
 ******************************************************************************************/

/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_SBK					0
#define USART_CR1_RWU 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M 					12
#define USART_CR1_UE 					13
#define USART_CR1_OVER8  				15



/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14


/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11

/*
 * Bit position definitions USART_SR
 */

#define USART_SR_PE        				0
#define USART_SR_FE        				1
#define USART_SR_NE        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_LBD        			8
#define USART_SR_CTS        			9


//SPI_FLAGS

#define SPI_READY       0
#define SPI_BUSY_IN_RX  1
#define SPI_BUSY_IN_TX  2

//spi_evnt_flag

#define SPI_EVENT_TX_CMPLT	1
#define	SPI_EVENT_RX_CMPLT	2
#define	SPI_EVENT_OVR_ERR	3
#define SPI_EVENT_CRC_ERR	4

#include "stm32f407xx_rcc.h"
#include "stm32f407xx_usart.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_gpio_driver.h"

#endif /* INC_STM32F407XX_H_ */
