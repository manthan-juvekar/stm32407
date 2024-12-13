/*
 * stm32f407xx.h
 *
 *  Created on: Dec 13, 2024
 *      Author: manthan juvekar
 */


#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include<stdint.h>
// file macros
#define __vo        volatile
/*
 * base addresses of Flash and SRAM memories
 */
#define FLASH_BASEADDR				0x08000000U
#define SRAM1_BASEADDR          	0x20000000U
#define SRAM2_BASEADDR          	0x2001C000U
#define ROM                     	0x1FFF0000
#define SRAM 				    	SRAM1_BASEADDR


/*
 * peripherals buses base addresses (AHBx , APBx)
 */
#define PERIPH_BASEADDR             0x40000000U
#define AHB1PERIPH_BASEADDR			0x40020000U
#define AHB2PERIPH_BASEADDR         0x50000000U
#define APB1PERIPH_BASEADDR         PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR         0x40010000U

/*
 * rcc base addresses
 */
#define RCC_BASEADDR      (AHB2PERIPH_BASEADDR + 3800)

/*
 * base addresses of AHB1 bus peripherals
 */
#define GPIOA_BASEADDR              (AHB1PERIPH_BASEADDR +0X0000)
#define GPIOB_BASEADDR   			(AHB1PERIPH_BASEADDR +0X0400)
#define GPIOC_BASEADDR				(AHB1PERIPH_BASEADDR +0X0800)
#define GPIOD_BASEADDR				(AHB1PERIPH_BASEADDR +0X0C00)
#define GPIOE_BASEADDR				(AHB1PERIPH_BASEADDR +0X1000)
#define GPIOF_BASEADDR				(AHB1PERIPH_BASEADDR +0X1400)
#define GPIOG_BASEADDR				(AHB1PERIPH_BASEADDR +0X1800)
#define GPIOH_BASEADDR				(AHB1PERIPH_BASEADDR +0X1C00)
#define GPIOI_BASEADDR				(AHB1PERIPH_BASEADDR +0X2000)

/*
 * base addresses of APB1 bus peripherals
 */
#define I2C1_BASEADDR                (APB1PERIPH_BASEADDR +0X5400)
#define I2C2_BASEADDR                (APB1PERIPH_BASEADDR +0X5800)
#define I2C3_BASEADDR                (APB1PERIPH_BASEADDR +0X5C00)
#define SPI2_BASEADDR                (APB1PERIPH_BASEADDR +0X3800)
#define SPI3_BASEADDR                (APB1PERIPH_BASEADDR +0X3C00)
#define USART2_BASEADDR              (APB1PERIPH_BASEADDR +0x4400)
#define USART3_BASEADDR              (APB1PERIPH_BASEADDR +0x4800)
#define UART4_BASEADDR               (APB1PERIPH_BASEADDR +0x4C00)
#define UART5_BASEADDR               (APB1PERIPH_BASEADDR +0x5000)

/*
 * base addresses of APB2 bus peripherals
 */

#define SPI1_BASEADDR                 (APB2PERIPH_BASEADDR +0x3000)
#define USART1_BASEADDR               (APB2PERIPH_BASEADDR +0x1000)
#define USART6_BASEADDR               (APB2PERIPH_BASEADDR +0x1400)
#define EXTI_BASEADDR                 (APB2PERIPH_BASEADDR +0x3C00)
#define SYSCFG_BASEADDR               (APB2PERIPH_BASEADDR +0x3800)

#endif /* INC_STM32F407XX_H_ */




/*
 * GPIOx register structure
*/
typedef struct{
	__vo uint32_t MODER;                 //GPIO port mode register                 (Address offset: 0x00)
	__vo uint32_t OTYPER;                //GPIO port output type register          (Address offset: 0x04)
	__vo uint32_t OSPEEDR; 			     //GPIO port output speed register         (Address offset: 0x08)
	__vo uint32_t PUPDR;                 //GPIO port pull-up/pull-down register    (Address offset: 0x0C)
	__vo uint32_t IDR;                   //GPIO port input data register           (Address offset: 0x10)
	__vo uint32_t ODR;                   //GPIO port output data register          (Address offset: 0x14)
	__vo uint32_t BSRR;                  //GPIO port bit set/reset register        (Address offset: 0x18)
	__vo uint32_t LCKR;                  //GPIO port configuration lock register   (Address offset: 0x1C)
	__vo uint32_t AFR[2];                /*AFRL and AFRH register
	                                     *GPIO alternate function registers        (Address offsets: 0x20-0x24)
	                                     */
}GPIO_RegDef_t;

/*
 * peripheral definitions (peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA                 ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB                 ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC                 ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD                 ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE                 ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF                 ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG                 ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH                 ((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI                 ((GPIO_RegDef_t*)GPIOI_BASEADDR)

/*
 * RCC register structure
 */
typedef struct{
	__vo uint32_t CR;                                    		// (Address offset:0x00 )
	__vo uint32_t  PLLCFGR;                                     // (Address offset: 0x04)
	__vo uint32_t CFGR ;                                        // (Address offset:0x08 )
	__vo uint32_t CIR ;                                         // (Address offset:0x0C)
	__vo uint32_t AHB1RSTR ;                                    // (Address offset: 0x10)
	__vo uint32_t AHB2RSTR  ;                                   // (Address offset:0x14 )
	__vo uint32_t  AHB3RSTR;                                    // (Address offset: 0x18)

	__vo uint32_t Reserved0 ;                                   // (Address offset:0x1C )
	__vo uint32_t APB1RSTR ;                                    // (Address offset:0x20 )
	__vo uint32_t APB2RSTR ;                                    // (Address offset:0x24 )

	__vo uint32_t Reserved1 ;                                   // (Address offset:0x28 )
	__vo uint32_t Reserved2 ;                                   // (Address offset:0x2C )
	__vo uint32_t AHB1ENR ;                                     // (Address offset:0x30 )
	__vo uint32_t AHB2ENR ;                                     // (Address offset:0x34 )
	__vo uint32_t AHB3ENR ;                                     // (Address offset: 0x38)

	__vo uint32_t Reserved3 ;                                   // (Address offset:0x3C )
	__vo uint32_t APB1ENR ;                                     // (Address offset:0x40 )
	__vo uint32_t APB2ENR ;                                     // (Address offset: 0x44)

	__vo uint32_t Reserved4 ;                                   // (Address offset:0x48 )
	__vo uint32_t  Reserved5;                                   // (Address offset:0x4C )
	__vo uint32_t AHB1LPENR ;                                   // (Address offset:0x50 )
	__vo uint32_t AHB2LPENR ;                                   // (Address offset: 0x54)
	__vo uint32_t AHB3LPENR ;                                   // (Address offset:0x58 )

	__vo uint32_t Reserved6 ;                                   // (Address offset:0x5C )
	__vo uint32_t  APB1LPENR;                                   // (Address offset:0x60)
	__vo uint32_t  APB2LPENR;                                   // (Address offset: 0x64)

	__vo uint32_t Reserved7 ;                                   // (Address offset:0x68 )
	__vo uint32_t Reserved8 ;                                   // (Address offset:0x6C )
	__vo uint32_t BDCR ;                                        // (Address offset: 0x70)

}RCC_RegDef_t;

// RCC register defination
#define RCC           ((RCC_RegDef_t*)RCC_BASEADDR)

//GPIOx clock enable macros
#define GPIOA_PCLK_EN()        ( RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()        ( RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()        ( RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()        ( RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()        ( RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN()        ( RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN()        ( RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN()        ( RCC->AHB1ENR |= (1<<7))
#define GPIOI_PCLK_EN()        ( RCC->AHB1ENR |= (1<<8))

//I2Cx clock enable

#define I2C1_PCLK_EN()          (RCC->APB1ENR |=(1<<21))
#define I2C2_PCLK_EN()          (RCC->APB1ENR |=(1<<22))
#define I2C3_PCLK_EN()          (RCC->APB1ENR |=(1<<23))

//SPIx clock enable
#define SPI1_PCLK_EN()          (RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()          (RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()          (RCC->APB1ENR |= (1<<15))

//UART clock enable

#define USART1_PCLK_EN()        (RCC->APB2ENR |= (1<<4))
#define USART2_PCLK_EN()        (RCC->APB1ENR |= (1<<17))
#define USART3_PCLK_EN()        (RCC->APB2ENR |= (1<<18))
#define UART4_PCLK_EN()        (RCC->APB2ENR |= (1<<19))
#define UART5_PCLK_EN()        (RCC->APB2ENR |= (1<<20)
#define USART6_PCLK_EN()        (RCC->APB2ENR |= (1<<5))


//SYSCFG clock enable
#define SYSCFG_PCLK_EN()        (RCC->APB2ENR |= (1<<14))

//GPIOx clock disable
#define GPIOA_PCLK_DI()        ( RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()        ( RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()        ( RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()        ( RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()        ( RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DI()        ( RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DI()        ( RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DI()        ( RCC->AHB1ENR &= ~(1<<7))
#define GPIOI_PCLK_DI()        ( RCC->AHB1ENR &= ~(1<<8))

//I2C clock disable
#define I2C1_PCLK_DI()          (RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI()          (RCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI()          (RCC->APB1ENR &= ~(1<<23))

//SPIx clock disable
#define SPI1_PCLK_DI()          (RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI()          (RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI()          (RCC->APB1ENR &= ~(1<<15))

//UART clock disable

#define USART1_PCLK_DI()        (RCC->APB2ENR &= ~(1<<4))
#define USART2_PCLK_DI()        (RCC->APB1ENR &= ~(1<<17))
#define USART3_PCLK_DI()        (RCC->APB2ENR &= ~(1<<18))
#define UART4_PCLK_DI()         (RCC->APB2ENR &= ~(1<<19))
#define UART5_PCLK_DI()         (RCC->APB2ENR &= ~(1<<20)
#define USART6_PCLK_DI()        (RCC->APB2ENR &= ~(1<<5))



//SYSCFG clock enable
#define SYSCFG_PCLK_DI()        (RCC->APB2ENR &= ~(1<<14))

