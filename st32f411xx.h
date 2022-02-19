/*
 * st32f411xx.h
 *
 *  Created on: Feb 5, 2022
 *      Author: Ekin
 */

#ifndef INC_ST32F411XX_H_
#define INC_ST32F411XX_H_

#include <stdint.h>

#define FLASH_BASEADDR 					0x08000000U
#define SRAM1_BASEADDR 					0x20000000U
#define ROM_BASEADDR					0x1FFF0000U
#define SRAM 							SRAM1_BASEADDR

#define PERIPHERAL_BASEADDR				0x40000000U
#define APB1_PERIPHERAL_BASE			PERIPHERAL_BASEADDR
#define APB2_PERIPHERAL_BASE			0x40010000U
#define AHB1_PERIPHERAL_BASE			0x40020000U
#define AHB2_PERIPHERAL_BASE			0x50000000U

#define GPIOA_BASEADDR 					0x40020000U
#define GPIOB_BASEADDR 					0x40020400U
#define GPIOC_BASEADDR 					0x40020800U
#define GPIOD_BASEADDR 					0x40020C00U
#define GPIOE_BASEADDR 					0x40021000U
#define GPIOH_BASEADDR 					0x40021C00U
#define RCC_BASEADDR					(AHB1_PERIPHERAL_BASE+ 0x3800)
#define SYSCFG_BASEADDR					0x40013800U

#define I2C3_BASEADDR					(APB1_PERIPHERAL_BASE+ 0x5C00)
#define I2C2_BASEADDR					(APB1_PERIPHERAL_BASE+ 0x5800)
#define I2C1_BASEADDR					(APB1_PERIPHERAL_BASE+ 0x5400)
#define USART2_BASEADDR					(APB1_PERIPHERAL_BASE+ 0x4400)
#define I2C3ext_BASEADDR				(APB1_PERIPHERAL_BASE+ 0x4000)
#define SPI3_BASEADDR					(APB1_PERIPHERAL_BASE+ 0x3C00)
#define SPI2_BASEADDR					(APB1_PERIPHERAL_BASE+ 0x3800)
#define I2C2ext_BASEADDR				(APB1_PERIPHERAL_BASE+ 0x3400)

#define SPI5_BASEADDR					(APB2_PERIPHERAL_BASE+ 0x5000)
#define EXTI_BASEADDR					(APB2_PERIPHERAL_BASE+ 0x3C00)
#define SPI1_BASEADDR					(APB2_PERIPHERAL_BASE+ 0x3000)
#define SPI4_BASEADDR					(APB2_PERIPHERAL_BASE+ 0x3400)
#define USART6_BASEADDR					(APB2_PERIPHERAL_BASE+ 0x1400)
#define USART1_BASEADDR					(APB2_PERIPHERAL_BASE+ 0x1000)

typedef struct{ /* Peripheral register definition for GPIOX*/
	volatile uint32_t MODER; 					/*GPIO port mode register				*/
	volatile uint32_t OTYPER;					/*GPIO port output type register		*/
	volatile uint32_t OSPEEDR;					/*GPIO port output speed register		*/
	volatile uint32_t PUPDR;						/*GPIO port pull-up/pull-down register	*/
	volatile uint32_t IDR;						/*GPIO port input data register			*/
	volatile uint32_t ODR;						/*GPIO port output data register		*/
	volatile uint32_t BSRR;						/*GPIO port bit set/reset register		*/
	volatile uint32_t LCKR;						/*GPIO port configuration lock register	*/
	volatile uint32_t AFR[2]; 					/*GPIO alternate function register		*/
}GPIO_RegDef_t;

#define GPIO_A 							((GPIO_RegDef_t*)GPIOA_BASEADDR);
#define GPIO_B  						((GPIO_RegDef_t*)GPIOB_BASEADDR);
#define GPIO_C  						((GPIO_RegDef_t*)GPIOC_BASEADDR);
#define GPIO_D  						((GPIO_RegDef_t*)GPIOD_BASEADDR);
#define GPIO_E  						((GPIO_RegDef_t*)GPIOE_BASEADDR);
#define GPIO_H  						((GPIO_RegDef_t*)GPIOH_BASEADDR);

typedef struct{
	volatile uint32_t CR;						/*RCC clock control register (RCC_CR)	*/
	volatile uint32_t PLLCFGR;					/*RCC PLL configuration register 		*/
	volatile uint32_t CFGR;						/*RCC clock configuration register 		*/
	volatile uint32_t CIR;						/*RCC clock interrupt register 			*/
	volatile uint32_t AHB1RSTR;					/*RCC AHB1 peripheral reset register 	*/
	volatile uint32_t AHB2RSTR;					/*RCC AHB2 peripheral reset register 	*/
	uint32_t RESERVED0[2];
	volatile uint32_t APB1RSTR;					/*RCC APB1 peripheral reset register 	*/
	volatile uint32_t APB2RSTR;					/*RCC APB2 peripheral reset register 	*/
	uint32_t RESERVED1[2];
	volatile uint32_t AHB1ENR;					/*RCC AHB1 peripheral clock enable register 	*/
	volatile uint32_t AHB2ENR;					/*RCC AHB2 peripheral clock enable register 	*/
	uint32_t RESERVED2[2];
	volatile uint32_t APB1ENR;					/*RCC APB1 peripheral clock enable register 	*/
	volatile uint32_t APB2ENR;					/*RCC APB2 peripheral clock enable register 	*/
	uint32_t RESERVED3[2];
	volatile uint32_t AHB1LPENR;					/*RCC AHB1 peripheral clock enable in low power mode register*/
	volatile uint32_t AHB2LPENR;					/*RCC AHB2 peripheral clock enable in low power mode register*/
	uint32_t RESERVED4[2];
	volatile uint32_t APB1LPENR;					/*RCC APB1 peripheral clock enable in low power mode register*/
	volatile uint32_t APB2LPENR;					/*RCC APB1 peripheral clock enable in low power mode register*/
	uint32_t RESERVED5[2];
	volatile uint32_t BDCR;						/*RCC Backup domain control register 	*/
	volatile uint32_t CSR;						/*RCC clock control & status register 	*/
	uint32_t RESERVED6[2];
	volatile uint32_t SSCGR;						/*RCC spread spectrum clock generation register	*/
	volatile uint32_t PLLI2SCFGR;				/*RCC PLLI2S configuration register 	*/
	volatile uint32_t RESERVED7;
	volatile uint32_t DCKCFGR;					/*RCC Dedicated Clocks Configuration Register 	*/
}RCC_RegDef_t;

#define RCC 							((RCC_RegDef_t*)RCC_BASEADDR)

#define GPIOA_PCLK_EN()					(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()					(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()					(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()					(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()					(RCC->AHB1ENR |= (1 << 4))
#define GPIOH_PCLK_EN()					(RCC->AHB1ENR |= (1 << 7))

#define I2C3_PCLK_EN()					(RCC->APB1ENR |= (1 << 23))
#define I2C2_PCLK_EN()					(RCC->APB1ENR |= (1 << 22))
#define I2C1_PCLK_EN()					(RCC->APB1ENR |= (1 << 21))
#define USART2_PCLK_EN()				(RCC->APB1ENR |= (1 << 17))
#define SPI3_PCLK_EN()					(RCC->APB1ENR |= (1 << 15))
#define SPI2_PCLK_EN()					(RCC->APB1ENR |= (1 << 14))

#define SPI5_PCLK_EN()					(RCC->APB2ENR |= (1 << 20))
#define SYSCFG_PCLK_EN()				(RCC->APB2ENR |= (1 << 14))
#define SPI4_PCLK_EN()					(RCC->APB2ENR |= (1 << 13))
#define SPI1_PCLK_EN()					(RCC->APB2ENR |= (1 << 12))
#define USART6_PCLK_EN()				(RCC->APB2ENR |= (1 << 5))
#define USART1_PCLK_EN()				(RCC->APB2ENR |= (1 << 4))


#define GPIOA_PCLK_DI()					(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()					(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()					(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()					(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()					(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOH_PCLK_DI()					(RCC->AHB1ENR &= ~(1 << 7))

#define I2C3_PCLK_DI()					(RCC->APB1ENR &= ~(1 << 23))
#define I2C2_PCLK_DI()					(RCC->APB1ENR &= ~(1 << 22))
#define I2C1_PCLK_DI()					(RCC->APB1ENR &= ~(1 << 21))
#define USART2_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 17))
#define SPI3_PCLK_DI()					(RCC->APB1ENR &= ~(1 << 15))
#define SPI2_PCLK_DI()					(RCC->APB1ENR &= ~(1 << 14))

#define SPI5_PCLK_DI()					(RCC->APB2ENR &= ~(1 << 20))
#define SYSCFG_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 14))
#define SPI4_PCLK_DI()					(RCC->APB2ENR &= ~(1 << 13))
#define SPI1_PCLK_DI()					(RCC->APB2ENR &= ~(1 << 12))
#define USART6_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 5))
#define USART1_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 4))

typedef struct{
	volatile uint32_t IMR;				/*Interrupt Mask Register			*/
	volatile uint32_t EMR;				/*Event Mask Register				*/
	volatile uint32_t RTSR;				/*Rising Trigger Selection Register	*/
	volatile uint32_t FTSR;				/*Falling Trigger Selection Register*/
	volatile uint32_t SWIER;			/*Software Interrupt Event Register	*/
	volatile uint32_t PR;				/*Pending Register					*/
}EXTI_RegDef_t;

#define EXTI 							((EXTI_RegDef_t*)EXTI_BASEADDR)

#define GPIOA_REG_RESET()				do{(RCC->AHB1RSTR |= (0b1 << 0));(RCC->AHB1RSTR &= ~(0b1 << 0));}while(0)
#define GPIOB_REG_RESET()				do{(RCC->AHB1RSTR |= (0b1 << 1));(RCC->AHB1RSTR &= ~(0b1 << 1));}while(0)
#define GPIOC_REG_RESET()				do{(RCC->AHB1RSTR |= (0b1 << 2));(RCC->AHB1RSTR &= ~(0b1 << 2));}while(0)
#define GPIOD_REG_RESET()				do{(RCC->AHB1RSTR |= (0b1 << 3));(RCC->AHB1RSTR &= ~(0b1 << 3));}while(0)
#define GPIOE_REG_RESET()				do{(RCC->AHB1RSTR |= (0b1 << 4));(RCC->AHB1RSTR &= ~(0b1 << 4));}while(0)
#define GPIOH_REG_RESET()				do{(RCC->AHB1RSTR |= (0b1 << 7));(RCC->AHB1RSTR &= ~(0b1 << 7));}while(0)


typedef struct{
	volatile uint32_t MEMRMP;			/*memory remap register									*/
	volatile uint32_t PMC;				/*peripheral mode configuration register				*/
	volatile uint32_t EXTICR[4];		/*external interrupt configuration register				*/
	uint32_t RESERVED[2];
	volatile uint32_t CMPCR;			/*Compensation cell control register					*/
}SYSCFG_RegDef_t;

#define SYSCFG 							((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define NVIC_BASEADDR					0xE000E000U
#define NVIC_ISER_BASEADDR				(NVIC_BASEADDR+0x100)
#define NVIC_ICER_BASEADDR				(NVIC_BASEADDR+0x180)
#define NVIC_ISPR_BASEADDR				(NVIC_BASEADDR+0x200)
#define NVIC_ICPR_BASEADDR				(NVIC_BASEADDR+0x280)
#define NVIC_IABR_BASEADDR				(NVIC_BASEADDR+0x300)
#define NVIC_IPR_BASEADDR				(NVIC_BASEADDR+0x400)
#define NVIC_STIR_BASEADDR				(NVIC_BASEADDR+0xE00)

#endif /* INC_ST32F411XX_H_ */
