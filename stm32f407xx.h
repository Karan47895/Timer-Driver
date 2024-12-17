/*
 * stm32f407xx.h
 *
 *  Created on: Jun 2, 2024
 *      Author: Karan Patel
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_
#include<stddef.h>
#include<stdint.h>
#include<stdio.h>



#define __vo volatile
#define __weak __attribute__((weak))

// ARM cortex Mx processor NVIC ISERx register addresses.

#define NVIC_ISER0       ((__vo uint32_t*) 0XE000E100)
#define NVIC_ISER1       ((__vo uint32_t*) 0XE000E104)
#define NVIC_ISER2       ((__vo uint32_t*) 0XE000E108)
#define NVIC_ISER3       ((__vo uint32_t*) 0XE000E10C)

// ARM cortex Mx processor NVIC ICERx register addresses.

#define NVIC_ICER0       ((__vo uint32_t*) 0XE000E180)
#define NVIC_ICER1       ((__vo uint32_t*) 0XE000E184)
#define NVIC_ICER2       ((__vo uint32_t*) 0XE000E188)
#define NVIC_ICER3       ((__vo uint32_t*) 0XE000E18C)


// ARM cortex Mx processor priority register address calculation.
#define NVIC_PR_BASE_ADDR ((__vo uint32_t*)0XE000E400)

#define NO_PR_BITS_IMPLEMENTED  4
// Base address of flash and SRAM memories.

#define FLASH_BASEADDR           0x08000000U  // base address for flash memory
#define SRAM1_BASEADDR           0x20000000U  // base address for SRAM1
#define SRAM2_BASEADDR           0X2001C000U  // base address for SRAM2
#define ROM_BASEADDR             0X1FFF0000U  // base address for ROM
#define SRAM                     SRAM1_BASEADDR



// Base address of ahbx and apbx bus peripheral devices.

#define SYSTICK_BASE              (0xE000E000UL + 0x0010UL)
#define PERIPH_BASE               0x40000000U
#define APB1PERIPH_BASEADDR       PERIPH_BASE
#define APB2PERIPH_BASEADDR       0x40010000U
#define AHB1PERIPH_BASEADDR       0x40020000U
#define AHB2PERIPH_BASEADDR       0x50000000U
#define SysTick                  ((SysTick_RegDef_t*)SYSTICK_BASE)


//Base address of peripherals which are hanging on AHB1 bus.



#define GPIOA_BASEADDR             (AHB1PERIPH_BASEADDR+0x0000)
#define GPIOB_BASEADDR             (AHB1PERIPH_BASEADDR+0x0400)
#define GPIOC_BASEADDR             (AHB1PERIPH_BASEADDR+0x0800)
#define GPIOD_BASEADDR             (AHB1PERIPH_BASEADDR+0x0C00)
#define GPIOE_BASEADDR             (AHB1PERIPH_BASEADDR+0x1000)
#define GPIOF_BASEADDR             (AHB1PERIPH_BASEADDR+0x1400)
#define GPIOG_BASEADDR             (AHB1PERIPH_BASEADDR+0x1800)
#define GPIOH_BASEADDR             (AHB1PERIPH_BASEADDR+0x1C00)
#define GPIOI_BASEADDR             (AHB1PERIPH_BASEADDR+0x2000)
#define GPIOJ_BASEADDR             (AHB1PERIPH_BASEADDR+0x2400)
#define GPIOK_BASEADDR             (AHB1PERIPH_BASEADDR+0x2800)
#define RCC_BASEADDR               (AHB1PERIPH_BASEADDR+0x3800)

//Base address of peripherals which are hanging on AHB1 bus.


#define I2C1_BASEADDR                (APB1PERIPH_BASEADDR+0x5400)
#define I2C2_BASEADDR                (APB1PERIPH_BASEADDR+0x5800)
#define I2C3_BASEADDR                (APB1PERIPH_BASEADDR+0x5C00)
#define SPI2_BASEADDR                (APB1PERIPH_BASEADDR+0x3800)
#define SPI3_BASEADDR                (APB1PERIPH_BASEADDR+0x3C00)
#define USART2_BASEADDR              (APB1PERIPH_BASEADDR+0x4400)
#define USART3_BASEADDR              (APB1PERIPH_BASEADDR+0x4800)
#define UART4_BASEADDR               (APB1PERIPH_BASEADDR+0x4C00)
#define UART5_BASEADDR               (APB1PERIPH_BASEADDR+0x5000)


//Base address of peripherals which are hanging on APB2 bus.


#define EXTI_BASEADDR                  (APB2PERIPH_BASEADDR+0x3C00)
#define SPI1_BASEADDR                  (APB2PERIPH_BASEADDR+0x3000)
#define USART1_BASEADDR                (APB2PERIPH_BASEADDR+0x1000)
#define USART6_BASEADDR                (APB2PERIPH_BASEADDR+0x1400)
#define SYSCFG_BASEADDR                (APB2PERIPH_BASEADDR+0x3800)



#define  ADC1_BASEADDR                 (APB2PERIPH_BASEADDR+0x2000)
#define  ADC2_BASEADDR                 (APB2PERIPH_BASEADDR+0x2100)
#define  ADC3_BASEADDR                 (APB2PERIPH_BASEADDR+0x2200)


#define ADC1                           ((ADC_RegDef_t*)ADC1_BASEADDR)
#define ADC2                           ((ADC_RegDef_t*)ADC2_BASEADDR)
#define ADC3                           ((ADC_RegDef_t*)ADC3_BASEADDR)

// Peripherals register defination structures

typedef struct{

	__vo uint32_t MODER;  //GPIO port mode register.
	__vo uint32_t OTYPER; //GPIO port output type register
	__vo uint32_t OSPEEDR; //GPIO port output speed register
	__vo uint32_t PUPDR;  //GPIO port pull-up/pull-down register
	__vo uint32_t IDR;    //GPIO port input data register
	__vo uint32_t ODR;    //GPIO port output data register
	__vo uint32_t BSRR;   //GPIO port bit set/reset register
	__vo uint32_t LCKR;   //GPIO port configuration lock register
	__vo uint32_t AFR[2]; // AFR[0]:GPIO alternate function low register,AFR[1]:GPIO alternate function high register

}GPIO_RegDef_t;



typedef struct
{
  __vo uint32_t CTRL;                   /*!< Offset: 0x000 (R/W)  SysTick Control and Status Register */
  __vo uint32_t LOAD;                   /*!< Offset: 0x004 (R/W)  SysTick Reload Value Register */
  __vo uint32_t VAL;                    /*!< Offset: 0x008 (R/W)  SysTick Current Value Register */
  __vo  uint32_t CALIB;                  /*!< Offset: 0x00C (R/ )  SysTick Calibration Register */
} SysTick_RegDef_t;

typedef struct{

	__vo uint32_t SR;
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SMPR1;
	__vo uint32_t SMPR2;
	__vo uint32_t JOFR1;
	__vo uint32_t JOFR2;
	__vo uint32_t JOFR3;
	__vo uint32_t JOFR4;
	__vo uint32_t HTR;
	__vo uint32_t LTR;
	__vo uint32_t SQR1;
	__vo uint32_t SQR2;
	__vo uint32_t SQR3;
	__vo uint32_t JSQR;
	__vo uint32_t JDR1;
	__vo uint32_t JDR2;
	__vo uint32_t JDR3;
	__vo uint32_t JDR4;
	__vo uint32_t DR;

}ADC_RegDef_t;


typedef struct
{
  __vo uint32_t LISR;     /*!< DMA low interrupt status register          */
  __vo uint32_t HISR;     /*!< DMA high interrupt status register         */
  __vo uint32_t LIFCR;    /*!< DMA low interrupt flag clear register      */
  __vo uint32_t HIFCR;    /*!< DMA high interrupt flag clear register     */

  __vo uint32_t S0CR;     /*!< DMA stream 0 configuration register        */
  __vo uint32_t S0NDTR;   /*!< DMA stream 0 number of data register       */
  __vo uint32_t S0PAR;    /*!< DMA stream 0 peripheral address register   */
  __vo uint32_t S0M0AR;   /*!< DMA stream 0 memory 0 address register     */
  __vo uint32_t S0M1AR;   /*!< DMA stream 0 memory 1 address register     */
  __vo uint32_t S0FCR;    /*!< DMA stream 0 FIFO control register         */

  __vo uint32_t S1CR;     /*!< DMA stream 1 configuration register        */
  __vo uint32_t S1NDTR;   /*!< DMA stream 1 number of data register       */
  __vo uint32_t S1PAR;    /*!< DMA stream 1 peripheral address register   */
  __vo uint32_t S1M0AR;   /*!< DMA stream 1 memory 0 address register     */
  __vo uint32_t S1M1AR;   /*!< DMA stream 1 memory 1 address register     */
  __vo uint32_t S1FCR;    /*!< DMA stream 1 FIFO control register         */

  __vo uint32_t S2CR;     /*!< DMA stream 2 configuration register        */
  __vo uint32_t S2NDTR;   /*!< DMA stream 2 number of data register       */
  __vo uint32_t S2PAR;    /*!< DMA stream 2 peripheral address register   */
  __vo uint32_t S2M0AR;   /*!< DMA stream 2 memory 0 address register     */
  __vo uint32_t S2M1AR;   /*!< DMA stream 2 memory 1 address register     */
  __vo uint32_t S2FCR;    /*!< DMA stream 2 FIFO control register         */

  __vo uint32_t S3CR;     /*!< DMA stream 3 configuration register        */
  __vo uint32_t S3NDTR;   /*!< DMA stream 3 number of data register       */
  __vo uint32_t S3PAR;    /*!< DMA stream 3 peripheral address register   */
  __vo uint32_t S3M0AR;   /*!< DMA stream 3 memory 0 address register     */
  __vo uint32_t S3M1AR;   /*!< DMA stream 3 memory 1 address register     */
  __vo uint32_t S3FCR;    /*!< DMA stream 3 FIFO control register         */

  __vo uint32_t S4CR;     /*!< DMA stream 4 configuration register        */
  __vo uint32_t S4NDTR;   /*!< DMA stream 4 number of data register       */
  __vo uint32_t S4PAR;    /*!< DMA stream 4 peripheral address register   */
  __vo uint32_t S4M0AR;   /*!< DMA stream 4 memory 0 address register     */
  __vo uint32_t S4M1AR;   /*!< DMA stream 4 memory 1 address register     */
  __vo uint32_t S4FCR;    /*!< DMA stream 4 FIFO control register         */

  __vo uint32_t S5CR;     /*!< DMA stream 5 configuration register        */
  __vo uint32_t S5NDTR;   /*!< DMA stream 5 number of data register       */
  __vo uint32_t S5PAR;    /*!< DMA stream 5 peripheral address register   */
  __vo uint32_t S5M0AR;   /*!< DMA stream 5 memory 0 address register     */
  __vo uint32_t S5M1AR;   /*!< DMA stream 5 memory 1 address register     */
  __vo uint32_t S5FCR;    /*!< DMA stream 5 FIFO control register         */

  __vo uint32_t S6CR;     /*!< DMA stream 6 configuration register        */
  __vo uint32_t S6NDTR;   /*!< DMA stream 6 number of data register       */
  __vo uint32_t S6PAR;    /*!< DMA stream 6 peripheral address register   */
  __vo uint32_t S6M0AR;   /*!< DMA stream 6 memory 0 address register     */
  __vo uint32_t S6M1AR;   /*!< DMA stream 6 memory 1 address register     */
  __vo uint32_t S6FCR;    /*!< DMA stream 6 FIFO control register         */

  __vo uint32_t S7CR;     /*!< DMA stream 7 configuration register        */
  __vo uint32_t S7NDTR;   /*!< DMA stream 7 number of data register       */
  __vo uint32_t S7PAR;    /*!< DMA stream 7 peripheral address register   */
  __vo uint32_t S7M0AR;   /*!< DMA stream 7 memory 0 address register     */
  __vo uint32_t S7M1AR;   /*!< DMA stream 7 memory 1 address register     */
  __vo uint32_t S7FCR;    /*!< DMA stream 7 FIFO control register         */

} DMA_RegDef_t;




typedef struct{

	__vo uint32_t  CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	uint32_t      RESERVED0;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	uint32_t      RESERVED1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	uint32_t      RESERVED2;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t      RESERVED3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	__vo uint32_t RESERVED4;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	uint32_t      RESERVED5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t      RESERVED6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t PLLSAICFGR;
	__vo uint32_t DCKCFGR;
	__vo uint32_t CKGATENR;
	__vo uint32_t DCKFGR2;

}RCC_RegDef_t;



typedef struct{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;

}EXTI_RegDef_t;



typedef struct{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	 uint32_t RESERVED1[2];
	__vo uint32_t CMPCR;
	 uint32_t RESERVED2[2];
	__vo uint32_t CFGR;

}SYSCFG_RegDef_t;

typedef struct{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;

}SPI_RegDef_t;


typedef struct{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t DR;
	__vo uint32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t CCR;
	__vo uint32_t TRISE;
	__vo uint32_t FLTR ;
}I2C_RegDef_t;


typedef struct{
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t BRR;
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t CR3;
	__vo uint32_t GTPR;
}USART_RegDef_t;


#define GPIOA      ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB      ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC      ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD      ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE      ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF      ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG      ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH      ((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI      ((GPIO_RegDef_t*)GPIOI_BASEADDR)



#define TIM1_BASEADDR        (APB2PERIPH_BASEADDR+0x0000)
#define TIM2_BASEADDR        (APB1PERIPH_BASEADDR+0x0000)
#define TIM3_BASEADDR        (APB1PERIPH_BASEADDR+0x0400)
#define TIM4_BASEADDR        (APB1PERIPH_BASEADDR+0x0800)
#define TIM5_BASEADDR        (APB1PERIPH_BASEADDR+0x0C00)
#define TIM6_BASEADDR        (APB1PERIPH_BASEADDR+0x1000)
#define TIM7_BASEADDR        (APB1PERIPH_BASEADDR+0x1400)
#define TIM8_BASEADDR        (APB2PERIPH_BASEADDR+0x0400)
#define TIM9_BASEADDR        (APB2PERIPH_BASEADDR+0x4000)
#define TIM10_BASEADDR       (APB2PERIPH_BASEADDR+0x4400)
#define TIM11_BASEADDR       (APB2PERIPH_BASEADDR+0x4800)
#define TIM12_BASEADDR       (APB1PERIPH_BASEADDR+0x1800)
#define TIM13_BASEADDR       (APB1PERIPH_BASEADDR+0x1C00)
#define TIM14_BASEADDR       (APB1PERIPH_BASEADDR+0x2000)

typedef struct{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SMCR;
	__vo uint32_t DIER;
	__vo uint32_t SR;
	__vo uint32_t EGR;
	__vo uint32_t CCMR1;
	__vo uint32_t CCMR2;
	__vo uint32_t CCER;
	__vo uint32_t CNT;
	__vo uint32_t PSC;
	__vo uint32_t ARR;
	__vo uint32_t CCR1;
	__vo uint32_t CCR2;
	__vo uint32_t CCR3;
	__vo uint32_t CCR4;
	__vo uint32_t DCR;
	__vo uint32_t DMAR;
	__vo uint32_t OR;

}TIMER_RegDef_t;


#define DMA1_BASEADDR    (AHB1PERIPH_BASEADDR+0x6000)
#define DMA2_BASEADDR    (AHB1PERIPH_BASEADDR+0x6400)



#define TIM1    ((TIMER_RegDef_t*)TIM1_BASEADDR)
#define TIM2    ((TIMER_RegDef_t*)TIM2_BASEADDR)
#define TIM3    ((TIMER_RegDef_t*)TIM3_BASEADDR)
#define TIM4    ((TIMER_RegDef_t*)TIM4_BASEADDR)
#define TIM5    ((TIMER_RegDef_t*)TIM5_BASEADDR)
#define TIM6    ((TIMER_RegDef_t*)TIM6_BASEADDR)
#define TIM7    ((TIMER_RegDef_t*)TIM7_BASEADDR)
#define TIM8    ((TIMER_RegDef_t*)TIM8_BASEADDR)
#define TIM9    ((TIMER_RegDef_t*)TIM9_BASEADDR)
#define TIM10   ((TIMER_RegDef_t*)TIM10_BASEADDR)
#define TIM11   ((TIMER_RegDef_t*)TIM11_BASEADDR)
#define TIM12   ((TIMER_RegDef_t*)TIM12_BASEADDR)
#define TIM13   ((TIMER_RegDef_t*)TIM13_BASEADDR)
#define TIM14   ((TIMER_RegDef_t*)TIM14_BASEADDR)


#define DMA1    ((DMA_RegDef_t*)DMA1_BASEADDR)
#define DMA2    ((DMA_RegDef_t*)DMA2_BASEADDR)


#define ADC_CCR_ADCPRE



#define RCC        ((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI       ((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG     ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)


#define SPI1            ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2            ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3            ((SPI_RegDef_t*)SPI3_BASEADDR)



#define I2C1            ((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2            ((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3            ((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1          ((USART_RegDef_t*)USART1_BASEADDR)
#define USART2          ((USART_RegDef_t*)USART2_BASEADDR)
#define USART3          ((USART_RegDef_t*)USART3_BASEADDR)
#define UART4           ((USART_RegDef_t*)UART4_BASEADDR)
#define UART5           ((USART_RegDef_t*)UART5_BASEADDR)
#define USART6          ((USART_RegDef_t*)USART6_BASEADDR)
//Clock enable macros for GPIOX peripherals.

#define GPIOA_PCLK_EN()    (RCC->AHB1ENR |=(1<<0));
#define GPIOB_PCLK_EN()    (RCC->AHB1ENR |=(1<<1));
#define GPIOC_PCLK_EN()    (RCC->AHB1ENR |=(1<<2));
#define GPIOD_PCLK_EN()    (RCC->AHB1ENR |=(1<<3));
#define GPIOE_PCLK_EN()    (RCC->AHB1ENR |=(1<<4));
#define GPIOF_PCLK_EN()    (RCC->AHB1ENR |=(1<<5));
#define GPIOG_PCLK_EN()    (RCC->AHB1ENR |=(1<<6));
#define GPIOH_PCLK_EN()    (RCC->AHB1ENR |=(1<<7));
#define GPIOI_PCLK_EN()    (RCC->AHB1ENR |=(1<<8));


// Clock enable macros for I2Cx peripherals.

#define I2C1_PCLK_EN()  (RCC->APB1ENR |=(1<<21));
#define I2C2_PCLK_EN()  (RCC->APB1ENR |=(1<<22));
#define I2C3_PCLK_EN()  (RCC->APB1ENR |=(1<<23));
//Clock enable macros for SPIx peripherals

#define SPI1_PCLK_EN()  (RCC->APB2ENR |=(1<<12));
#define SPI2_PCLK_EN()  (RCC->APB1ENR |=(1<<14));
#define SPI3_PCLK_EN()  (RCC->APB1ENR |=(1<<15));




#define ADC1_PCLK_EN()  (RCC->APB2ENR |=(1<<8));
#define ADC2_PCLK_EN()  (RCC->APB1ENR |=(1<<9));
#define ADC3_PCLK_EN()  (RCC->APB1ENR |=(1<<10));


#define ADC1_PCLK_DI()  (RCC->APB2ENR &=~(1<<8));
#define ADC2_PCLK_DI()  (RCC->APB1ENR &=~(1<<9));
#define ADC3_PCLK_DI()  (RCC->APB1ENR &=~(1<<10));


//Clock enable macros for USARTx peripherals
#define USART1_PCLK_EN()  (RCC->APB2ENR |=(1<<4));
#define USART2_PCLK_EN()  (RCC->APB1ENR |=(1<<17));
#define USART3_PCLK_EN()  (RCC->APB1ENR |=(1<<18));
#define USART6_PCLK_EN()  (RCC->APB2ENR |=(1<<5));
#define UART4_PCLK_EN()   (RCC->APB1ENR |=(1<<19));
#define UART5_PCLK_EN()   (RCC->APB1ENR |=(1<<20));
// Clock enable macros for SYSCFG peripherals.
#define SYSCFG_PCLK_EN()  (RCC->APB2ENR |=(1<<14));

//Clock disable macros for GPIOx peripherals.
#define GPIOA_PCLK_DI()    (RCC->AHB1ENR &=~(1<<0));
#define GPIOB_PCLK_DI()    (RCC->AHB1ENR &=~(1<<1));
#define GPIOC_PCLK_DI()    (RCC->AHB1ENR &=~(1<<2));
#define GPIOD_PCLK_DI()    (RCC->AHB1ENR &=~(1<<3));
#define GPIOE_PCLK_DI()    (RCC->AHB1ENR &=~(1<<4));
#define GPIOF_PCLK_DI()    (RCC->AHB1ENR &=~(1<<5));
#define GPIOG_PCLK_DI()    (RCC->AHB1ENR &=~(1<<6));
#define GPIOH_PCLK_DI()    (RCC->AHB1ENR &=~(1<<7));
#define GPIOI_PCLK_DI()    (RCC->AHB1ENR &=~(1<<8));


// Clock disable macros for I2Cx peripherals.

#define I2C1_PCLK_DI()  (RCC->APB1ENR &=~(1<<21));
#define I2C2_PCLK_DI()  (RCC->APB1ENR &=~(1<<22));
#define I2C3_PCLK_DI()  (RCC->APB1ENR &=~(1<<23));
//Clock enable macros for SPIx peripherals

#define SPI1_PCLK_DI()  (RCC->APB2ENR &=~(1<<12));
#define SPI2_PCLK_DI()  (RCC->APB1ENR &=~(1<<14));
#define SPI3_PCLK_DI()  (RCC->APB1ENR &=~(1<<15));

//Clock enable macros for USARTx peripherals
#define USART1_PCLK_DI()  (RCC->APB2ENR &=~(1<<4));
#define USART2_PCLK_DI()  (RCC->APB1ENR &=~(1<<17));
#define USART3_PCLK_DI()  (RCC->APB1ENR &=~(1<<18));
#define USART6_PCLK_DI()  (RCC->APB2ENR &=~(1<<5));
#define UART4_PCLK_DI()   (RCC->APB1ENR &=~(1<<19));
#define UART5_PCLK_DI()   (RCC->APB1ENR &=~(1<<20));

// Clock enable macros for SYSCFG peripherals.
#define SYSCFG_PCLK_DI()  (RCC->APB2ENR &=~(1<<14));

// Macros to reset GPIOX peripherals.
#define GPIOA_REG_RESET()   do {(RCC->AHB1RSTR|=(1<<0)); (RCC->AHB1RSTR&=~(1<<0)); } while(0)
#define GPIOB_REG_RESET()   do {(RCC->AHB1RSTR|=(1<<1)); (RCC->AHB1RSTR&=~(1<<1)); } while(0)
#define GPIOC_REG_RESET()   do {(RCC->AHB1RSTR|=(1<<2)); (RCC->AHB1RSTR&=~(1<<2)); } while(0)
#define GPIOD_REG_RESET()   do {(RCC->AHB1RSTR|=(1<<3)); (RCC->AHB1RSTR&=~(1<<3)); } while(0)
#define GPIOE_REG_RESET()   do {(RCC->AHB1RSTR|=(1<<4)); (RCC->AHB1RSTR&=~(1<<4)); } while(0)
#define GPIOF_REG_RESET()   do {(RCC->AHB1RSTR|=(1<<5)); (RCC->AHB1RSTR&=~(1<<5)); } while(0)
#define GPIOG_REG_RESET()   do {(RCC->AHB1RSTR|=(1<<6)); (RCC->AHB1RSTR&=~(1<<6)); } while(0)
#define GPIOH_REG_RESET()   do {(RCC->AHB1RSTR|=(1<<7)); (RCC->AHB1RSTR&=~(1<<7)); } while(0)
#define GPIOI_REG_RESET()   do {(RCC->AHB1RSTR|=(1<<8)); (RCC->AHB1RSTR&=~(1<<8)); } while(0)


// Resetting the registers.
#define SPI1_REG_RESET()   do {(RCC->APB2RSTR|=(1<<12)); (RCC->APB2RSTR&=~(1<<12)); } while(0)
#define SPI2_REG_RESET()   do {(RCC->APB2RSTR|=(1<<14)); (RCC->APB2RSTR&=~(1<<14)); } while(0)
#define SPI3_REG_RESET()   do {(RCC->APB2RSTR|=(1<<15)); (RCC->APB2RSTR&=~(1<<15)); } while(0)

#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA)?0:\
		(x == GPIOB)?1:\
		(x == GPIOC)?2:\
		(x == GPIOD)?3:\
        (x == GPIOE)?4:\
        (x == GPIOF)?5:\
        (x == GPIOG)?6:\
        (x == GPIOH)?7: \
        (x == GPIOI)?8:0)


#define TIM1_PCLK_EN()  (RCC->APB2ENR |=(1<<0))
#define TIM8_PCLK_EN()  (RCC->APB2ENR |=(1<<1))
#define TIM9_PCLK_EN()  (RCC->APB2ENR |=(1<<16))
#define TIM10_PCLK_EN()  (RCC->APB2ENR |=(1<<17))
#define TIM11_PCLK_EN()  (RCC->APB2ENR |=(1<<18))
#define TIM2_PCLK_EN()  (RCC->APB1ENR |=(1<<0))
#define TIM3_PCLK_EN()  (RCC->APB1ENR |=(1<<1))
#define TIM4_PCLK_EN()  (RCC->APB1ENR |=(1<<2))
#define TIM5_PCLK_EN()  (RCC->APB1ENR |=(1<<3))
#define TIM6_PCLK_EN()  (RCC->APB1ENR |=(1<<4))
#define TIM7_PCLK_EN()  (RCC->APB1ENR |=(1<<5))
#define TIM12_PCLK_EN()  (RCC->APB1ENR |=(1<<6))
#define TIM13_PCLK_EN()  (RCC->APB1ENR |=(1<<7))
#define TIM14_PCLK_EN()  (RCC->APB1ENR |=(1<<8))



#define TIM1_PCLK_DI()  (RCC->APB2ENR &=~(1<<0))
#define TIM8_PCLK_DI()  (RCC->APB2ENR &=(1<<1))
#define TIM9_PCLK_DI()  (RCC->APB2ENR &=~(1<<16))
#define TIM10_PCLK_DI()  (RCC->APB2ENR &=~(1<<17))
#define TIM11_PCLK_DI()  (RCC->APB1ENR &=~(1<<18))
#define TIM2_PCLK_DI()  (RCC->APB1ENR &=~(1<<0))
#define TIM3_PCLK_DI()  (RCC->APB1ENR &=~(1<<1))
#define TIM4_PCLK_DI()  (RCC->APB1ENR &=~(1<<2))
#define TIM5_PCLK_DI()  (RCC->APB1ENR &=~(1<<3))
#define TIM6_PCLK_DI()  (RCC->APB1ENR &=~(1<<4))
#define TIM7_PCLK_DI()  (RCC->APB1ENR &=~(1<<5))
#define TIM12_PCLK_DI()  (RCC->APB1ENR &=~(1<<6))
#define TIM13_PCLK_DI()  (RCC->APB1ENR &=~(1<<7))
#define TIM14_PCLK_DI()  (RCC->APB1ENR &=~(1<<8))

#define ENABLE   1
#define DISABLE  0
#define SET      ENABLE
#define RESET    DISABLE
#define GPIO_PIN_SET  SET
#define GPIO_PIN_RESET RESET

//Bit position definationss of CR1 peripheral.

#define SPI_CR1_CPHA         0
#define SPI_CR1_CPOL         1
#define SPI_CR1_MSTR         2
#define SPI_CR1_BR           3
#define SPI_CR1_SPE          6
#define SPI_CR1_LSBFIRST     7
#define SPI_CR1_SSI          8
#define SPI_CR1_SSM          9
#define SPI_CR1_RXONLY       10
#define SPI_CR1_DFF          11
#define SPI_CR1_CRCNEXT      12
#define SPI_CR1_CRCEN        13
#define SPI_CR1_BIDIOE       14
#define SPI_CR1_BIDIMODE     15

//Bit position definationss of CR2 peripheral.
#define SPI_CR2_RXDMAEN      0
#define SPI_CR2_TXDMAEN      1
#define SPI_CR2_SSOE         2
#define SPI_CR2_FRF          4
#define SPI_CR2_ERRIE        5
#define SPI_CR2_RXNEIE       6
#define SPI_CR2_TXEIE        7


// Bit position defination of SPI_SR

#define SPI_SR_RXNE             0
#define SPI_SR_TXE              1
#define SPI_SR_CHSIDE           2
#define SPI_SR_UDR              3
#define SPI_SR_CRCERR           4
#define SPI_SR_MODF             5
#define SPI_SR_OVR              6
#define SPI_SR_BSY              7
#define SPI_SR_FRE              8


//bit position defination of i2c_cr1

#define I2C_CR1_PE               0
#define I2C_CR1_NOSTRETCH        7
#define I2C_CR1_START            8
#define I2C_CR1_STOP             9
#define I2C_CR1_ACK              10
#define I2C_CR1_SWRST            15


//bit position defination of i2c_cr2

#define I2C_CR2_FREQ               0
#define I2C_CR2_ITERREN            8
#define I2C_CR2_ITEVTEN            9
#define I2C_CR2_ITBUFEN            10

//bit position defination of i2c_oar1

#define I2C_OAR_ADD0               0
#define I2C_OAR_ADD71              1
#define I2C_OAR_ADD98              8
#define I2C_OAR_ADDMODE            15


//bit position defination of i2c_SR1

#define I2C_SR1_SB                    0
#define I2C_SR1_ADDR                  1
#define I2C_SR1_BTF                   2
#define I2C_SR1_ADD10                 3
#define I2C_SR1_STOPF                 4
#define I2C_SR1_RXNE                  6
#define I2C_SR1_TXE                   7
#define I2C_SR1_BERR                  8
#define I2C_SR1_ARLO                  9
#define I2C_SR1_AF                    10
#define I2C_SR1_OVR                   11
#define I2C_SR1_TIMEOUT               14


//bit position defination of i2c_SR2

#define I2C_SR2_MSL                   0
#define I2C_SR2_BUSY                  1
#define I2C_SR2_TRA                   2
#define I2C_SR2_GENCALL               4
#define I2C_SR2_DUALF                 7

//bit position defination of i2c_CCR
#define I2C_CCR_CCR                   0
#define I2C_CCR_DUTY                  14
#define I2C_CCR_FS                    15
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

//IRQ(interrupt request) number of stm32f407x MCU.

/******  Cortex-M4 Processor Exceptions Numbers ****************************************************************/
#define  NonMaskableInt_IRQn          -14   /*!< 2 Non Maskable Interrupt                                          */
#define  MemoryManagement_IRQn        -12   /*!< 4 Cortex-M4 Memory Management Interrupt                           */
#define  BusFault_IRQn               -11    /*!< 5 Cortex-M4 Bus Fault Interrupt                                   */
#define UsageFault_IRQn               -10    /*!< 6 Cortex-M4 Usage Fault Interrupt                                 */
#define  SVCall_IRQn                  -5     /*!< 11 Cortex-M4 SV Call Interrupt                                    */
#define  DebugMonitor_IRQn             -4     /*!< 12 Cortex-M4 Debug Monitor Interrupt                              */
#define  PendSV_IRQn                  -2     /*!< 14 Cortex-M4 Pend SV Interrupt                                    */
#define  SysTick_IRQn                 -1     /*!< 15 Cortex-M4 System Tick Interrupt                                */
  /******  STM32 specific Interrupt Numbers **********************************************************************/
#define WWDG_IRQn                    0      /*!< Window WatchDog Interrupt                                         */
#define PVD_IRQn                     1      /*!< PVD through EXTI Line detection Interrupt                         */
#define TAMP_STAMP_IRQn              2      /*!< Tamper and TimeStamp interrupts through the EXTI line             */
#define RTC_WKUP_IRQn                3      /*!< RTC Wakeup interrupt through the EXTI line                        */
#define FLASH_IRQn                   4      /*!< FLASH global Interrupt                                            */
#define RCC_IRQn                     5      /*!< RCC global Interrupt                                              */
#define EXTI0_IRQn                   6      /*!< EXTI Line0 Interrupt                                              */
#define EXTI1_IRQn                   7      /*!< EXTI Line1 Interrupt                                              */
#define EXTI2_IRQn                   8      /*!< EXTI Line2 Interrupt                                              */
#define EXTI3_IRQn                   9      /*!< EXTI Line3 Interrupt                                              */
#define EXTI4_IRQn                   10     /*!< EXTI Line4 Interrupt                                              */
#define DMA1_Stream0_IRQn            11     /*!< DMA1 Stream 0 global Interrupt                                    */
#define DMA1_Stream1_IRQn            12     /*!< DMA1 Stream 1 global Interrupt                                    */
#define DMA1_Stream2_IRQn            13     /*!< DMA1 Stream 2 global Interrupt                                    */
#define DMA1_Stream3_IRQn            14     /*!< DMA1 Stream 3 global Interrupt                                    */
#define DMA1_Stream4_IRQn            15     /*!< DMA1 Stream 4 global Interrupt                                    */
#define DMA1_Stream5_IRQn            16     /*!< DMA1 Stream 5 global Interrupt                                    */
#define DMA1_Stream6_IRQn            17     /*!< DMA1 Stream 6 global Interrupt                                    */
#define ADC_IRQn                     18     /*!< ADC1, ADC2 and ADC3 global Interrupts                             */
#define CAN1_TX_IRQn                 19     /*!< CAN1 TX Interrupt                                                 */
#define CAN1_RX0_IRQn                20     /*!< CAN1 RX0 Interrupt                                                */
#define CAN1_RX1_IRQn                21     /*!< CAN1 RX1 Interrupt                                                */
#define CAN1_SCE_IRQn                22     /*!< CAN1 SCE Interrupt                                                */
#define EXTI9_5_IRQn                 23     /*!< External Line[9:5] Interrupts                                     */
#define TIM1_BRK_TIM9_IRQn           24    /*!< TIM1 Break interrupt and TIM9 global interrupt                    */
#define TIM1_UP_TIM10_IRQn           25     /*!< TIM1 Update Interrupt and TIM10 global interrupt                  */
#define TIM1_TRG_COM_TIM11_IRQn      26     /*!< TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
#define TIM1_CC_IRQn                 27     /*!< TIM1 Capture Compare Interrupt                                    */
#define TIM2_IRQn                    28     /*!< TIM2 global Interrupt                                             */
#define TIM3_IRQn                    29     /*!< TIM3 global Interrupt                                             */
#define  TIM4_IRQn                    30     /*!< TIM4 global Interrupt                                             */
#define I2C1_EV_IRQn                 31     /*!< I2C1 Event Interrupt                                              */
#define I2C1_ER_IRQn                 32     /*!< I2C1 Error Interrupt                                              */
#define I2C2_EV_IRQn                 33     /*!< I2C2 Event Interrupt                                              */
#define I2C2_ER_IRQn                 34    /*!< I2C2 Error Interrupt                                              */
#define  SPI2_IRQn                    36     /*!< SPI2 global Interrupt                                             */
#define  USART1_IRQn                  37     /*!< USART1 global Interrupt                                           */
#define  USART2_IRQn                  38    /*!< USART2 global Interrupt                                           */
#define  USART3_IRQn                  39     /*!< USART3 global Interrupt                                           */
#define  EXTI15_10_IRQn               40     /*!< External Line[15:10] Interrupts                                   */
#define RTC_Alarm_IRQn               41     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
#define OTG_FS_WKUP_IRQn             42     /*!< USB OTG FS Wakeup through EXTI line interrupt                     */
#define TIM8_BRK_TIM12_IRQn          43     /*!< TIM8 Break Interrupt and TIM12 global interrupt                   */
#define TIM8_UP_TIM13_IRQn           44     /*!< TIM8 Update Interrupt and TIM13 global interrupt                  */
#define TIM8_TRG_COM_TIM14_IRQn      45     /*!< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
#define TIM8_CC_IRQn                 46     /*!< TIM8 Capture Compare global interrupt                             */
#define  DMA1_Stream7_IRQn            47     /*!< DMA1 Stream7 Interrupt                                            */
#define  FSMC_IRQn                    48     /*!< FSMC global Interrupt                                             */
#define  SDIO_IRQn                    49     /*!< SDIO global Interrupt                                             */
#define  TIM5_IRQn                    50     /*!< TIM5 global Interrupt                                             */
#define  SPI3_IRQn                    51    /*!< SPI3 global Interrupt                                             */
#define  UART4_IRQn                   52     /*!< UART4 global Interrupt                                            */
#define  UART5_IRQn                   53     /*!< UART5 global Interrupt                                            */
#define TIM6_DAC_IRQn                 54    /*!< TIM6 global and DAC1&2 underrun error  interrupts                 */
#define TIM7_IRQn                     55     /*!< TIM7 global interrupt                                             */
#define  DMA2_Stream0_IRQn            56     /*!< DMA2 Stream 0 global Interrupt                                    */
#define DMA2_Stream1_IRQn             57     /*!< DMA2 Stream 1 global Interrupt                                    */
#define  DMA2_Stream2_IRQn            58     /*!< DMA2 Stream 2 global Interrupt                                    */
#define  DMA2_Stream3_IRQn            59     /*!< DMA2 Stream 3 global Interrupt                                    */
#define DMA2_Stream4_IRQn            60     /*!< DMA2 Stream 4 global Interrupt                                    */
#define ETH_IRQn                     61     /*!< Ethernet global Interrupt                                         */
#define ETH_WKUP_IRQn                62     /*!< Ethernet Wakeup through EXTI line Interrupt                       */
#define CAN2_TX_IRQn                63     /*!< CAN2 TX Interrupt                                                 */
#define CAN2_RX0_IRQn                64     /*!< CAN2 RX0 Interrupt                                                */
#define CAN2_RX1_IRQn                65     /*!< CAN2 RX1 Interrupt                                                */
#define  CAN2_SCE_IRQn                66     /*!< CAN2 SCE Interrupt                                                */
#define  OTG_FS_IRQn                  67    /*!< USB OTG FS global Interrupt                                       */
#define DMA2_Stream5_IRQn            68     /*!< DMA2 Stream 5 global interrupt                                    */
#define DMA2_Stream6_IRQn            69     /*!< DMA2 Stream 6 global interrupt                                    */
#define DMA2_Stream7_IRQn            70     /*!< DMA2 Stream 7 global interrupt                                    */
#define USART6_IRQn                  71     /*!< USART6 global interrupt                                           */
#define I2C3_EV_IRQn                 72     /*!< I2C3 event interrupt                                              */
#define I2C3_ER_IRQn                 73    /*!< I2C3 error interrupt                                              */
#define OTG_HS_EP1_OUT_IRQn          74     /*!< USB OTG HS End Point 1 Out global interrupt                       */
#define OTG_HS_EP1_IN_IRQn           75    /*!< USB OTG HS End Point 1 In global interrupt                        */
#define OTG_HS_WKUP_IRQn             76     /*!< USB OTG HS Wakeup through EXTI interrupt                          */
#define  OTG_HS_IRQn                  77     /*!< USB OTG HS global interrupt                                       */
#define DCMI_IRQn                    78     /*!< DCMI global interrupt                                             */
#define RNG_IRQn                     80    /*!< RNG global Interrupt                                              */
#define FPU_IRQn                     81      /*!< FPU global interrupt                                              */

//macros for all the possible priority levels.

#define NVIC_IRQ_PRI0   0
#define NVIC_IRQ_PRI1   1
#define NVIC_IRQ_PRI2   2
#define NVIC_IRQ_PRI3   3
#define NVIC_IRQ_PRI4   4
#define NVIC_IRQ_PRI5   5
#define NVIC_IRQ_PRI6   6
#define NVIC_IRQ_PRI7   7
#define NVIC_IRQ_PRI8   8
#define NVIC_IRQ_PRI9   9
#define NVIC_IRQ_PRI10  10
#define NVIC_IRQ_PRI11  11
#define NVIC_IRQ_PRI12  12
#define NVIC_IRQ_PRI13  13
#define NVIC_IRQ_PRI14  14
#define NVIC_IRQ_PRI15  15

/*
  * @brief   AF 0 selection
  */
#define GPIO_AF0_RTC_50Hz      ((uint8_t)0x00)  /* RTC_50Hz Alternate Function mapping                       */
#define GPIO_AF0_MCO           ((uint8_t)0x00)  /* MCO (MCO1 and MCO2) Alternate Function mapping            */
#define GPIO_AF0_TAMPER        ((uint8_t)0x00)  /* TAMPER (TAMPER_1 and TAMPER_2) Alternate Function mapping */
#define GPIO_AF0_SWJ           ((uint8_t)0x00)  /* SWJ (SWD and JTAG) Alternate Function mapping             */
#define GPIO_AF0_TRACE         ((uint8_t)0x00)  /* TRACE Alternate Function mapping                          */

/**
  * @brief   AF 1 selection
  */
#define GPIO_AF1_TIM1          ((uint8_t)0x01)  /* TIM1 Alternate Function mapping */
#define GPIO_AF1_TIM2          ((uint8_t)0x01)  /* TIM2 Alternate Function mapping */

/**
  * @brief   AF 2 selection
  */
#define GPIO_AF2_TIM3          ((uint8_t)0x02)  /* TIM3 Alternate Function mapping */
#define GPIO_AF2_TIM4          ((uint8_t)0x02)  /* TIM4 Alternate Function mapping */
#define GPIO_AF2_TIM5          ((uint8_t)0x02)  /* TIM5 Alternate Function mapping */

/**
  * @brief   AF 3 selection
  */
#define GPIO_AF3_TIM8          ((uint8_t)0x03)  /* TIM8 Alternate Function mapping  */
#define GPIO_AF3_TIM9          ((uint8_t)0x03)  /* TIM9 Alternate Function mapping  */
#define GPIO_AF3_TIM10         ((uint8_t)0x03)  /* TIM10 Alternate Function mapping */
#define GPIO_AF3_TIM11         ((uint8_t)0x03)  /* TIM11 Alternate Function mapping */

/**
  * @brief   AF 4 selection
  */
#define GPIO_AF4_I2C1          ((uint8_t)0x04)  /* I2C1 Alternate Function mapping */
#define GPIO_AF4_I2C2          ((uint8_t)0x04)  /* I2C2 Alternate Function mapping */
#define GPIO_AF4_I2C3          ((uint8_t)0x04)  /* I2C3 Alternate Function mapping */

/**
  * @brief   AF 5 selection
  */
#define GPIO_AF5_SPI1          ((uint8_t)0x05)  /* SPI1 Alternate Function mapping        */
#define GPIO_AF5_SPI2          ((uint8_t)0x05)  /* SPI2/I2S2 Alternate Function mapping   */
#define GPIO_AF5_I2S3ext       ((uint8_t)0x05)  /* I2S3ext_SD Alternate Function mapping  */

/**
  * @brief   AF 6 selection
  */
#define GPIO_AF6_SPI3          ((uint8_t)0x06)  /* SPI3/I2S3 Alternate Function mapping  */
#define GPIO_AF6_I2S2ext       ((uint8_t)0x06)  /* I2S2ext_SD Alternate Function mapping */

/**
  * @brief   AF 7 selection
  */
#define GPIO_AF7_USART1        ((uint8_t)0x07)  /* USART1 Alternate Function mapping     */
#define GPIO_AF7_USART2        ((uint8_t)0x07)  /* USART2 Alternate Function mapping     */
#define GPIO_AF7_USART3        ((uint8_t)0x07)  /* USART3 Alternate Function mapping     */
#define GPIO_AF7_I2S3ext       ((uint8_t)0x07)  /* I2S3ext_SD Alternate Function mapping */

/**
  * @brief   AF 8 selection
  */
#define GPIO_AF8_UART4         ((uint8_t)0x08)  /* UART4 Alternate Function mapping  */
#define GPIO_AF8_UART5         ((uint8_t)0x08)  /* UART5 Alternate Function mapping  */
#define GPIO_AF8_USART6        ((uint8_t)0x08)  /* USART6 Alternate Function mapping */

/**
  * @brief   AF 9 selection
  */
#define GPIO_AF9_CAN1          ((uint8_t)0x09)  /* CAN1 Alternate Function mapping  */
#define GPIO_AF9_CAN2          ((uint8_t)0x09)  /* CAN2 Alternate Function mapping  */
#define GPIO_AF9_TIM12         ((uint8_t)0x09)  /* TIM12 Alternate Function mapping */
#define GPIO_AF9_TIM13         ((uint8_t)0x09)  /* TIM13 Alternate Function mapping */
#define GPIO_AF9_TIM14         ((uint8_t)0x09)  /* TIM14 Alternate Function mapping */

/**
  * @brief   AF 10 selection
  */
#define GPIO_AF10_OTG_FS        ((uint8_t)0x0A)  /* OTG_FS Alternate Function mapping */
#define GPIO_AF10_OTG_HS        ((uint8_t)0x0A)  /* OTG_HS Alternate Function mapping */

/**
  * @brief   AF 11 selection
  */
#define GPIO_AF11_ETH           ((uint8_t)0x0B)  /* ETHERNET Alternate Function mapping */

/**
  * @brief   AF 12 selection
  */
#define GPIO_AF12_FSMC          ((uint8_t)0x0C)  /* FSMC Alternate Function mapping                     */
#define GPIO_AF12_OTG_HS_FS     ((uint8_t)0x0C)  /* OTG HS configured in FS, Alternate Function mapping */
#define GPIO_AF12_SDIO          ((uint8_t)0x0C)  /* SDIO Alternate Function mapping                     */

/**
  * @brief   AF 13 selection
  */
#define GPIO_AF13_DCMI          ((uint8_t)0x0D)  /* DCMI Alternate Function mapping */

/**
  * @brief   AF 15 selection
  */

#define GPIO_AF15_EVENTOUT      ((uint8_t)0x0F)  /* EVENTOUT Alternate Function mapping */

// DMA related definations.

#define DMA1EN         (1U<<21)
#define DMA2EN         (1U<<22)
#define DMA_S_EN       (1U<<0)

#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include"stm32f407xx_i2c_driver.h"
#include "stm32f407xx_usart_driver.h"
#include "stm32f407xx_rcc_driver.h"
#endif /* INC_STM32F407XX_H_ */
