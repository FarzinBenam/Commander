/**
  ******************************************************************************
  * @file    main.h 
  * @author  Farzin M.Benam
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * 
  * 
  *
  * 
  * 
  * 
  * 
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* HAL drivers */
#include "stm32l4xx_hal_conf.h"



/* LL drivers */
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_system.h"
#include "stm32l4xx_ll_utils.h"
//#include "stm32l4xx_ll_pwr.h"
//#include "stm32l4xx_ll_exti.h"
#include "stm32l4xx_ll_gpio.h"
/* LL drivers specific to LL examples IPs */
//#include "stm32l4xx_ll_adc.h"
//#include "stm32l4xx_ll_comp.h"
#include "stm32l4xx_ll_cortex.h"
#include "stm32l4xx_ll_crc.h"
//#include "stm32l4xx_ll_dac.h"
//#include "stm32l4xx_ll_dma.h"
#include "stm32l4xx_ll_i2c.h"
//#include "stm32l4xx_ll_iwdg.h"
//#include "stm32l4xx_ll_lptim.h"
//#include "stm32l4xx_ll_lpuart.h"
//#include "stm32l4xx_ll_opamp.h"
//#include "stm32l4xx_ll_rng.h"
#include "stm32l4xx_ll_rtc.h"
#include "stm32l4xx_ll_spi.h"
//#include "stm32l4xx_ll_swpmi.h"
//#include "stm32l4xx_ll_tim.h"
#include "stm32l4xx_ll_usart.h"
//#include "stm32l4xx_ll_wwdg.h"

/* FCMD Libs */
#include "fcmd.h"
#include "configs.h"
#include "Process.h"
#include "uart.h"
//#include "wifi.h"
#include "qspi.h"
#include "fs.h"
#include "components.h"
#include "mx25r6435f.h"
//////////////////////////////
// wifi library
#include "es_wifi.h"
#include "es_wifi_conf.h"
#include "es_wifi_io.h"
#include "WiFi.h"


#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Exported types ------------------------------------------------------------*/
/* define a structure for RTC_TR register with bit fields */
typedef struct{
	uint32_t second		:7;
	uint32_t reserved1:1;
	uint32_t minute		:7;
	uint32_t reserved2:1;
	uint32_t hour			:6;
	uint32_t ampm			:1;
	uint32_t reserved3:9;
}rtc_time_params;

/* define a structure for RTC_DR register with bit fields */
typedef struct{
	uint32_t Day			:6;
	uint32_t reserved1:2;
	uint32_t month		:5;
	uint32_t weekday	:3;
	uint32_t year			:8;
	uint32_t reserved2:8;
}rtc_date_params;


/* ==============   BOARD SPECIFIC CONFIGURATION CODE BEGIN    ============== */
/**
  * @brief GPIO Clock Enable Macros
  */
#define GPIOA_CLK_ENABLE()                LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA)
#define GPIOB_CLK_ENABLE()                LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB)
#define GPIOC_CLK_ENABLE()                LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC)
#define GPIOD_CLK_ENABLE()                LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOD)
#define GPIOE_CLK_ENABLE()                LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOE)
/**
  * @brief USART Clock Enable Macros
  */
#define USART1_CLK_ENABLE()               LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1)
#define USART2_CLK_ENABLE()               LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2)
#define USART3_CLK_ENABLE()               LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3)
#define USART4_CLK_ENABLE()               LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART4)
/**
  * @brief I2C Clock Enable Macros
  */
#define I2C1_CLK_ENABLE()                 LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1)
#define I2C2_CLK_ENABLE()                 LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C2)
#define I2C3_CLK_ENABLE()                 LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C3)

/**
  * @brief SPI Clock Enable Macros
  */
#define SPI1_CLK_ENABLE()                 LL_APB2_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI1)
#define SPI2_CLK_ENABLE()                 LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2)
#define SPI3_CLK_ENABLE()                 LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI3)


/**
  * @brief LED1
  */
#define LED1_PIN                          LL_GPIO_PIN_5
#define LED1_PORT                         GPIOA
#define LED1_CLK_ENABLE()                 LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA)
#define LED1_ON()		                      SET_BIT(LED1_PORT->BSRR, LED1_PIN)
#define LED1_OFF()	                      SET_BIT(LED1_PORT->BRR, LED1_PIN)


/**
  * @brief LED2
  */
#define LED2_PIN                          LL_GPIO_PIN_14
#define LED2_PORT                         GPIOB
#define LED2_CLK_ENABLE()                 LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB)
#define LED2_ON()                         SET_BIT(LED2_PORT->BSRR, LED2_PIN)
#define LED2_OFF()	                      SET_BIT(LED2_PORT->BRR, LED2_PIN)

/**
  * @brief LED3
  */
#define LED3_PIN                          LL_GPIO_PIN_9
#define LED3_PORT                         GPIOC
#define LED3_CLK_ENABLE()                 LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC)
#define LED3_ON()                         SET_BIT(LED3_PORT->BSRR, LED3_PIN)
#define LED3_OFF()	                      SET_BIT(LED3_PORT->BRR, LED3_PIN)


/**
  * @brief push-button
  */
#define BUTTON_PIN                         LL_GPIO_PIN_13
#define BUTTON_PORT                        GPIOC
#define BUTTON_CLK_ENABLE()                LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC)

                                                
/**
  * @brief ISM43362-M3G-L44 (wifi module definitions)
  */
#define WIFI_RESET_PIN                      LL_GPIO_PIN_8
#define WIFI_RESET_PORT                     GPIOE
                                                
#define WIFI_READY_PIN                      LL_GPIO_PIN_1
#define WIFI_READY_PORT                     GPIOE
#define WIFI_READY_EXTI_IRQn                EXTI1_IRQn
                                                
#define WIFI_WAKEUP_PIN                     LL_GPIO_PIN_13
#define WIFI_WAKEUP_PORT                    GPIOB

/**
  * @brief SPI3 Pins
  */
#define SPI3_NSS_PIN                        LL_GPIO_PIN_0
#define SPI3_NSS_PORT                       GPIOE

#define SPI3_CLK_PIN                        LL_GPIO_PIN_10
#define SPI3_CLK_PORT                       GPIOC

#define SPI3_MISO_PIN                       LL_GPIO_PIN_11
#define SPI3_MISO_PORT                      GPIOC

#define SPI3_MOSI_PIN                       LL_GPIO_PIN_12
#define SPI3_MOSI_PORT                      GPIOC


                                                
/* ==============   BOARD SPECIFIC CONFIGURATION CODE END      ============== */

/**
  * @brief Toggle periods for various blinking modes
  */
#define LED_BLINK_FAST  200
#define LED_BLINK_SLOW  500
#define LED_BLINK_ERROR 1000

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/* Private function prototypes -----------------------------------------------*/
                                                
                                                
#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
