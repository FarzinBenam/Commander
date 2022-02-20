#ifndef _CONFIGS_H
#define _CONFIGS_H

#ifdef __cplusplus
 extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Configurations    ---------------------------------------------------------*/
#define rtos            0
#define COM             1               // usart1 used to communicate with COM port1
#define hc_05           0								// uart4 used if the hc-05 ble module used
#define systick         1               // use RTC for systick
#define qspi_debug      0
#define wifi_echo       1


//#define F_CPU           SystemCoreClock
#define terminal_hc_05  hc_05
#define terminal_COM    !terminal_hc_05

/* General definitions   -----------------------------------------------------*/
#define HTS221_SLAVE_ADD    0xBE
#define I2C_MEM_ADD_LSB     (__ADDRESS__)              ((uint8_t)((uint16_t)((__ADDRESS__) & (uint16_t)(0x00FFU))))
#define HTS221_BIT(x) ((uint8_t)x)

/* Private constants ---------------------------------------------------------*/
//////////////////////////////////////////////////////////////////////
#if (COM == 0) && (hc_05 == 0)
#error	"No Terminal available! Neither COM port nor HC-05 are not defined!"
#endif

#if (terminal_hc_05 == 1)
	#define terminal_usart	UART4
	#define terminal_IRQn		UART4_IRQn		
#elif (COM == 1)
	#define terminal_usart	USART1
	#define terminal_IRQn		USART1_IRQn
#endif

#define hc_05_USART				UART4
#define hc_05_BAUDRATE		UART4_BAUDRATE
#define hc_05_IRQn				UART4_IRQn
#define COM_USART					USART1
#define COM_BAUDRATE  		USART1_BAUDRATE
#define COM_IRQn					USART1_IRQn

#define USART1_BAUDRATE		115200
#define USART3_BAUDRATE		115200
#define UART4_BAUDRATE		9600

#define USART1_TX					LL_GPIO_PIN_6
#define USART1_RX					LL_GPIO_PIN_7
#define USART1_PORT				GPIOB

#define UART4_RX					LL_GPIO_PIN_1
#define UART4_TX					LL_GPIO_PIN_0
#define UART4_PORT				GPIOA

//#define TX_BUFFER_SIZE    1024
//#define RX_BUFFER_SIZE    1024
//#define TX_TIMEOUT        2000
//#define RX_TIMEOUT        2000


/*******************************************************************************
 * defintions
 *******************************************************************************/
uint8_t   Configs (void);
void      wifi_Config (void);
void      HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

/* ==============   I2C2 SPECIFI FUNCTIONS         ========================== */
void      i2c2_read									(uint8_t SADD, uint8_t ReadADD, uint32_t TransferSize, uint8_t *buffer);
void      i2c2_write								(uint8_t SADD, uint8_t WriteADD, uint32_t TransferSize, uint8_t *buffer);
uint8_t   i2c2_sensor_read					(uint16_t DeviceAddr, uint8_t RegisterAddr);
void      i2c2_sensor_write					(uint16_t DeviceAddr, uint8_t RegisterAddr, uint8_t *tmp);
void      i2c2_sensor_readmultiple	(uint16_t DeviceAddr, uint8_t RegisterAddr, uint8_t *buffer, uint8_t readsize);

/* ==============   Time Functions      ===================================== */
void		  rtc_set		  (LL_RTC_TimeTypeDef *time, LL_RTC_DateTypeDef *date);
void		  rtc_read		(LL_RTC_TimeTypeDef	*time, LL_RTC_DateTypeDef	*date);

/* ==============   CRC Functions      ====================================== */
uint8_t   crc_8bit 		(uint8_t *buffer, uint8_t length);

/* ==============   General Functions  ====================================== */
uint32_t  getTick	            (void);
void		  _bsp_clk_freq_get   (void);
void		  nl                  (uint8_t line);
void      halt                (char Message[]);
void    togglepin             (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void      Error_Handler       (void);

#endif
