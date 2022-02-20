#ifndef PROCESS_H_
#define PROCESS_H_
#include "main.h"



#define GPIO_NUMBER           (16u)
#define SOCKET_NUM (0)
#define TIMEOUT (20)
#define BUF_SZ (1024)

//void	OsInits (void);

///*******************************************************************************/ 
//// FCMD Processes

// RTC commands
void  _time_cmd   (uint8_t *cmdbuffer);
void	time_show		(void);
void	time_set		(uint8_t *cmdbuffer);

// SPI Flash commands
void	_logread_cmd			(uint8_t *cmdbuffer);
void	_chip_erase_cmd   (uint8_t *cmdbuffer);
void	_sector_erase_cmd	(uint8_t *cmdbuffer);

// temperature and humidity commands
void  _tmprt_cmd  (uint8_t *cmdbuffer);
void  _hmdty_cmd  (uint8_t *cmdbuffer);



// gerneral commands
void	Help                (uint8_t *cmdbuffer);

//void	WiFi				(uint8_t *cmdbuffer);
void	Wifilisten          (uint8_t *cmdbuffer);

///*******************************************************************************/ 
//// Configurations
//void	RCC_Config 			(void);
//void  I2C2_Config     (void);
//void	ISR_Config			(void);
//void	RTC_Config			(void);
//void	Systick_EN			(int tick);
//uint32_t getTick			(void);
//void	wifi_init 			(void);

//// Pheripheral Funtions
//void	WIFI_wakeup			(void);
//int		wifi_recieve		(void);
//void	wifi_cmd			(uint8_t *pdata);
//void	wifi_init_cmds		(void);
//void	wifi_Connect		(void);
//int		wifi_cmd_send		(uint8_t *pdata);
//void	_wifi_send			(const char *format, ...);
//int		_wifi_read			(void);

//void	time_UTC			(int epoch);

//void	OsInits				(void);



//// The FreeRTOS task functions prototype
//void    rtos_Config         (void);
//void    send1Task           (void *argument);
//void    send2Task           (void *argument);
//void    CmderStatusPoll     (void *argument);
//void    led1Task            (void *argument);
//void    led2Task            (void *argument);
//void    TimeTerminal        (void *argument);


//void	delayS				(int Seconds );
//void	delayMs				(int MilliSecond);
//void	halt				(int num);
//void	Welcome				(void);
//void	line				(int leng);
//void	BckSpc				(uint8_t Line);
//void	nl					(int Count);
//void	leds				(void);
//void	led1				(void);
//void	led2				(void);
//void    togglepin 		    (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);



//// Interrupts
//void	EXTI15_10_IRQHandler	(void);
//void	USART1_IRQHandler		(void);

#endif  /* PROCESSES_H_ */
