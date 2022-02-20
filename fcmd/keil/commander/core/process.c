#include "main.h"



/************************************************************************
 *	Definitions
 ************************************************************************/
extern volatile int tick;
extern volatile int _tick;
//volatile int debug = 0;


extern Status	SysStatus;
extern volatile uint8_t	WifiStatus;
extern volatile uint8_t	wificmdStatus;
extern Commands	Cmds[MAX_COMMANDS]; // array of structures

static uint8_t	GernealBuffer[GENERAL_BUFFER_SIZE];
//static uint8_t	IICBuffer[IIC_BUFFER_SIZE];
static uint8_t	WifiBuffer[WIFI_BUFFER_SIZE];
//I2C_HandleTypeDef hi2c;

//// create mutex handle
//SemaphoreHandle_t uartMutex;
//SemaphoreHandle_t CmdStatusMutex;


/************************************************************************
 * System Processes
 ************************************************************************/
/**
  * @brief  
  * @param  None
  * @retval None
  */
void	_time_cmd (uint8_t *cmdbuffer)
{
	
  // CmdStatus = 2 -> cmd without parameter
	if (SysStatus.CmdStatus == 2){   
    time_show();
  }
	// CmdStatus = 3 -> cmd without parameter
	if (SysStatus.CmdStatus == 3){
		time_set(cmdbuffer);
	}
	
	cmd_exit();
}
/**
  * @brief  
  * @param  None
  * @retval None
  */
void	time_show (void)
{
	LL_RTC_TimeTypeDef	time;
	LL_RTC_DateTypeDef	date;
	
	rtc_read(&time, &date);

	terminal("\n%.2X:%.2X:%.2X ",time.Hours, time.Minutes, time.Seconds);
	if(RTC->TR & RTC_TR_PM){
		terminal("PM");
	}
	terminal("__ 20%.2X-%.2X-%.2X ", date.Year, date.Month, date.Day);
	switch(date.WeekDay){
		case LL_RTC_WEEKDAY_MONDAY:
			terminal("Mon");
			break;
		case LL_RTC_WEEKDAY_TUESDAY:
			terminal("Tues");
			break;
		case LL_RTC_WEEKDAY_WEDNESDAY:
			terminal("Wednes");
			break;
		case LL_RTC_WEEKDAY_THURSDAY:
			terminal("Thurs");
			break;
		case LL_RTC_WEEKDAY_FRIDAY:
			terminal("Fri");
			break;
		case LL_RTC_WEEKDAY_SATURDAY:
			terminal("Satur");
			break;
		case LL_RTC_WEEKDAY_SUNDAY:
			terminal("Sun");
			break;
		
	}
	terminal("day");
}
/**
  * @brief  just read the rtc and show the time
  * @param  None
  * @retval None
  */
void	time_set (uint8_t *cmdbuffer)
{
  LL_RTC_TimeTypeDef time;
	LL_RTC_DateTypeDef date;
  uint8_t buff_idx = 0, i;
	char temp[20];
	int temp1[10];

	unsigned int x, y , z;
	char *ptr;
	
	// clear the time & date structs
	memset(&time, 0, sizeof(time));
	memset(&date, 0, sizeof(date));
	memset(&temp1,0, sizeof(temp1));
	
  
	/* format of setting time:
	** hour minute second year month day weekday
	** example: time 23 15 00 21 10 26 3
	** always SPACE in between numbers
	*/
  /*  !!!!!!!! this method is also true !!!!!!
	sscanf((const char *)cmdbuffer, "%s %d %d %d %d %d %d %d",temp, 
																														&temp1[0], 
																														&temp1[1], 
																														&temp1[2], 
																														&temp1[3],
																														&temp1[4],
																														&temp1[5],
																														&temp1[6]);	
	i = 0;
	time.Hours    = temp1[i++];
	time.Minutes  = temp1[i++];
	time.Seconds	= temp1[i++];
	date.Year			= temp1[i++];
	date.Month		= temp1[i++];
	date.Day			= temp1[i++];
	date.WeekDay	= temp1[i++];
  */
	
  // both above and under method can be applied
  atoi((const char *)strtok((char *)cmdbuffer, " "));     // to skip ke cmd it self
  time.Hours    = atoi((const char *)strtok(NULL, " "));  // NULL as an input mean to countine from where it left
	time.Minutes  = atoi((const char *)strtok(NULL, " "));
	time.Seconds	= atoi((const char *)strtok(NULL, " "));
	date.Year			= atoi((const char *)strtok(NULL, " "));
	date.Month		= atoi((const char *)strtok(NULL, " "));
	date.Day			= atoi((const char *)strtok(NULL, " "));
	date.WeekDay	= atoi((const char *)strtok(NULL, " "));
  
  
//	// use to DEBUG
//	terminal("\n%d %d %d %d %d %d %d",time.Hours, 
//																		time.Minutes,
//																		time.Seconds,
//																		date.Year,
//																		date.Month,
//																		date.Day,
//																		date.WeekDay);
	
  rtc_set(&time, &date);
  time_show();
}
/**
  * @brief  read the given log number
  * @param  log number
  * @retval None
  */
void	_logread_cmd (uint8_t *cmdbuffer)
{
	fs_folder_def fsfolder;
	uint32_t log_num;

	fsfolder.StartAddr = 0;
	fsfolder.EndAddr = 	MX25R6435F_FLASH_SIZE;
	
  // CmdStatus = 3 -> cmd without parameter
	if (SysStatus.CmdStatus == 3){
    
    log_num = param_decode(cmdbuffer);    // extracting the nummber enterred from terminal
    
		if(T_H_log_finder(&fsfolder, log_num) != SUCCESS){
			terminal("\nT_H_log_finder: ERROR!");
		}
  }
  else {
    terminal("\nNo log nummber entered!"); 
  }

	cmd_exit();
}
/**
  * @brief  Full chip erase of the SPI Flash
  * @param  None
  * @retval None
  */
void	_chip_erase_cmd	(uint8_t *cmdbuffer)
{
  qspi_Erase_Chip();
  cmd_exit();
}
/**
  * @brief  Sector erase of the SPI flash
  * @param  Sector address
  * @retval None
  */
void	_sector_erase_cmd	(uint8_t *cmdbuffer)
{
  uint32_t sector_add = param_decode(cmdbuffer);
  
  
  qspi_EraseSector( sector_add/ MX25R6435F_SECTOR_SIZE);
  cmd_exit();
}
/**
  * @brief  temporature
  * @param  None
  * @retval None
  */
void	_hmdty_cmd (uint8_t *cmdbuffer)
{
	float i = HTS221_H_ReadHumidity(HTS221_SLAVE_ADD);
  terminal("\nHumdity:%.2f %%", i);
	cmd_exit();
}
/**
  * @brief  humidity 
  * @param  None
  * @retval None
  */
void	_tmprt_cmd (uint8_t *cmdbuffer)
{
	float i = HTS221_T_ReadTemp(HTS221_SLAVE_ADD);
  terminal("\nTemperature:%.2f `C", i);
  
	cmd_exit();
}

void    Help            (uint8_t *cmdbuffer)
{

  nl(1);
  terminal("List of commands: ");
  nl(1);
  
  for(int i = 0; i < 10; i++){
    nl(1);
    terminal("%s", Cmds[i].cmd_name);
  }
  
  cmd_exit();
}
/**
  * @brief  
  * @param  None
  * @retval None
  */
void	Wifilisten (uint8_t *cmdbuffer)
{
  uint8_t temp = 0;
  uint16_t idx = 0;
  
	__disable_irq();
  
  terminal("\n_____Wifi Manual AT-Command_____\n");
	
  while(1){
    
    
    if(terminal_usart->ISR & USART_ISR_RXNE){   // wait for the terminal
      temp = terminal_usart->RDR;               // reading the terminal
    
    
      if(temp == SCAPE_BUT){                    // if '`' reveived, exit the manual mode
          nl(1);
          terminal("Exiting Command Mode");
          nl(1);
          break;
      }
      else if (temp == ENTER){                  // Enter reveced, so time to read the wifi module
        
        _usart_send_b(terminal_usart, temp);      // Terminal Echo (CR, LF)
        _usart_send_b(terminal_usart, '\n');
        terminal_usart->RQR |= (1<<3);
        
        
        WifiBuffer[idx++] = temp;                   // buffer the reveived data
        WifiBuffer[idx++] = '\n';                   // buffer the reveived data
        
        if(SPI_WIFI_SendData((uint8_t *)WifiBuffer, idx, 10)){        // sending the ENTER
          FCMD_SPI_WIFI_ReceiveData(&temp, 0, 10);     // reading the wifi module
          idx = 0;
          continue;
        }
      }
      
      
      _usart_send_b(terminal_usart, temp);      // Terminal Echo
      terminal_usart->RQR |= (1<<3);  
      
      if(idx < WIFI_BUFFER_SIZE){
        WifiBuffer[idx++] = temp;                   // buffer the reveived data
      }
      
      
      
      LED2_PORT->ODR ^= LED2_PIN;
    }
  }
  
  __enable_irq();
  cmd_exit();
}






// new process prototype function
/** 
  * @brief  
  * @param  None
  * @retval None
  */
//void	XXXX (uint8_t *cmdbuffer)
//{
//	
//	cmd_exit();
//}
/***************************************************************END OF FILE****/
