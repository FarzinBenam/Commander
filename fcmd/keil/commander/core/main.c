/*
  ******************************************************************************
  * @file
  * @author		Farzin M.Benam
  * @version	v.1 ARM STM32L4
  * @date			2020-06-20
  * @brief
	*						Commander Project
  *******************************************************************************
	* @attention
  *
	* Copyright (C) 2010-2020 Farzin Memaran Benam <Farzin.Memaran@gmail.com>
	*
	* This file is part of Promzit Commander project.
	*
	* This project can not be copied and/or distributed without the express
	* permission of Farzin Memaran Benam.
  *
	* Copyright (C) Promzit Systems, Inc - All Rights Reserved
  * <h2><center>&copy; COPYRIGHT 2020 Farzin_M.Benam</center></h2>
	*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "main.h"

//************************************************************************
//* Specific Variable Definitions
//************************************************************************
/*
Global status variable
	CmdStatus = 0 -> normal status
	CmdStatus = 1 -> usart int
	CmdStatus = 2 -> command Entered
	CmdStatus = 3 -> cmd without parameter
	CmdStatus = 4 -> cmd with parameter
	CmdStatus = 5 -> wifi ready

	WifiStatus = 0 -> wifi off
	WifiStatus = 1 -> wifi initiated and ready
	WifiStatus = 2 -> wifi recieving respond
	wificmdStatus = 0 -> no wifi command
	wificmdStatus = 1 -> wifi interrupt recieved
	*/
Status SysStatus;
uint8_t WifiStatus = 0;



/* Private typedef -----------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Constant definitions ------------------------------------------------------*/



int main (void)
{
  SysStatus.CmdStatus = 0;
  
  Configs();
  
  terminal("\nval: %d", POSITION_VAL(20));
  
  
//  wifi_command("$$$");
//  wifi_command("F?");
//  wifi_command("A?");
//  wifi_command("I?");
//  wifi_command("F?");
//  wifi_command("---");
  
	while(1){
    if (SysStatus.CmdStatus == 1) USART_Process();
    if (SysStatus.CmdStatus == 2)	cmd_entry();
	}

}


 /******************* (C) COPYRIGHT 2020 Farzin_M.Benam *****END OF FILE****/
