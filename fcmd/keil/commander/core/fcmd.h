#ifndef FCMD_H_
#define FCMD_H_

#include "main.h"

/**
  * @brief Commander defines
  */
#define MAX_COMMANDS        20
#define CMD_BUFFER_SIZE     32
#define IIC_BUFFER_SIZE     512
#define	WIFI_BUFFER_SIZE    1024
#define GENERAL_BUFFER_SIZE	200
																								
#define ENTER         0x0D
#define SPACE         ' '
#define STX           0x02
#define BACKSPACE1    0x08
#define BACKSPACE2    0x7F
#define CommandSign   '>'
#define STAR          '*'
#define UNDERLINE     '_'
#define SCAPE_BUT     '`'

//************************************************************************
//*	CMD declarations and related parameters on the PROM                  
//************************************************************************
// Kernel Specific Constants
// General commands
static const char _CMD_help[]		      = "HELP";
static const char _CMD_clock_speed[]  = "CLOCK";

// RTC commands
static const char _CMD_time[]		    = "TIME";
static const char _CMD_date[]		    = "DATE";
static const char _CMD_alarm[]	    = "ALARM";

// WiFi commands
static const char _CMD_wifi[]		    = "WIFI";
static const char _CMD_wifilisten[] = "WIFILISTEN";

// SPI Flash commands
static const char _CMD_logread[]    = "LOGREAD";
static const char _CMD_chip_erase[] = "CHIPERASE";
static const char _CMD_sec_erase[]  = "SECERASE";

// Temperature & Humidity sensor commands
static const char _CMD_tmprture[]   = "TEMP";
static const char _CMD_humidity[]   = "HUM";

static const char _CMD_tmp[]		    = "TMP";
static const char _CMD_lcd[]		    = "LCD";
static const char _CMD_tmpbck[]	    = "TMPBCK";
static const char _CMD_tmprep[]	    = "TMPREP";
static const char _CMD_tmpbckoff[]	= "TMPBCKOFF";
static const char _CMD_tmpbckread[]	= "TMPBCKREAD";

//******************************************
// Kernel Specific Constants
static const char wc_note[]			= "WELCOME TO FCMD";
static const char CmdError_1[]	= "[!] unknown Command!"; //Enter HELP for more information.";
static const char CmdError_2[]	= "[!] Wrong Input!";
static const char UpatedNote[]	= "updated!";
static const char New_Line[]		= "\n\r";

// Temperature Specific Constants
static const char TMPNote[]			= "Temperature";
//static unsigned char month_days[12]	= {31,28,31,30,31,30,31,31,30,31,30,31};
//static unsigned char week_days[7]	= {4,5,6,0,1,2,3};






//************************************************************************
//* Specific Structure Definitions
//************************************************************************
typedef struct{
	uint8_t	ProgramStatus :1;
	//CmdStatus = 0 -> normal status
	//CmdStatus = 1 -> command Entered
	//CmdStatus = 2 -> cmd without parameter
	//CmdStatus = 3 -> cmd with parameter
	//CmdStatus = 4 -> Usart RX intterupt
	//
	//
	//
	uint8_t	CmdStatus :3;
	uint8_t	CmdProcessStatus :1; 
	uint8_t	System1SecStatus1 :1;
	uint8_t	System1SecStatus2 :1;
	uint8_t	Unused:1;
}Status;
typedef struct{
  // Pointer to a CMD
	const char *cmd_name;
	/* function pointer Declaration:
  ** return_type (*function_name)(arguments)    
  */
	void (*cmd_func_ptr)(uint8_t buffer[]);  
} Commands;

//************************************************************************
//* FCMD Function Definitions
//************************************************************************
uint8_t USART_Process		(void);
void	cmd_process_inits	(void);
uint8_t add_command		(const char *cmd, void (*function_ptr)(uint8_t buffer[]));
void	cmd_entry			  (void);
void	cmd_exit			  (void);
uint8_t	cmd_decode		(uint8_t *cmd, const char *OsCmds);
uint8_t	param_decode		(uint8_t *cmd);
#endif  /* FCMD_H_ */
