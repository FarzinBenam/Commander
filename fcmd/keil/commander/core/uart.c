/**
  
  * @file			
  * @author		Farzin M.Benam
  * @version	
  * @date     2020-12-18
  * @brief		
  *             
  *******************************************************************************
 The serial interface USART1 is directly available as a Virtual COM port of the PC 
 connected to the ST-LINK/V2-1 USB connector CN7. The Virtual COM port settings 
 are configured as: 115200 b/s, 8 bits data, no parity, 1 stop bit, no flow control.
 
 USART1_TX	PB6	AF7(as alternate fucntion AF7)
 USART1_RX	PB7	AF7(as alternate fucntion AF7)
 
 UART4_TX	PA0	AF8(as alternate fucntion AF8)
 UART4_RX	PA1	AF8(as alternate fucntion AF8)
 
 USR_BUTTON	PC13
  *******************************************************************************/ 
#include "uart.h"


/*******************************************************************************
 * USART Config Functions
 *******************************************************************************/
uint32_t	_usart_read (USART_TypeDef *USARTx)
{
	while(!(USARTx->ISR & USART_ISR_RXNE));					// RXNE: Read data register not empty
	
	return USARTx->RDR;
}
uint32_t	_usart_send_b	(USART_TypeDef *USARTx, int ch) 
{    
  while(!(USARTx->ISR & USART_ISR_TXE));					// TXE: Transmit data register empty
  USARTx->TDR = (ch & (uint16_t)0x01FF);

  return ch;
}
void			_usart_send_s	(USART_TypeDef *USARTx,const char Message[])
{
	volatile int i = 0;

	while(Message[i]){
		while(!(USARTx->ISR & USART_ISR_TXE));
		USARTx->TDR = Message[i++];
	}
}










/******************************************************************************* 
 * interface to the stdio.h library.
 * All the I/O directed to the console.
 * after this we can use the stdio functions like: printf
********************************************************************************/
//to avoid redifintion of the __FILE we comment this session
//struct	__FILE {
//	int handle;
//	 /* Whatever you require here. If the only file you are using is
//		* standard output using printf() for debugging, no file handling
//		* is required. */
//};
#define STDIN   0
#define STDOUT  1
#define STDERR  2
#define LF   '\n'
#define CR   '\r'

FILE __stdin =  {STDIN};
FILE __stdout = {STDOUT};
FILE __stderr = {STDERR};

int fgetc (FILE *f){
    int c;
    c = _usart_read(terminal_usart);          // read the character from console
    
    if (c == CR){                             // if \r, after it is echoed, a \n sent
        _usart_send_b(terminal_usart, c);     // echo
        c = LF;
    }
    _usart_send_b(terminal_usart, c);         // echo
    
    return c;
}

int fputc (int c, FILE *f){
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  return _usart_send_b(terminal_usart, c);    // write the character to console
}



 /******************* (C) COPYRIGHT 2021 Farzin_M.Benam *****END OF FILE****/
