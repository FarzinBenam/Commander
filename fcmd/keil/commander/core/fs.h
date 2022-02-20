#ifndef _FS_H
#define _FS_H

#ifdef __cplusplus
 extern "C" {
#endif



#include "main.h"


/*******************************************************************************
 * defintions
 *******************************************************************************/
#define   QSPI_START_ADDRESS  0x8000
#define		FS_FOLDER_START_TAG	
/* Exported constants --------------------------------------------------------*/
typedef union{
	uint8_t		length08[4];
	uint32_t	length32;
}fs_lengths_32_8;

typedef struct{
  char		*buffer;
  uint8_t	length;
}fs_init_def;

typedef struct{
  char      Name[10];			// Name of the folder
  char      Commant[200]; // descreption of the folder
	uint32_t	StartAddr;		// starting address of the folder on spi flash
	uint32_t	EndAddr;			// end address of the folder on spi flash
	uint32_t	LastLogAddr;	// address of the last log
	uint8_t   LogLength;
  uint32_t  LogCount;
  

}fs_folder_def;



/* Exported functions --------------------------------------------------------*/ 
uint8_t   fs_init         (void);
uint8_t	  fs_log          (char *data);
uint8_t	  fs_new_folder   (fs_folder_def *fsfolder);

void		  T_H_log         (fs_folder_def *fsfolder);
uint8_t   T_H_log_finder  (fs_folder_def *fsfolder, uint32_t log_num);


#endif

