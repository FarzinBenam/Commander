/**
  
  * @file			FS
  * @author		Farzin M.Benam
  * @version	
  * @date     2021-08-21
  * @brief		log file system
  *             
  *******************************************************************************/ 
#include "fs.h"

/* Exported constants --------------------------------------------------------*/
extern const char	_TEMPERATURE_HUMIDITY_FOLDER_NAME[];
extern const char	start_tag[];
extern const char	end_tag[];
extern const char	_Error[];
/* Private define ------------------------------------------------------------*/
#define		T_H_FOLDER_START_ADDR	0x00//QSPI_START_ADDRESS
#define		T_H_FOLDER_END_ADDR		MX25R6435F_FLASH_SIZE//0x120000
#define		T_H_MAX_LOG_SIZE			50
//#define		T_H_LOG_LENGTH				23

//static uint32_t _fs_search_free_sector (fs_folder_def *fsfolderint, int size);
static uint32_t fs_search_last_log (fs_folder_def *fsfolder, uint32_t log_req);
static void     T_H_struct_init (fs_folder_def *fsfolder);
static void     T_H_log_init (fs_folder_def *fsfolder);
static uint8_t  T_H_log_decode (uint8_t *buffer, uint32_t address, uint32_t length);
static void     buffer_clear  (uint8_t *buffer, uint32_t length);

union T_H_LOG_Parameters{
	uint32_t	sensor_32b;
	uint8_t		sensor_8b;
};
union char_to_16hex{
	uint16_t	num16bit;
	uint8_t		nim8bit[2];
};

static uint8_t QSPI_Buffer[MX25R6435F_SECTOR_SIZE];
static fs_folder_def fsfolder;

/* LogFS Functions -----------------------------------------------------------*/
/**
  * @brief  
  * @param  None
  * @retval None
  */
uint8_t fs_init  (void)
{
  uint8_t status = SUCCESS;
  /*
    - finding the number of logs
    - saving the address of the first log
    - saving the address of the last log
    - saving the number of the valid logs
    
  */
  time_show();
  T_H_struct_init(&fsfolder);
  time_show();
  
  return status;
}
uint8_t fs_log (char *data)
{
	uint8_t status = SUCCESS;
	uint8_t temp;
	uint32_t address = fsfolder.LastLogAddr;
	fs_lengths_32_8 length;
	
	// check if the struct configured before
  if(fsfolder.StartAddr >= fsfolder.EndAddr){
    terminal("\nLogFS struct has not beed configured!");
    return ERROR;
  }
  // check for the end of spi flash memory and folder end address
	if (address>= MX25R6435F_FLASH_SIZE || address>= fsfolder.EndAddr){
		terminal("\nEnd Address reached!\nEither Memory or Folder is FULL!\n");
		return ERROR;
	}
  
	// adding header to the log
	// sending the start tag
	status = qspi_WriteMemory((uint8_t *)start_tag, address, 2);
	
	// sending the length of the string
	if(status == SUCCESS){
		address += strlen(start_tag);
		length.length32 = strlen(data);
		status = qspi_WriteMemory((uint8_t *)length.length08, address, 4);
	} else {
    terminal("\nwrite error: start_tag");
  }
	
	// sending the payload
	if(status == SUCCESS){
		address += (4 + 1); // that 1 is for the option byte
		length.length32 = strlen(data);
		status = qspi_WriteMemory((uint8_t *)data, address, length.length32);

	}else {
    terminal("\nwrite error: length");
  }
	
	// sending the CRC
	if(status == SUCCESS){
		address += length.length32;
		temp = crc_8bit((uint8_t*)data, length.length32); // CRC Calculation
		status = qspi_WriteMemory(&temp, address, 1);
	}else {
    terminal("\nwrite error: payload");
  }
	
	// sending the End tag
	if(status == SUCCESS){
		address++;
		status = qspi_WriteMemory((uint8_t *)end_tag, address, 2);
		address += 2;
	}else {
    terminal("\nwrite error: crc");
  }
  
  
	status = qspi_ReadMemory(QSPI_Buffer, fsfolder.LastLogAddr, 24);
		
	if(status == SUCCESS){
//    nl(1);
//		for(int i = 0; i < 24; i++){
//			terminal("%.2X ", QSPI_Buffer[i]);
//		}
    fsfolder.LastLogAddr = address;
	}else {
    terminal("\nwrite error: end_tag");
  }
	
  return status;
}









/**
  * @brief  
  * @param  *fsfolder   related log folder ptr
	* @param  log_req			> 0 if searching for a specific log (log nummber 0 is undefined!)
  * @retval None
  */
static  uint32_t  fs_search_last_log			(fs_folder_def *fsfolder, uint32_t log_num)
{
	uint32_t address = fsfolder->StartAddr;
	uint32_t count = 0, dbug_count1 = 0, dbug_count2 = 0;
	volatile uint32_t null_count = 0;
	uint16_t buf_idx;
	uint8_t tag_idx = 0, status;
	char tags[4];	
	

	// gather start and end tag in an array to search for them
	sprintf(tags, "%s%s", start_tag, end_tag);
	
	//fsfolder->LastLogAddr = fsfolder->StartAddr;
	fsfolder->LogLength = 0;
	
	
  while(address < fsfolder->EndAddr){
    // clear the buffer to 0x00
		//memset(QSPI_Buffer, 0, sizeof(QSPI_Buffer));
    
    // read a page of qspi
		if(qspi_ReadMemory(QSPI_Buffer, address, MX25R6435F_SECTOR_SIZE) != SUCCESS){
			terminal("\n%s_fs_search_last_log-> qspi_ReadMemory", _Error);
			terminal("\naddress: 0x%X", address);
			return ERROR;
		}
    
    // check the first 10 bytes, if they were 0xFF, skip that page (means empty page)
    buf_idx = 10; 
    while(buf_idx){
      if(QSPI_Buffer[buf_idx--] != 0xFF){
        break;
      }
    }
    if(buf_idx == 0){
      //terminal("%d_",buf_idx);
      address += MX25R6435F_SECTOR_SIZE;
      continue;
    }
  
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  buf_idx = 0; 
    // searching for the tags (start and end)
    //nl(1);
		while(buf_idx < MX25R6435F_SECTOR_SIZE){
     // terminal("%X ", QSPI_Buffer[buf_idx]);
      
			// searching for the start and end tags
			if(QSPI_Buffer[buf_idx++] == tags[tag_idx]){
        ++tag_idx;
        if(tag_idx == 1){
          continue;
        }
        ++count;
      //  nl(1);
        
        buf_idx += 22;
      }
      tag_idx = 0;
    }
    address += MX25R6435F_SECTOR_SIZE;
  }
  
  terminal("\naddress: %X", address);
  
  return count;
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  
/*     
		buf_idx = 0; 
    // searching for the tags (start and end)
		while(buf_idx < MX25R6435F_SECTOR_SIZE){
			
			// searching for the start and end tags
			if(QSPI_Buffer[buf_idx] == tags[tag_idx]){
        #if(qspi_debug == 2)
				terminal("\ntag%d_%X", tag_idx, QSPI_Buffer[buf_idx]);
				#endif
				
				++tag_idx;
				// check if the tags are exactly one after another
				if((tag_idx == 1) || (tag_idx == 3)){
					++buf_idx;
					continue;
				}
         if(tag_idx == 2){
          buf_idx += 20;
          continue;
        } 
				#if(qspi_debug == 2)
				nl(1);
				#endif
				
				if(tag_idx == 4)		// if both tags are found save the next free logs address	
				{
					++count;
					tag_idx = 0;
					
          #if(qspi_debug >= 1)
          terminal(" - Log Nr:%d   buf_idx:%.4d   address: 0x%X \n", count, buf_idx, address);
          #endif

					
					if(count == log_num){                                 // finding the desired log
            terminal("\nLog %d address = 0x%X\n", log_num, fsfolder->LastLogAddr);
						
            status = T_H_log_decode(QSPI_Buffer, fsfolder->LastLogAddr, fsfolder->LogLength + 4);	// +4 because i dont count tags here
            return status;          
          }
					fsfolder->LastLogAddr = (address + buf_idx + 1);  // +1 because i in buf_idx at the end
					fsfolder->LogLength = 0;
				}
			}
      else {
				// check if the tags are exactly one after another
				if(tag_idx == 1){
					tag_idx = 0;
					#if(qspi_debug == 2)
					terminal("\nhaha");
					terminal("\n- Log Nr:%d   buf_idx:%.4d   address: 0x%X \n", count, buf_idx, address);
					#endif
					
					++dbug_count1;
					continue;
				} 
        // sometimes CRC value is equal with the the 3th tag value (0x60), so we should skip it, 
        // advice: change the CRC position in the log packet to reduce the accurance of such problem
				else if(tag_idx == 3){
					tag_idx = 2;
					#if(qspi_debug == 2)
					terminal("\nhuhu");
					terminal("\n - Log Nr:%d   buf_idx:%.4d   address: 0x%X \n", count, buf_idx, address);
					#endif
					
					++dbug_count2;
					continue;
				}
				#if(qspi_debug == 2)
				terminal("%X ",QSPI_Buffer[buf_idx]);
				#endif
			}
      

			if(tag_idx == 2){
        fsfolder->LogLength++;
      }
			if(fsfolder->LogLength > T_H_MAX_LOG_SIZE){
				terminal("hoho");
				while(1);
			}
			
			++buf_idx;
		}
    #if(qspi_debug >= 1)
		terminal("\n\n_____________________________________________________________________\nAddress inc\n");
    #endif
		address += MX25R6435F_SECTOR_SIZE;
	}
	
	terminal("\ncount: %d, dbug_count1:%d,  dbug_count2:%d", count, dbug_count1, dbug_count2);
	
	// if log_num requested, but not found
	if(log_num){
		terminal("\nlog %d not found!", log_num);
		return ERROR;
	}
	return count; */
}
/**
  * @brief  
  * @param  None
  * @retval None
  */
/**
  * @brief  
  * @param  None
  * @retval None
  */



/* Temperature and Humidity logging Functions  -------------------------------*/
/**
  * @brief  
  * @param  *fsfolder   related log folder ptr
  * @retval None
  */
static void	T_H_struct_init (fs_folder_def *fsfolder)
{
//	strcpy(fsfolder->Name, _TEMPERATURE_HUMIDITY_FOLDER_NAME); // be carefull with the size of array
//  strcpy(fsfolder->Commant, "the folder used for stroing the temperature and Humidity logs");
	fsfolder->StartAddr = T_H_FOLDER_START_ADDR;
	fsfolder->EndAddr = 	T_H_FOLDER_END_ADDR;
  fsfolder->LogCount = fs_search_last_log(fsfolder, 0);
  
	terminal("\nLog Count: %d\nLast log : 0x%X", fsfolder->LogCount, fsfolder->LastLogAddr);
}
/**
  * @brief  
  * @param  None
  * @retval None
  */
void	T_H_log		(fs_folder_def *fsfolder)
{
	LL_RTC_TimeTypeDef time;
	LL_RTC_DateTypeDef date;
	union T_H_LOG_Parameters sensor;
	uint8_t	temp[3];
	uint16_t str_length;
	
  // gathering the temperature, humidity and clock data
	sensor.sensor_32b = HTS221_T_ReadTemp(HTS221_SLAVE_ADD);
	temp[0] = sensor.sensor_8b;

	sensor.sensor_32b = HTS221_H_ReadHumidity(HTS221_SLAVE_ADD);
	temp[1] = sensor.sensor_8b;
	
	rtc_read(&time, &date);
  
  // making the payload data
	sprintf(	(char*)QSPI_Buffer, "%.2X%.2X%.2X%.2X%.2X%.2d%.2d",
						time.Hours,
						time.Minutes,
						date.Year,
						date.Month,
						date.Day,
						temp[0],
						temp[1]);

	str_length = strlen((char*)QSPI_Buffer);
	//terminal("\npayload:\n%s    length:%d\n", QSPI_Buffer, str_length);
  
	
	fs_log((char*)QSPI_Buffer);
}
/**
  * @brief  
  * @param  None
  * @retval None
  */
uint8_t  T_H_log_finder (fs_folder_def *fsfolder, uint32_t log_num)
{
  // log nummber 0 is undefined!
  if(log_num ==  0){
    terminal("\nlog 0 is undefined!");
    return ERROR;
  }

  if(fs_search_last_log(fsfolder, log_num) != SUCCESS){
		return ERROR;
	}
  return SUCCESS;
  
}
static  uint8_t   T_H_log_decode (uint8_t *buffer, uint32_t address, uint32_t length)
{
  uint8_t status = SUCCESS;
  uint32_t buf_idx = 0, payload_idx = 0;
  fs_lengths_32_8 payload_length;
  uint8_t payload[length];
  
  //terminal("\npayload at addr: 0x%X", address);
  // clear the buffer to 0x00
	memset(QSPI_Buffer, 0, sizeof(QSPI_Buffer));
	
  status = qspi_ReadMemory(QSPI_Buffer, address, length);
  
//  while(buf_idx < length){
//    terminal("%.2X ",QSPI_Buffer[buf_idx++]);
//  }
  if(status == SUCCESS){
		buf_idx = 0;
		if(QSPI_Buffer[buf_idx++] != start_tag[0]) return ERROR;
		if(QSPI_Buffer[buf_idx++] != start_tag[1]) return ERROR;
		payload_length.length08[0] = QSPI_Buffer[buf_idx++];
		payload_length.length08[1] = QSPI_Buffer[buf_idx++];
		payload_length.length08[2] = QSPI_Buffer[buf_idx++];
		payload_length.length08[3] = QSPI_Buffer[buf_idx++];
		if(QSPI_Buffer[buf_idx++] != 0xFF){
			terminal("\nLog not accessible - Deleted!");
			return ERROR;      // deleted log
		}
		
		while(payload_idx < payload_length.length32){
			payload[payload_idx++] = QSPI_Buffer[buf_idx++];
		}
		if(crc_8bit(payload, payload_length.length32) != QSPI_Buffer[buf_idx++]){ // CRC Calculation
			terminal("\nLog not valid - CRC Error!");
			return ERROR;      // CRC not match
		}
		if(QSPI_Buffer[buf_idx++] != end_tag[0]) return ERROR;
		if(QSPI_Buffer[buf_idx] != end_tag[1]) return ERROR;
		
		payload_idx = 0;
		
		//year
		terminal("\n20%c%c.", payload[4], payload[5]);
		//month
		terminal("%c%c.", payload[6], payload[7]);
		//day
		terminal("%c%c ", payload[8], payload[9]);
		
		//hour
		terminal("__ %c%c:", payload[0], payload[1]);
		//minute
		terminal("%c%c", payload[2], payload[3]);
		
	 
		
		//temperature
		terminal("\nT:%c%c C _ ", payload[10], payload[11]);
		//humidity
		terminal("H:%c%c %%", payload[12], payload[13]);
		
  }
  
  
  return status;
}

/******************* (C) COPYRIGHT 2021 Farzin_M.Benam *****END OF FILE****/
