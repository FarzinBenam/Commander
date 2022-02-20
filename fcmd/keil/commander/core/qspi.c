 
 
 /**
  * @file			
  * @author		Farzin M.Benam
  * @version	
  * @date     2021-08-23
  * @brief		
  *             
  *******************************************************************************

  *******************************************************************************/ 
#include "qspi.h"

/* Private define ------------------------------------------------------------*/
extern const char	_Error[];
/** @defgroup QSPI_Private_Constants QSPI Private Constants
  * @{
  */
#define MEMORY_SECTOR_SIZE                  MX25R6435F_SECTOR_SIZE
#define MEMORY_PAGE_SIZE                    MX25R6435F_PAGE_SIZE
#define QSPI_TIMEOUT												500

#define QSPI_QUAD_DISABLE       0x0
#define QSPI_QUAD_ENABLE        0x1

#define QSPI_HIGH_PERF_DISABLE  0x0
#define QSPI_HIGH_PERF_ENABLE   0x1

#define QSPI_FUNCTIONAL_MODE_INDIRECT_WRITE 0x00000000U                     /*!<Indirect write mode*/
#define QSPI_FUNCTIONAL_MODE_INDIRECT_READ  ((uint32_t)QUADSPI_CCR_FMODE_0) /*!<Indirect read mode*/
#define QSPI_FUNCTIONAL_MODE_AUTO_POLLING   ((uint32_t)QUADSPI_CCR_FMODE_1) /*!<Automatic polling mode*/
#define QSPI_FUNCTIONAL_MODE_MEMORY_MAPPED  ((uint32_t)QUADSPI_CCR_FMODE)   /*!<Memory-mapped mode*/
/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
static uint8_t qspi_Init(QSPI_InitTypeDef *Init);
static uint8_t qspi_ResetMemory (void);
static void QSPI_Config(QSPI_CommandTypeDef *cmd, uint32_t FunctionalMode);
static void qspi_gpio_Init (void);
static void qspi_Configuration (void);
static uint8_t qspi_AutoPollingMemReady (void);
static uint8_t qspi_AutoPolling (QSPI_CommandTypeDef *cmd, QSPI_AutoPollingTypeDef *cfg);
static void qspi_Read_All_Reg (uint8_t* test_buffer);
static uint8_t qspi_Abort (void);
static uint8_t qspi_WriteEnable(void);
static uint8_t qspi_Command (QSPI_CommandTypeDef *cmd);
static uint8_t qspi_Transmit (uint8_t *pData);
static uint8_t qspi_Receive (uint8_t *pData);
static uint8_t qspi_WaitFlagStateUntilTimeout(__IO uint32_t Register, uint32_t Flag, FlagStatus State, uint32_t Timeout);
static uint8_t qspi_QuadMode(uint8_t Operation);
static uint8_t qspi_HighPerfMode(uint8_t Operation);
static uint8_t qspi_GetStatus(void);

/*******************************************************************************
 * QUADSPI Functions
 *******************************************************************************/
 /* QUADSPI init function */
uint8_t  qspi_config (void)
{
	ErrorStatus status = SUCCESS;
	QSPI_InitTypeDef Init;

  /*  System level initialization **********************************************
  - QUADSPI Initialization:
  - ClockPrescaler = 2, QSPI clock = FAHB / 2+1 = 80MHz / (ClockPrescaler+1) = 26.67MHz
  - FIFO when 8 more bytes written or read
  - don't sample the data read from memory half-clock cycle later
  - flash size = 64Mb = 8MB = 2^(22+1) bytes. 
  - the read and wirte command should CS# high in 30ns
  - clock stay low between two command
  */
  Init.ClockPrescaler     = 2; /* QSPI clock = 80MHz / (ClockPrescaler+1) = 26.67MHz */
  Init.FifoThreshold      = 4;
  Init.SampleShifting     = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  Init.FlashSize          = POSITION_VAL(MX25R6435F_FLASH_SIZE) - 1;
  Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  Init.ClockMode          = QSPI_CLOCK_MODE_0;
	if (qspi_Init(&Init) != SUCCESS)
  {
		terminal("\n%sqspi_Init", _Error);
    return ERROR;
  }
  
	/*  QSPI memory reset   ******************************************************/
	if(qspi_ResetMemory() != SUCCESS){
		terminal("\n%sqspi_ResetChip", _Error);
    return ERROR;
	}
	
	/* QSPI quad enable **********************************************************/
	if(qspi_QuadMode(QSPI_QUAD_ENABLE) != SUCCESS){
		terminal("\n%sqspi_QuadMode", _Error);
    return ERROR;
	}
	/* High performance mode enable **********************************************/
	if (qspi_HighPerfMode(QSPI_HIGH_PERF_ENABLE) != SUCCESS)
  {
		terminal("\n%sqspi_HighPerfMode", _Error);
    return ERROR;
  }
	
	/* Re-configure the clock for the high performance mode */
  Init.ClockPrescaler = 1; /* QSPI clock = 80MHz / (ClockPrescaler+1) = 40MHz */

  if (qspi_Init(&Init) != SUCCESS)
  {
		terminal("\n%sqspi_Init -> Prescaler = 1", _Error);
    return ERROR;
  }
	
  return SUCCESS;
}

/**
  * @brief Set the command configuration.
  * @param cmd : structure that contains the command configuration information
  * @note   This function is used only in Indirect Read or Write Modes
  */
static uint8_t  qspi_Command(QSPI_CommandTypeDef *cmd)
{
	ErrorStatus status = ERROR;
	uint16_t i = 0;
	
  /* Wait till BUSY flag reset */
  while(status != SUCCESS){
    status = qspi_WaitFlagStateUntilTimeout(QUADSPI->SR, QUADSPI_SR_BUSY, RESET, QSPI_TIMEOUT);
  }
    

		
	
	if(status == SUCCESS){
		/* Call the configuration function */
		QSPI_Config(cmd, QSPI_FUNCTIONAL_MODE_INDIRECT_WRITE);
		
		/* make a small delay */
		for (uint8_t i  = 0; i < 100; i++) {
			__NOP();
		}
		
		if (cmd->DataMode == QSPI_DATA_NONE)
		{
			/* When there is no data phase, the transfer start as soon as the configuration is done
			so wait until TC flag is set to go back in idle state */
			status = qspi_WaitFlagStateUntilTimeout(QUADSPI->SR, QUADSPI_SR_TCF, SET, QSPI_TIMEOUT);
			
			/* Clears QSPI_FLAG_TC's flag status. */
			QUADSPI->FCR |= QUADSPI_FCR_CTCF;
		}
	}
	#if(qspi_debug == 1)
	else
	{
		terminal("\n%sqspi_Command - > Busy", _Error);
	}
  #endif
	/* Return function status */
	return status;
}

/**
  * @brief Transmit an amount of data in blocking mode.
  * @param pData : pointer to data buffer
  * @note   This function is used only in Indirect Write Mode
  * @retval HAL status
  */
static uint8_t  qspi_Transmit (uint8_t *pData)
{
	uint8_t status = SUCCESS;
  __IO uint32_t *data_reg = &QUADSPI->DR;
	
  __IO uint32_t   TxSize;       /* QSPI Tx Transfer size              */
  __IO uint32_t   TxCount;      /* QSPI Tx Transfer Counter           */
  uint8_t         *TxBufferPtr; /* Pointer to QSPI Tx transfer Buffer */

  if(pData != NULL )
  {

    /* Configure counters and size of the handle */
    TxCount = READ_REG(QUADSPI->DLR) + 1U;
    TxSize = READ_REG(QUADSPI->DLR) + 1U;
    TxBufferPtr = pData;

    /* Configure QSPI: CCR register with functional as indirect write */
    MODIFY_REG(QUADSPI->CCR, QUADSPI_CCR_FMODE, QSPI_FUNCTIONAL_MODE_INDIRECT_WRITE);

    while(TxCount > 0U)
    {
      /* Wait until FT flag is set to send data */
			status = qspi_WaitFlagStateUntilTimeout(QUADSPI->SR, QUADSPI_SR_FTF, SET, QSPI_TIMEOUT);
			
			if(status != SUCCESS){
				terminal("\n%sqspi_Transmit: QUADSPI_SR_FTF", _Error);
				break;
			}

      *((__IO uint8_t *)data_reg) = *TxBufferPtr;
      TxBufferPtr++;
      TxCount--;
    }
    
		if(status == SUCCESS)
		{
			/* make a small delay */
			for (uint8_t i  = 0; i < 100; i++) {
				__NOP();
			}
			
			/* Wait until TC flag is set to go back in idle state */
			status = qspi_WaitFlagStateUntilTimeout(QUADSPI->SR, QUADSPI_SR_TCF, SET, QSPI_TIMEOUT);
			
			if(status == SUCCESS){
				/* Clear Transfer Complete bit */
				QUADSPI->FCR |= QUADSPI_FCR_CTCF;
				
				 #if  (defined(STM32L471xx) || defined(STM32L475xx) || defined(STM32L476xx) || defined(STM32L485xx) || defined(STM32L486xx))
				/* Clear Busy bit */
				status = qspi_Abort();
				if(status != SUCCESS){
					terminal("\n%sqspi_Transmit -> qspi_Abort", _Error);
				}
				
				#endif
			}
			else{
				terminal("\n%sqspi_Transmit -> QUADSPI_SR_TCF", _Error);
			}
		}
	}
		
	return status;
}


/**
  * @brief Receive an amount of data in blocking mode.
  * @param hqspi : QSPI handle
  * @param pData : pointer to data buffer
  * @param Timeout : Timeout duration
  * @note   This function is used only in Indirect Read Mode
  * @retval HAL status
  */
static uint8_t  qspi_Receive (uint8_t *pData)
{
	uint8_t status;
  uint32_t addr_reg = READ_REG(QUADSPI->AR);
  __IO uint32_t *data_reg = &QUADSPI->DR;
	
  uint8_t                   *RxBufferPtr;	/* Pointer to QSPI Rx transfer Buffer */
  __IO uint32_t             RxSize;       /* QSPI Rx Transfer size              */
  __IO uint32_t             RxCount;  	  /* QSPI Rx Transfer Counter           */

  if(pData != NULL ){
		/* Configure counters and size of QSPI Recieve */
    RxCount = READ_REG(QUADSPI->DLR) + 1U;
    RxSize = READ_REG(QUADSPI->DLR) + 1U;
    RxBufferPtr = pData;

    /* Configure QSPI: CCR register with functional as indirect read */
    MODIFY_REG(QUADSPI->CCR, QUADSPI_CCR_FMODE, QSPI_FUNCTIONAL_MODE_INDIRECT_READ);

    /* Start the transfer by re-writing the address in AR register */
    WRITE_REG(QUADSPI->AR, addr_reg);
		
		/* make a small delay */
		for (uint8_t i  = 0; i < 10; i++) {
			__NOP();
		}
    /* Wait till BUSY flag reset */
    //status = qspi_WaitFlagStateUntilTimeout(QUADSPI->SR, QUADSPI_SR_BUSY, RESET, QSPI_TIMEOUT);

		
    while(RxCount > 0U){
      /* Wait until FT or TC flag is set to read received data */
			status = qspi_WaitFlagStateUntilTimeout(QUADSPI->SR, (QUADSPI_SR_TCF|QUADSPI_SR_FTF), SET, 1000);//QSPI_TIMEOUT);

			if(status != SUCCESS){
				terminal("\n%sqspi_Receive -> FT or TC flag", _Error);
				break;
			}

      *RxBufferPtr = *((__IO uint8_t *)data_reg);
      RxBufferPtr++;
      RxCount--;
    }

    if (status == SUCCESS){
			/* Wait until TC flag is set to go back in idle state */
      status = qspi_WaitFlagStateUntilTimeout(QUADSPI->SR, QUADSPI_SR_TCF, SET, QSPI_TIMEOUT);
			
			if  (status == SUCCESS){
				/* Clear Transfer Complete bit */
				QUADSPI->FCR |= QUADSPI_FCR_CTCF;
				
				/* Workaround - Extra data written in the FIFO at the end of a read transfer */
				status = qspi_Abort();
				if(status != SUCCESS){
					terminal("\n%sqspi_Receive -> QSPI_ERROR_INVALID_PARAM", _Error);
				}					
      }
    }
	}
  else{
		terminal("\n%sqspi_Receive -> qspi_Abort", _Error);
		status = ERROR;
	}
	

  return status;
}


uint8_t  qspi_EraseSector(uint32_t Sector)
{
	QSPI_CommandTypeDef qspiCmd;
	int i = 0;
	while(qspi_GetStatus() == QSPI_BUSY){
		++i;
		if(i >= QSPI_TIMEOUT){
			terminal("\n%sqspi_EraseSector -> qspi_GetStatus", _Error);
			return ERROR;
		}
	}
  
  if (Sector >= (uint32_t)(MX25R6435F_FLASH_SIZE/MX25R6435F_SECTOR_SIZE))
  {
		terminal("\n%sqspi_EraseSector -> Out of range address", _Error);
    return ERROR;
  }
  
  /* Initialize the erase command */
  qspiCmd.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  qspiCmd.Instruction       = SECTOR_ERASE_CMD;
  qspiCmd.AddressMode       = QSPI_ADDRESS_1_LINE;
  qspiCmd.AddressSize       = QSPI_ADDRESS_24_BITS;
  qspiCmd.Address           = (Sector * MX25R6435F_SECTOR_SIZE);
  qspiCmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  qspiCmd.DataMode          = QSPI_DATA_NONE;
  qspiCmd.DummyCycles       = 0;
  qspiCmd.DdrMode           = QSPI_DDR_MODE_DISABLE;
  qspiCmd.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  qspiCmd.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
  
  /* Enable write operations */
  if (qspi_WriteEnable() != SUCCESS)
  {
		terminal("\n%sqspi_EraseSector -> qspi_WriteEnable", _Error);
    return ERROR;
  }
  
  
  /* Send the command */
  if (qspi_Command(&qspiCmd) != SUCCESS)
  {
		terminal("\n%sqspi_EraseSector -> qspi_Command", _Error);
    return ERROR;
  }
  
  return SUCCESS;
}

/**
  * @brief  Writes an amount of data to the QSPI memory.
  * @param  pData     : Pointer to data to be written
  * @param  WriteAddr : Write start address
  * @param  Size      : Size of data to write    
  * @retval QSPI memory status
  */
uint8_t  qspi_WriteMemory(uint8_t* pData, uint32_t WriteAddr, uint32_t Size)
{
	QSPI_CommandTypeDef qspiCmd;
	uint32_t end_addr, current_size, current_addr;


  /* Calculation of the size between the write address and the end of the page */
  current_size = MX25R6435F_PAGE_SIZE - (WriteAddr % MX25R6435F_PAGE_SIZE);

  /* Check if the size of the data is less than the remaining place in the page */
  if (current_size > Size)
  {
    current_size = Size;
  }

  /* Initialize the address variables */
  current_addr = WriteAddr;
  end_addr = WriteAddr + Size;

  /* Initialize the program command */
  qspiCmd.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  qspiCmd.Instruction       = QUAD_PAGE_PROG_CMD;
  qspiCmd.AddressMode       = QSPI_ADDRESS_4_LINES;
  qspiCmd.AddressSize       = QSPI_ADDRESS_24_BITS;
  qspiCmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  qspiCmd.DataMode          = QSPI_DATA_4_LINES;
  qspiCmd.DummyCycles       = 0;
  qspiCmd.DdrMode           = QSPI_DDR_MODE_DISABLE;
  qspiCmd.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  qspiCmd.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
  
  /* Perform the write page by page */
  do
  {
    qspiCmd.Address = current_addr;
    qspiCmd.NbData  = current_size;

    /* Enable write operations */
    if (qspi_WriteEnable() != SUCCESS)
    {
      terminal("\nqspi_WriteMemory: WriteEnable");
      return ERROR;
    }
    
    /* Configure the command */
    if (qspi_Command(&qspiCmd) != SUCCESS)
    {
      terminal("\nqspi_WriteMemory: Command");
      return ERROR;
    }
    
    /* Transmission of the data */
    if (qspi_Transmit(pData) != SUCCESS)
    {
      terminal("\nqspi_WriteMemory: Transmit");
      return ERROR;
    }
    
    /* Configure automatic polling mode to wait for end of program */  
    if (qspi_AutoPollingMemReady() != SUCCESS)
    {
      terminal("\nqspi_WriteMemory: AutoPollingMemReady");
      return ERROR;
    }
    
    /* Update the address and size variables for next page programming */
    current_addr += current_size;
    pData += current_size;
    current_size = ((current_addr + MX25R6435F_PAGE_SIZE) > end_addr) ? (end_addr - current_addr) : MX25R6435F_PAGE_SIZE;
  } while (current_addr < end_addr);
  
  return SUCCESS;
}
/**
  * @brief  Reads an amount of data from the QSPI memory.
  * @param  pData    : Pointer to data to be read
  * @param  ReadAddr : Read start address
  * @param  Size     : Size of data to read    
  * @retval QSPI memory status
  */
uint8_t  qspi_ReadMemory(uint8_t* pData, uint32_t ReadAddr, uint32_t Size)
{
	  QSPI_CommandTypeDef qspiCmd;

  /* Initialize the read command */
  qspiCmd.InstructionMode    = QSPI_INSTRUCTION_1_LINE;
  qspiCmd.Instruction        = QUAD_INOUT_READ_CMD;
  qspiCmd.AddressMode        = QSPI_ADDRESS_4_LINES;
  qspiCmd.AddressSize        = QSPI_ADDRESS_24_BITS;
  qspiCmd.Address            = ReadAddr;
  qspiCmd.AlternateByteMode  = QSPI_ALTERNATE_BYTES_4_LINES;
  qspiCmd.AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS;
  qspiCmd.AlternateBytes     = MX25R6435F_ALT_BYTES_NO_PE_MODE;
  qspiCmd.DataMode           = QSPI_DATA_4_LINES;
  qspiCmd.DummyCycles        = MX25R6435F_DUMMY_CYCLES_READ_QUAD;
  qspiCmd.NbData             = Size;
  qspiCmd.DdrMode            = QSPI_DDR_MODE_DISABLE;
  qspiCmd.DdrHoldHalfCycle   = QSPI_DDR_HHC_ANALOG_DELAY;
  qspiCmd.SIOOMode           = QSPI_SIOO_INST_EVERY_CMD;
  
  /* Configure the command */
  if (qspi_Command(&qspiCmd) != SUCCESS)
  {
		terminal("\n%sqspi_ReadMemory -> qspi_Command", _Error);
    return ERROR;
  }
  
  /* Reception of the data */
  if (qspi_Receive(pData) != SUCCESS)
  {
		terminal("\n%sqspi_ReadMemory -> qspi_Receive", _Error);
    return ERROR;
  }

  return SUCCESS;
}
void  qspi_Erase_Chip(void)
{
	QSPI_CommandTypeDef sCommand;
	
	terminal("\nchip erase started");
	time_show();
	
	qspi_WriteEnable();

	/* Erasing Sequence --------------------------------- */
	sCommand.Instruction = CHIP_ERASE_CMD;
	sCommand.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	sCommand.AddressSize = QSPI_ADDRESS_24_BITS;
	sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
	sCommand.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
	sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
	sCommand.AddressMode = QSPI_ADDRESS_NONE;
	sCommand.Address = 0;
	sCommand.DataMode = QSPI_DATA_NONE;
	sCommand.DummyCycles = 0;


	qspi_Command(&sCommand);

	qspi_AutoPollingMemReady();
	time_show();
  terminal("\nChip Erase: OK!");
}



/*******************************************************************************
 * QUADSPI static Functions
 *******************************************************************************/
/**
  * @brief Initialize the QSPI mode according to the specified parameters
  *        in the QSPI_InitTypeDef and initialize the associated handle.
  * @param hqspi : QSPI handle
  * @retval HAL status
  */
static uint8_t qspi_Init(QSPI_InitTypeDef *Init)
{
  uint8_t status;

  /* Check the QSPI_CommandTypeDef allocation */
  if(Init == NULL)
  {
    return ERROR;
  }
	
	/* Init the low level hardware : GPIO, CLOCK */
	qspi_gpio_Init();

  /* Configure QSPI FIFO Threshold */
  MODIFY_REG(QUADSPI->CR, QUADSPI_CR_FTHRES,
             ((Init->FifoThreshold - 1U) << QUADSPI_CR_FTHRES_Pos));

  /* Wait till BUSY flag reset */
  status = qspi_WaitFlagStateUntilTimeout(QUADSPI->SR, QUADSPI_SR_BUSY, RESET, QSPI_TIMEOUT);

  if(status == SUCCESS)
  {
    /* Configure QSPI Clock Prescaler and Sample Shift */
    MODIFY_REG(QUADSPI->CR, (QUADSPI_CR_PRESCALER | QUADSPI_CR_SSHIFT),
               ((Init->ClockPrescaler << QUADSPI_CR_PRESCALER_Pos) |
                 Init->SampleShifting));


    /* Configure QSPI Flash Size, CS High Time and Clock Mode */
    MODIFY_REG(QUADSPI->DCR, (QUADSPI_DCR_FSIZE | QUADSPI_DCR_CSHT | QUADSPI_DCR_CKMODE),
               ((Init->FlashSize << QUADSPI_DCR_FSIZE_Pos) |
                 Init->ChipSelectHighTime | Init->ClockMode));

    /* Enable the QSPI peripheral */
    QUADSPI->CR |= QUADSPI_CR_EN;
  }

  /* Return function status */
  return status;
}


/**
  * @brief  This function reset the QSPI memory.
  * @param  hqspi : QSPI handle
  * @retval None
  */

static uint8_t qspi_ResetMemory (void)
{
	QSPI_CommandTypeDef qspiCmd;
	
	/* Initialize the reset enable command */
	qspiCmd.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  qspiCmd.Instruction       = RESET_ENABLE_CMD;
  qspiCmd.AddressMode       = QSPI_ADDRESS_NONE;
	qspiCmd.Address						= 0;
  qspiCmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  qspiCmd.DataMode          = QSPI_DATA_NONE;
  qspiCmd.DummyCycles       = 0;
  qspiCmd.DdrMode           = QSPI_DDR_MODE_DISABLE;
  qspiCmd.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  qspiCmd.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;


	/* Send the command */
	if (qspi_Command(&qspiCmd) != SUCCESS){
		terminal("\n%sqspi_Resetchip -> RESET_ENABLE_CMD", _Error);
    return ERROR;
  }

	/* Send the reset memory command */
	qspiCmd.Instruction				= RESET_MEMORY_CMD;

	/* Send the command */
	if (qspi_Command(&qspiCmd) != SUCCESS){
		terminal("\n%sqspi_Resetchip -> RESET_MEMORY_CMD", _Error);
    return ERROR;
  }
	return SUCCESS;
}
/**
  * @brief  This function enables/disables the Quad mode of the memory.
  * @param  hqspi     : QSPI handle
  * @param  Operation : QSPI_QUAD_ENABLE or QSPI_QUAD_DISABLE mode  
  * @retval None
  */
static uint8_t qspi_QuadMode(uint8_t Operation)
{
  QSPI_CommandTypeDef qspiCmd;
  uint8_t reg;

  /* Read status register */
  qspiCmd.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  qspiCmd.Instruction       = READ_STATUS_REG_CMD;
  qspiCmd.AddressMode       = QSPI_ADDRESS_NONE;
  qspiCmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  qspiCmd.DataMode          = QSPI_DATA_1_LINE;
  qspiCmd.DummyCycles       = 0;
  qspiCmd.NbData            = 1;
  qspiCmd.DdrMode           = QSPI_DDR_MODE_DISABLE;
  qspiCmd.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  qspiCmd.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  if (qspi_Command(&qspiCmd) != SUCCESS)
  {
    return ERROR;
  }

  if (qspi_Receive(&reg) != SUCCESS)
  {
    return ERROR;
  }
	
  /* Enable write operations */
  if (qspi_WriteEnable() != SUCCESS)
  {
    return ERROR;
  }
	
  /* Activate/deactivate the Quad mode */
  if (Operation == QSPI_QUAD_ENABLE)
  {
    SET_BIT(reg, MX25R6435F_SR_QE);
  }
  else
  {
    CLEAR_BIT(reg, MX25R6435F_SR_QE);
  }

  qspiCmd.Instruction = WRITE_STATUS_CFG_REG_CMD;

  if (qspi_Command(&qspiCmd) != SUCCESS)
  {
    return ERROR;
  }
  if (qspi_Transmit(&reg) != SUCCESS)
  {
    return ERROR;
  }

  /* Wait that memory is ready */  
  if (qspi_AutoPollingMemReady() != SUCCESS)
  {
    return ERROR;
  }
  
  /* Check the configuration has been correctly done */
  qspiCmd.Instruction = READ_STATUS_REG_CMD;

  if (qspi_Command(&qspiCmd) != SUCCESS)
  {
    return ERROR;
  }

  if (qspi_Receive(&reg) != SUCCESS)
  {
    return ERROR;
  }
  
  if ((((reg & MX25R6435F_SR_QE) == 0) && (Operation == QSPI_QUAD_ENABLE)) ||
      (((reg & MX25R6435F_SR_QE) != 0) && (Operation == QSPI_QUAD_DISABLE)))
  {
    return ERROR;
  }

  return SUCCESS;
}
/**
  * @brief  This function enables/disables the high performance mode of the memory.
  * @param  hqspi     : QSPI handle
  * @param  Operation : QSPI_HIGH_PERF_ENABLE or QSPI_HIGH_PERF_DISABLE high performance mode    
  * @retval None
  */
static uint8_t qspi_HighPerfMode(uint8_t Operation)
{
  QSPI_CommandTypeDef qspiCmd;
  uint8_t reg[3];

  /* Read status register */
  qspiCmd.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  qspiCmd.Instruction       = READ_STATUS_REG_CMD;
  qspiCmd.AddressMode       = QSPI_ADDRESS_NONE;
  qspiCmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  qspiCmd.DataMode          = QSPI_DATA_1_LINE;
  qspiCmd.DummyCycles       = 0;
  qspiCmd.NbData            = 1;
  qspiCmd.DdrMode           = QSPI_DDR_MODE_DISABLE;
  qspiCmd.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  qspiCmd.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  if (qspi_Command(&qspiCmd) != SUCCESS)
  {
    return ERROR;
  }

  if (qspi_Receive(&reg[0]) != SUCCESS)
  {
    return ERROR;
  }

  /* Read configuration registers */
  qspiCmd.Instruction = READ_CFG_REG_CMD;
  qspiCmd.NbData      = 2;

  if (qspi_Command(&qspiCmd) != SUCCESS)
  {
    return ERROR;
  }

  if (qspi_Receive(&reg[1]) != SUCCESS)
  {
    return ERROR;
  }

  /* Enable write operations */
  if (qspi_WriteEnable() != SUCCESS)
  {
    return ERROR;
  }
  
  /* Activate/deactivate the Quad mode */
  if (Operation == QSPI_HIGH_PERF_ENABLE)
  {
    SET_BIT(reg[2], MX25R6435F_CR2_LH_SWITCH);
  }
  else
  {
    CLEAR_BIT(reg[2], MX25R6435F_CR2_LH_SWITCH);
  }

  qspiCmd.Instruction = WRITE_STATUS_CFG_REG_CMD;
  qspiCmd.NbData      = 3;

  if (qspi_Command(&qspiCmd) != SUCCESS)
  {
    return ERROR;
  }

  if (qspi_Transmit(&reg[0]) != SUCCESS)
  {
    return ERROR;
  }

  /* Wait that memory is ready */  
  if (qspi_AutoPollingMemReady() != SUCCESS)
  {
    return ERROR;
  }
  
  /* Check the configuration has been correctly done */
  qspiCmd.Instruction = READ_CFG_REG_CMD;
  qspiCmd.NbData      = 2;

  if (qspi_Command(&qspiCmd) != SUCCESS)
  {
    return ERROR;
  }

  if (qspi_Receive(&reg[0]) != SUCCESS)
  {
    return ERROR;
  }
  
  if ((((reg[1] & MX25R6435F_CR2_LH_SWITCH) == 0) && (Operation == QSPI_HIGH_PERF_ENABLE)) ||
      (((reg[1] & MX25R6435F_CR2_LH_SWITCH) != 0) && (Operation == QSPI_HIGH_PERF_DISABLE)))
  {
    return ERROR;
  }

  return SUCCESS;
}

/*******************************************************************************
 * 
 *******************************************************************************/
static void qspi_gpio_Init (void)
{
  LL_GPIO_InitTypeDef GPIO_InitDef;
  
  /*    (SCL = PB10, SDA = PB11) 
  PE10 - QUADSPI_CLK
  PE11 - QUADSPI_NCS
  PE12 - QUADSPI_BK1_IO0
  PE13 - QUADSPI_BK1_IO1
  PE14 - QUADSPI_BK1_IO2
  PE15 - QUADSPI_BK1_IO3
  */
  /* Enable the peripheral clock of GPIOE */
  GPIOE_CLK_ENABLE();
  
  /* Configure Pins as :  Alternate function (AF10),
                          High Speed,
                          Push-Pull,
                          No pull-up or pull-down
  */
  LL_GPIO_StructInit(&GPIO_InitDef);    // de-init the gpio struct
  
  GPIO_InitDef.Pin = LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13 | LL_GPIO_PIN_14 | LL_GPIO_PIN_15;
  GPIO_InitDef.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitDef.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitDef.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitDef.Pull = LL_GPIO_PULL_NO;
  GPIO_InitDef.Alternate = LL_GPIO_AF_10;
  LL_GPIO_Init(GPIOE, &GPIO_InitDef);
	
	/* Enable the Quad-SPI interface clock */
  RCC->AHB3ENR |= RCC_AHB3ENR_QSPIEN;
  /* Reset QSPI peripheral */
  RCC->AHB3RSTR |= RCC_AHB3RSTR_QSPIRST;  // Reset
  RCC->AHB3RSTR &= ~RCC_AHB3RSTR_QSPIRST; // Release reset
}

static uint8_t qspi_AutoPolling (QSPI_CommandTypeDef *cmd, QSPI_AutoPollingTypeDef *cfg)
{
	uint8_t status = SUCCESS;
  /* Wait till BUSY flag reset */
	status = qspi_WaitFlagStateUntilTimeout(QUADSPI->SR, QUADSPI_SR_BUSY, RESET, QSPI_TIMEOUT);
	
	if(status == SUCCESS){
		/* Configure QSPI: PSMAR register with the status match value */
		WRITE_REG(QUADSPI->PSMAR, cfg->Match);

		/* Configure QSPI: PSMKR register with the status mask value */
		WRITE_REG(QUADSPI->PSMKR, cfg->Mask);

		/* Configure QSPI: PIR register with the interval value */
		WRITE_REG(QUADSPI->PIR, cfg->Interval);

		/* Configure QSPI: CR register with Match mode and Automatic stop enabled
		(otherwise there will be an infinite loop in blocking mode) */
		MODIFY_REG(QUADSPI->CR, (QUADSPI_CR_PMM | QUADSPI_CR_APMS), (cfg->MatchMode | QSPI_AUTOMATIC_STOP_ENABLE));

		/* Call the configuration function */
		cmd->NbData = cfg->StatusBytesSize;
		QSPI_Config(cmd, QSPI_FUNCTIONAL_MODE_AUTO_POLLING);
		
		/* wait for memory to be ready */
    uint16_t i = 0;
    while(qspi_GetStatus() == QSPI_BUSY){
      ++i;
      if(i >= QSPI_TIMEOUT){
        terminal("\n%sqspi_AutoPolling -> qspi_GetStatus", _Error);
        return ERROR;
      }
    }
		
		/* Wait until SM flag is set to go back in idle state */
		status = qspi_WaitFlagStateUntilTimeout(QUADSPI->SR, QUADSPI_SR_SMF, SET, QSPI_TIMEOUT);
		
		if(status == SUCCESS){
			/* Clears QSPI_FLAG_SM's flag status. */
			QUADSPI->FCR |= QUADSPI_FCR_CSMF;
		} else {
      terminal("\nqspi_AutoPolling: qspi_WaitFlagStateUntilTimeout");
    }
	}

  /* Return function status */
  return status;
}
static uint8_t qspi_AutoPollingMemReady (void)
{
  QSPI_CommandTypeDef qspiCmd;
	QSPI_AutoPollingTypeDef qspiCfg;

	/* Configure automatic polling mode to wait for memory ready ------ */
	qspiCmd.InstructionMode 	= QSPI_INSTRUCTION_1_LINE;
	qspiCmd.Instruction				= READ_STATUS_REG_CMD;
	qspiCmd.AddressMode 			= QSPI_ADDRESS_NONE;
	qspiCmd.AlternateByteMode	= QSPI_ALTERNATE_BYTES_NONE;
	qspiCmd.DataMode 					= QSPI_DATA_1_LINE;
	qspiCmd.DummyCycles 			= 0;
	qspiCmd.DdrMode 					= QSPI_DDR_MODE_DISABLE;
	qspiCmd.DdrHoldHalfCycle 	= QSPI_DDR_HHC_ANALOG_DELAY;
	qspiCmd.SIOOMode 					= QSPI_SIOO_INST_EVERY_CMD;

	qspiCfg.Match 					= MX25R6435F_SR_QE;
	qspiCfg.Mask 						= 0xFF;
	qspiCfg.MatchMode 			= QSPI_MATCH_MODE_AND;
	qspiCfg.StatusBytesSize = 1;
	qspiCfg.Interval 				= 0x10;
	qspiCfg.AutomaticStop 	= QSPI_AUTOMATIC_STOP_ENABLE;

  if(qspi_AutoPolling(&qspiCmd , &qspiCfg) != SUCCESS){
		return ERROR;
	}
	return SUCCESS;
}
static ErrorStatus qspi_WriteEnable (void)
{
	QSPI_CommandTypeDef qspiCmd;
	QSPI_AutoPollingTypeDef qspiCfg;

	/* Enable write operations ------------------------------------------ */
	qspiCmd.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  qspiCmd.Instruction       = WRITE_ENABLE_CMD;
  qspiCmd.AddressMode       = QSPI_ADDRESS_NONE;
  qspiCmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  qspiCmd.DataMode          = QSPI_DATA_NONE;
  qspiCmd.DummyCycles       = 0;
  qspiCmd.DdrMode           = QSPI_DDR_MODE_DISABLE;
  qspiCmd.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  qspiCmd.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
	
	if (qspi_Command(&qspiCmd) != SUCCESS)
  {
		terminal("\n%sqspi_WriteEnable -> WRITE_ENABLE_CMD", _Error);
    return ERROR;
  }
	//LL_mDelay(2000);
	
	/* Configure automatic polling mode to wait for write enabling ---- */
	qspiCfg.Match           = MX25R6435F_SR_WEL;
  qspiCfg.Mask            = MX25R6435F_SR_WEL;
  qspiCfg.MatchMode       = QSPI_MATCH_MODE_AND;
  qspiCfg.StatusBytesSize = 1;
  qspiCfg.Interval        = 0x10;
  qspiCfg.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

  qspiCmd.Instruction    = READ_STATUS_REG_CMD;
  qspiCmd.DataMode       = QSPI_DATA_1_LINE;
  
	if(qspi_AutoPolling(&qspiCmd , &qspiCfg) != SUCCESS){
		terminal("\n%sqspi_WriteEnable -> qspi_AutoPolling", _Error);
		return ERROR;
	}
	return SUCCESS;
}
/*
 Enable quad mode and set dummy cycles count
*/
static void qspi_Configuration (void)
{
	QSPI_CommandTypeDef qspiCmd;
	uint8_t test_buffer[4] = { 0 };
	
	qspi_Read_All_Reg(test_buffer);
	/*modify buffer to enable quad mode*/
	test_buffer[0] |= 0x40;

	/*set dummy cycles*/
	test_buffer[1] &= ~0xC0;

	/*enable hight proform*/
	test_buffer[2] |= 0x02;

	qspiCmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	qspiCmd.AddressSize = QSPI_ADDRESS_24_BITS;
	qspiCmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	qspiCmd.DdrMode = QSPI_DDR_MODE_DISABLE;
	qspiCmd.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
	qspiCmd.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
	qspiCmd.Instruction = WRITE_STATUS_CFG_REG_CMD;
	qspiCmd.AddressMode = QSPI_ADDRESS_NONE;
	qspiCmd.DataMode = QSPI_DATA_1_LINE;
	qspiCmd.DummyCycles = 0;
	qspiCmd.NbData = 3;

	qspi_Command(&qspiCmd);

	qspi_Transmit(test_buffer);
}
static void qspi_Read_All_Reg (uint8_t* test_buffer)
{
	QSPI_CommandTypeDef qspiCmd;
	/*read status register*/
	qspiCmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	qspiCmd.Instruction = READ_STATUS_REG_CMD;
	qspiCmd.AddressMode = QSPI_ADDRESS_NONE;
	qspiCmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	qspiCmd.DataMode = QSPI_DATA_1_LINE;
	qspiCmd.DummyCycles = 0;
	qspiCmd.DdrMode = QSPI_DDR_MODE_DISABLE;
	qspiCmd.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
	qspiCmd.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
	qspiCmd.NbData = 1;

	qspi_Command(&qspiCmd);
  qspi_Receive(test_buffer);
  
	/*read configuration register*/
	qspiCmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	qspiCmd.Instruction = READ_CFG_REG_CMD;
	qspiCmd.AddressMode = QSPI_ADDRESS_NONE;
	qspiCmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	qspiCmd.DataMode = QSPI_DATA_1_LINE;
	qspiCmd.DummyCycles = 0;
	qspiCmd.DdrMode = QSPI_DDR_MODE_DISABLE;
	qspiCmd.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
	qspiCmd.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
	qspiCmd.NbData = 2;

	qspi_Command(&qspiCmd);
	qspi_Receive(&(test_buffer[1]));
  
}
/**
* @brief  Abort the current transmission.
* @param  hqspi : QSPI handle
* @retval HAL status
*/
static uint8_t qspi_Abort (void)
{
	uint8_t status;
  if ((QUADSPI->CR & QUADSPI_CR_DMAEN) != 0U)
  {
    /* Disable the DMA transfer by clearing the DMAEN bit in the QSPI CR register */
    CLEAR_BIT(QUADSPI->CR, QUADSPI_CR_DMAEN);
  }

  /* Configure QSPI: CR register with Abort request */
  SET_BIT(QUADSPI->CR, QUADSPI_CR_ABORT);

  /* Wait until TC flag is set to go back in idle state */
	status = qspi_WaitFlagStateUntilTimeout(QUADSPI->SR, QUADSPI_SR_TCF, SET, QSPI_TIMEOUT);
	if(status == SUCCESS){
		QUADSPI->FCR |= QUADSPI_FCR_CTCF;
		
		/* Wait until BUSY flag is reset */
		status = qspi_WaitFlagStateUntilTimeout(QUADSPI->SR, QUADSPI_SR_BUSY, RESET, QSPI_TIMEOUT);
	}
	
	if(status == SUCCESS){
		/* Reset functional mode configuration to indirect write mode by default */
		CLEAR_BIT(QUADSPI->CCR, QUADSPI_CCR_FMODE);
	}
	
	return status;
}

/**
  * @brief  Configure the communication registers.
  * @param  cmd : structure that contains the command configuration information
  * @param  FunctionalMode : functional mode to configured
  *           This parameter can be one of the following values:
  *            @arg QSPI_FUNCTIONAL_MODE_INDIRECT_WRITE: Indirect write mode
  *            @arg QSPI_FUNCTIONAL_MODE_INDIRECT_READ: Indirect read mode
  *            @arg QSPI_FUNCTIONAL_MODE_AUTO_POLLING: Automatic polling mode
  *            @arg QSPI_FUNCTIONAL_MODE_MEMORY_MAPPED: Memory-mapped mode
  * @retval None
  */
static void QSPI_Config(QSPI_CommandTypeDef *cmd, uint32_t FunctionalMode)
{
	
  if ((cmd->DataMode != QSPI_DATA_NONE) && (FunctionalMode != QSPI_FUNCTIONAL_MODE_MEMORY_MAPPED))
  {
    /* Configure QSPI: DLR register with the number of data to read or write */
    WRITE_REG(QUADSPI->DLR, (cmd->NbData - 1U));
  }

  if (cmd->InstructionMode != QSPI_INSTRUCTION_NONE)
  {
    if (cmd->AlternateByteMode != QSPI_ALTERNATE_BYTES_NONE)
    {
      /* Configure QSPI: ABR register with alternate bytes value */
      WRITE_REG(QUADSPI->ABR, cmd->AlternateBytes);

      if (cmd->AddressMode != QSPI_ADDRESS_NONE)
      {
        /*---- Command with instruction, address and alternate bytes ----*/
        /* Configure QSPI: CCR register with all communications parameters */
        WRITE_REG(QUADSPI->CCR, (cmd->DdrMode | cmd->DdrHoldHalfCycle | cmd->SIOOMode |
                                 cmd->DataMode | (cmd->DummyCycles << QUADSPI_CCR_DCYC_Pos) |
                                 cmd->AlternateBytesSize | cmd->AlternateByteMode |
                                 cmd->AddressSize | cmd->AddressMode | cmd->InstructionMode |
                                 cmd->Instruction | FunctionalMode));

        if (FunctionalMode != QSPI_FUNCTIONAL_MODE_MEMORY_MAPPED)
        {
          /* Configure QSPI: AR register with address value */
          WRITE_REG(QUADSPI->AR, cmd->Address);
        }
      }
      else
      {
        /*---- Command with instruction and alternate bytes ----*/
        /* Configure QSPI: CCR register with all communications parameters */
        WRITE_REG(QUADSPI->CCR, (cmd->DdrMode | cmd->DdrHoldHalfCycle | cmd->SIOOMode |
                                         cmd->DataMode | (cmd->DummyCycles << QUADSPI_CCR_DCYC_Pos) |
                                         cmd->AlternateBytesSize | cmd->AlternateByteMode |
                                         cmd->AddressMode | cmd->InstructionMode |
                                         cmd->Instruction | FunctionalMode));
      }
    }
    else
    {
      if (cmd->AddressMode != QSPI_ADDRESS_NONE)
      {
        /*---- Command with instruction and address ----*/
        /* Configure QSPI: CCR register with all communications parameters */
        WRITE_REG(QUADSPI->CCR, (cmd->DdrMode | cmd->DdrHoldHalfCycle | cmd->SIOOMode |
                                         cmd->DataMode | (cmd->DummyCycles << QUADSPI_CCR_DCYC_Pos) |
                                         cmd->AlternateByteMode | cmd->AddressSize | cmd->AddressMode |
                                         cmd->InstructionMode | cmd->Instruction | FunctionalMode));

        if (FunctionalMode != QSPI_FUNCTIONAL_MODE_MEMORY_MAPPED)
        {
          /* Configure QSPI: AR register with address value */
          WRITE_REG(QUADSPI->AR, cmd->Address);
        }
      }
      else
      {
        /*---- Command with only instruction ----*/
        /* Configure QSPI: CCR register with all communications parameters */
        WRITE_REG(QUADSPI->CCR, (cmd->DdrMode | cmd->DdrHoldHalfCycle | cmd->SIOOMode |
                                         cmd->DataMode | (cmd->DummyCycles << QUADSPI_CCR_DCYC_Pos) |
                                         cmd->AlternateByteMode | cmd->AddressMode |
                                         cmd->InstructionMode | cmd->Instruction | FunctionalMode));
      }
    }
  }
  else
  {
    if (cmd->AlternateByteMode != QSPI_ALTERNATE_BYTES_NONE)
    {
      /* Configure QSPI: ABR register with alternate bytes value */
      WRITE_REG(QUADSPI->ABR, cmd->AlternateBytes);

      if (cmd->AddressMode != QSPI_ADDRESS_NONE)
      {
        /*---- Command with address and alternate bytes ----*/
        /* Configure QSPI: CCR register with all communications parameters */
        WRITE_REG(QUADSPI->CCR, (cmd->DdrMode | cmd->DdrHoldHalfCycle | cmd->SIOOMode |
                                 cmd->DataMode | (cmd->DummyCycles << QUADSPI_CCR_DCYC_Pos) |
                                 cmd->AlternateBytesSize | cmd->AlternateByteMode |
                                 cmd->AddressSize | cmd->AddressMode |
                                 cmd->InstructionMode | FunctionalMode));

        if (FunctionalMode != QSPI_FUNCTIONAL_MODE_MEMORY_MAPPED)
        {
          /* Configure QSPI: AR register with address value */
          WRITE_REG(QUADSPI->AR, cmd->Address);
        }
      }
      else
      {
        /*---- Command with only alternate bytes ----*/
        /* Configure QSPI: CCR register with all communications parameters */
        WRITE_REG(QUADSPI->CCR, (cmd->DdrMode | cmd->DdrHoldHalfCycle | cmd->SIOOMode |
                                 cmd->DataMode | (cmd->DummyCycles << QUADSPI_CCR_DCYC_Pos) |
                                 cmd->AlternateBytesSize | cmd->AlternateByteMode |
                                 cmd->AddressMode | cmd->InstructionMode | FunctionalMode));
      }
    }
    else
    {
      if (cmd->AddressMode != QSPI_ADDRESS_NONE)
      {
        /*---- Command with only address ----*/
        /* Configure QSPI: CCR register with all communications parameters */
        WRITE_REG(QUADSPI->CCR, (cmd->DdrMode | cmd->DdrHoldHalfCycle | cmd->SIOOMode |
                                 cmd->DataMode | (cmd->DummyCycles << QUADSPI_CCR_DCYC_Pos) |
                                 cmd->AlternateByteMode | cmd->AddressSize |
                                 cmd->AddressMode | cmd->InstructionMode | FunctionalMode));

        if (FunctionalMode != QSPI_FUNCTIONAL_MODE_MEMORY_MAPPED)
        {
          /* Configure QSPI: AR register with address value */
          WRITE_REG(QUADSPI->AR, cmd->Address);
        }
      }
      else
      {
        /*---- Command with only data phase ----*/
        if (cmd->DataMode != QSPI_DATA_NONE)
        {
          /* Configure QSPI: CCR register with all communications parameters */
          WRITE_REG(QUADSPI->CCR, (cmd->DdrMode | cmd->DdrHoldHalfCycle | cmd->SIOOMode |
                                           cmd->DataMode | (cmd->DummyCycles << QUADSPI_CCR_DCYC_Pos) |
                                           cmd->AlternateByteMode | cmd->AddressMode |
                                           cmd->InstructionMode | FunctionalMode));
        }
      }
    }
  }
}



/**
  * @brief  Wait for a flag state until timeout.
  * @param  Register : QSPI Register
  * @param  Flag : Flag checked
  * @param  State : Value of the flag expected
  * @param  Timeout : Duration of the timeout
  * @retval ErrorStatus
  */
static ErrorStatus qspi_WaitFlagStateUntilTimeout(__IO uint32_t Register, uint32_t Flag, FlagStatus State, uint32_t Timeout)
{
	uint32_t i = 0;
  /* Wait until flag is in expected state */
  while((_QSPI_GET_FLAG(Register, Flag)) != State)
  {
		++i;
    /* Check for the Timeout */
    if (i >= Timeout)
    {
			//terminal("\nqspi_WaitFlagStateUntilTimeout, i:%d, timeout: %d", i, Timeout);
			return ERROR;
    }
  }
	
	//terminal("\ni: %d - %d", i, (Register & Flag));
  
	return SUCCESS;
}



/**
  * @brief  Reads current status of the QSPI memory.
  * @retval QSPI memory status
  */
static uint8_t qspi_GetStatus(void)
{
  QSPI_CommandTypeDef qspiCmd;
  uint8_t reg;

  /* Initialize the read security register command */
  qspiCmd.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  qspiCmd.Instruction       = READ_SEC_REG_CMD;
  qspiCmd.AddressMode       = QSPI_ADDRESS_NONE;
  qspiCmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  qspiCmd.DataMode          = QSPI_DATA_1_LINE;
  qspiCmd.DummyCycles       = 0;
  qspiCmd.NbData            = 1;
  qspiCmd.DdrMode           = QSPI_DDR_MODE_DISABLE;
  qspiCmd.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  qspiCmd.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (qspi_Command(&qspiCmd) != SUCCESS)
  {
    return ERROR;
  }

  /* Reception of the data */
  if (qspi_Receive(&reg) != SUCCESS)
  {
    return ERROR;
  }
  
  /* Check the value of the register */
  if ((reg & (MX25R6435F_SECR_P_FAIL | MX25R6435F_SECR_E_FAIL)) != 0)
  {
    return ERROR;
  }
  else if ((reg & (MX25R6435F_SECR_PSB | MX25R6435F_SECR_ESB)) != 0)
  {
    return QSPI_SUSPENDED;
  }

  /* Initialize the read status register command */
  qspiCmd.Instruction       = READ_STATUS_REG_CMD;

  /* Configure the command */
  if (qspi_Command(&qspiCmd) != SUCCESS)
  {
    return ERROR;
  }

  /* Reception of the data */
  if (qspi_Receive(&reg) != SUCCESS)
  {
    return ERROR;
  }

  /* Check the value of the register */
  if ((reg & MX25R6435F_SR_WIP) != 0)
  {
    return QSPI_BUSY;
  }
  else
  {
    return SUCCESS;
  }
}
/******************* (C) COPYRIGHT 2021 Farzin_M.Benam *****END OF FILE****/
