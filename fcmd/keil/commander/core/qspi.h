#ifndef _QSPI_H
#define _QSPI_H

#ifdef __cplusplus
 extern "C" {
#endif



#include "main.h"


/*******************************************************************************
 * defintions
 *******************************************************************************/
 /* QSPI Error codes */
#define QSPI_OK            ((uint8_t)0x00)
#define QSPI_ERROR         ((uint8_t)0x01)
#define QSPI_BUSY          ((uint8_t)0x02)
#define QSPI_NOT_SUPPORTED ((uint8_t)0x04)
#define QSPI_SUSPENDED     ((uint8_t)0x08)
/* Exported types ------------------------------------------------------------*/
/**
  * @brief  QSPI Init structure definition
  */
typedef struct
{
  uint32_t ClockPrescaler;     /* Specifies the prescaler factor for generating clock based on the AHB clock.
                                  This parameter can be a number between 0 and 255 */
  uint32_t FifoThreshold;      /* Specifies the threshold number of bytes in the FIFO (used only in indirect mode)
                                  This parameter can be a value between 1 and 16 */
  uint32_t SampleShifting;     /* Specifies the Sample Shift. The data is sampled 1/2 clock cycle delay later to
                                  take in account external signal delays. (It should be QSPI_SAMPLE_SHIFTING_NONE in DDR mode)
                                  This parameter can be a value of @ref QSPI_SampleShifting */
  uint32_t FlashSize;          /* Specifies the Flash Size. FlashSize+1 is effectively the number of address bits
                                  required to address the flash memory. The flash capacity can be up to 4GB
                                  (addressed using 32 bits) in indirect mode, but the addressable space in
                                  memory-mapped mode is limited to 256MB
                                  This parameter can be a number between 0 and 31 */
  uint32_t ChipSelectHighTime; /* Specifies the Chip Select High Time. ChipSelectHighTime+1 defines the minimum number
                                  of clock cycles which the chip select must remain high between commands.
                                  This parameter can be a value of @ref QSPI_ChipSelectHighTime */
  uint32_t ClockMode;          /* Specifies the Clock Mode. It indicates the level that clock takes between commands.
                                  This parameter can be a value of @ref QSPI_ClockMode */
#if defined(QUADSPI_CR_DFM)
  uint32_t FlashID;            /* Specifies the Flash which will be used,
                                  This parameter can be a value of @ref QSPI_Flash_Select */
  uint32_t DualFlash;          /* Specifies the Dual Flash Mode State
                                  This parameter can be a value of @ref QSPI_DualFlash_Mode */
#endif
}QSPI_InitTypeDef;

/**
  * @brief  QSPI Command structure definition
  */
typedef struct
{
  uint32_t Instruction;        /* Specifies the Instruction to be sent
                                  This parameter can be a value (8-bit) between 0x00 and 0xFF */
  uint32_t Address;            /* Specifies the Address to be sent (Size from 1 to 4 bytes according AddressSize)
                                  This parameter can be a value (32-bits) between 0x0 and 0xFFFFFFFF */
  uint32_t AlternateBytes;     /* Specifies the Alternate Bytes to be sent (Size from 1 to 4 bytes according AlternateBytesSize)
                                  This parameter can be a value (32-bits) between 0x0 and 0xFFFFFFFF */
  uint32_t AddressSize;        /* Specifies the Address Size
                                  This parameter can be a value of @ref QSPI_AddressSize */
  uint32_t AlternateBytesSize; /* Specifies the Alternate Bytes Size
                                  This parameter can be a value of @ref QSPI_AlternateBytesSize */
  uint32_t DummyCycles;        /* Specifies the Number of Dummy Cycles.
                                  This parameter can be a number between 0 and 31 */
  uint32_t InstructionMode;    /* Specifies the Instruction Mode
                                  This parameter can be a value of @ref QSPI_InstructionMode */
  uint32_t AddressMode;        /* Specifies the Address Mode
                                  This parameter can be a value of @ref QSPI_AddressMode */
  uint32_t AlternateByteMode;  /* Specifies the Alternate Bytes Mode
                                  This parameter can be a value of @ref QSPI_AlternateBytesMode */
  uint32_t DataMode;           /* Specifies the Data Mode (used for dummy cycles and data phases)
                                  This parameter can be a value of @ref QSPI_DataMode */
  uint32_t NbData;             /* Specifies the number of data to transfer. (This is the number of bytes)
                                  This parameter can be any value between 0 and 0xFFFFFFFF (0 means undefined length
                                  until end of memory)*/
  uint32_t DdrMode;            /* Specifies the double data rate mode for address, alternate byte and data phase
                                  This parameter can be a value of @ref QSPI_DdrMode */
  uint32_t DdrHoldHalfCycle;   /* Specifies if the DDR hold is enabled. When enabled it delays the data
                                  output by one half of system clock in DDR mode.
                                  Not available on all devices.
                                  This parameter can be a value of @ref QSPI_DdrHoldHalfCycle */
  uint32_t SIOOMode;           /* Specifies the send instruction only once mode
                                  This parameter can be a value of @ref QSPI_SIOOMode */
}QSPI_CommandTypeDef;

/**
  * @brief  QSPI Auto Polling mode configuration structure definition
  */
typedef struct
{
  uint32_t Match;              /* Specifies the value to be compared with the masked status register to get a match.
                                  This parameter can be any value between 0 and 0xFFFFFFFF */
  uint32_t Mask;               /* Specifies the mask to be applied to the status bytes received.
                                  This parameter can be any value between 0 and 0xFFFFFFFF */
  uint32_t Interval;           /* Specifies the number of clock cycles between two read during automatic polling phases.
                                  This parameter can be any value between 0 and 0xFFFF */
  uint32_t StatusBytesSize;    /* Specifies the size of the status bytes received.
                                  This parameter can be any value between 1 and 4 */
  uint32_t MatchMode;          /* Specifies the method used for determining a match.
                                  This parameter can be a value of @ref QSPI_MatchMode */
  uint32_t AutomaticStop;      /* Specifies if automatic polling is stopped after a match.
                                  This parameter can be a value of @ref QSPI_AutomaticStop */
}QSPI_AutoPollingTypeDef;

/* Exported constants --------------------------------------------------------*/
/** @defgroup QSPI_Exported_Constants QSPI Exported Constants
  * @{
  */

/** @defgroup QSPI_SampleShifting QSPI Sample Shifting
  * @{
  */
#define QSPI_SAMPLE_SHIFTING_NONE      0x00000000U                   /*!<No clock cycle shift to sample data*/
#define QSPI_SAMPLE_SHIFTING_HALFCYCLE ((uint32_t)QUADSPI_CR_SSHIFT) /*!<1/2 clock cycle shift to sample data*/
/**
  * @}
  */

/** @defgroup QSPI_ChipSelectHighTime QSPI ChipSelect High Time
  * @{
  */
#define QSPI_CS_HIGH_TIME_1_CYCLE      0x00000000U                                         /*!<nCS stay high for at least 1 clock cycle between commands*/
#define QSPI_CS_HIGH_TIME_2_CYCLE      ((uint32_t)QUADSPI_DCR_CSHT_0)                      /*!<nCS stay high for at least 2 clock cycles between commands*/
#define QSPI_CS_HIGH_TIME_3_CYCLE      ((uint32_t)QUADSPI_DCR_CSHT_1)                      /*!<nCS stay high for at least 3 clock cycles between commands*/
#define QSPI_CS_HIGH_TIME_4_CYCLE      ((uint32_t)QUADSPI_DCR_CSHT_0 | QUADSPI_DCR_CSHT_1) /*!<nCS stay high for at least 4 clock cycles between commands*/
#define QSPI_CS_HIGH_TIME_5_CYCLE      ((uint32_t)QUADSPI_DCR_CSHT_2)                      /*!<nCS stay high for at least 5 clock cycles between commands*/
#define QSPI_CS_HIGH_TIME_6_CYCLE      ((uint32_t)QUADSPI_DCR_CSHT_2 | QUADSPI_DCR_CSHT_0) /*!<nCS stay high for at least 6 clock cycles between commands*/
#define QSPI_CS_HIGH_TIME_7_CYCLE      ((uint32_t)QUADSPI_DCR_CSHT_2 | QUADSPI_DCR_CSHT_1) /*!<nCS stay high for at least 7 clock cycles between commands*/
#define QSPI_CS_HIGH_TIME_8_CYCLE      ((uint32_t)QUADSPI_DCR_CSHT)                        /*!<nCS stay high for at least 8 clock cycles between commands*/
/**
  * @}
  */

/** @defgroup QSPI_ClockMode QSPI Clock Mode
  * @{
  */
#define QSPI_CLOCK_MODE_0              0x00000000U                    /*!<Clk stays low while nCS is released*/
#define QSPI_CLOCK_MODE_3              ((uint32_t)QUADSPI_DCR_CKMODE) /*!<Clk goes high while nCS is released*/

/** @defgroup QSPI_AddressSize QSPI Address Size
  * @{
  */
#define QSPI_ADDRESS_8_BITS            0x00000000U                      /*!<8-bit address*/
#define QSPI_ADDRESS_16_BITS           ((uint32_t)QUADSPI_CCR_ADSIZE_0) /*!<16-bit address*/
#define QSPI_ADDRESS_24_BITS           ((uint32_t)QUADSPI_CCR_ADSIZE_1) /*!<24-bit address*/
#define QSPI_ADDRESS_32_BITS           ((uint32_t)QUADSPI_CCR_ADSIZE)   /*!<32-bit address*/
/**
  * @}
  */

/** @defgroup QSPI_AlternateBytesSize QSPI Alternate Bytes Size
  * @{
  */
#define QSPI_ALTERNATE_BYTES_8_BITS    0x00000000U                      /*!<8-bit alternate bytes*/
#define QSPI_ALTERNATE_BYTES_16_BITS   ((uint32_t)QUADSPI_CCR_ABSIZE_0) /*!<16-bit alternate bytes*/
#define QSPI_ALTERNATE_BYTES_24_BITS   ((uint32_t)QUADSPI_CCR_ABSIZE_1) /*!<24-bit alternate bytes*/
#define QSPI_ALTERNATE_BYTES_32_BITS   ((uint32_t)QUADSPI_CCR_ABSIZE)   /*!<32-bit alternate bytes*/
/**
  * @}
  */

/** @defgroup QSPI_InstructionMode QSPI Instruction Mode
* @{
*/
#define QSPI_INSTRUCTION_NONE          0x00000000U                     /*!<No instruction*/
#define QSPI_INSTRUCTION_1_LINE        ((uint32_t)QUADSPI_CCR_IMODE_0) /*!<Instruction on a single line*/
#define QSPI_INSTRUCTION_2_LINES       ((uint32_t)QUADSPI_CCR_IMODE_1) /*!<Instruction on two lines*/
#define QSPI_INSTRUCTION_4_LINES       ((uint32_t)QUADSPI_CCR_IMODE)   /*!<Instruction on four lines*/
/**
  * @}
  */

/** @defgroup QSPI_AddressMode QSPI Address Mode
* @{
*/
#define QSPI_ADDRESS_NONE              0x00000000U                      /*!<No address*/
#define QSPI_ADDRESS_1_LINE            ((uint32_t)QUADSPI_CCR_ADMODE_0) /*!<Address on a single line*/
#define QSPI_ADDRESS_2_LINES           ((uint32_t)QUADSPI_CCR_ADMODE_1) /*!<Address on two lines*/
#define QSPI_ADDRESS_4_LINES           ((uint32_t)QUADSPI_CCR_ADMODE)   /*!<Address on four lines*/
/**
  * @}
  */

/** @defgroup QSPI_AlternateBytesMode QSPI Alternate Bytes Mode
* @{
*/
#define QSPI_ALTERNATE_BYTES_NONE      0x00000000U                      /*!<No alternate bytes*/
#define QSPI_ALTERNATE_BYTES_1_LINE    ((uint32_t)QUADSPI_CCR_ABMODE_0) /*!<Alternate bytes on a single line*/
#define QSPI_ALTERNATE_BYTES_2_LINES   ((uint32_t)QUADSPI_CCR_ABMODE_1) /*!<Alternate bytes on two lines*/
#define QSPI_ALTERNATE_BYTES_4_LINES   ((uint32_t)QUADSPI_CCR_ABMODE)   /*!<Alternate bytes on four lines*/
/**
  * @}
  */

/** @defgroup QSPI_DataMode QSPI Data Mode
  * @{
  */
#define QSPI_DATA_NONE                 0x00000000U                     /*!<No data*/
#define QSPI_DATA_1_LINE               ((uint32_t)QUADSPI_CCR_DMODE_0) /*!<Data on a single line*/
#define QSPI_DATA_2_LINES              ((uint32_t)QUADSPI_CCR_DMODE_1) /*!<Data on two lines*/
#define QSPI_DATA_4_LINES              ((uint32_t)QUADSPI_CCR_DMODE)   /*!<Data on four lines*/
/**
  * @}
  */

/** @defgroup QSPI_DdrMode QSPI DDR Mode
  * @{
  */
#define QSPI_DDR_MODE_DISABLE          0x00000000U                  /*!<Double data rate mode disabled*/
#define QSPI_DDR_MODE_ENABLE           ((uint32_t)QUADSPI_CCR_DDRM) /*!<Double data rate mode enabled*/
/**
  * @}
  */

/** @defgroup QSPI_DdrHoldHalfCycle QSPI DDR Data Output Delay
  * @{
  */
#define QSPI_DDR_HHC_ANALOG_DELAY      0x00000000U                  /*!<Delay the data output using analog delay in DDR mode*/
#if defined(QUADSPI_CCR_DHHC)
#define QSPI_DDR_HHC_HALF_CLK_DELAY    ((uint32_t)QUADSPI_CCR_DHHC) /*!<Delay the data output by one half of system clock in DDR mode*/
#endif
/**
  * @}
  */

/** @defgroup QSPI_SIOOMode QSPI Send Instruction Mode
  * @{
  */
#define QSPI_SIOO_INST_EVERY_CMD       0x00000000U                  /*!<Send instruction on every transaction*/
#define QSPI_SIOO_INST_ONLY_FIRST_CMD  ((uint32_t)QUADSPI_CCR_SIOO) /*!<Send instruction only for the first command*/
/**
  * @}
  */

/** @defgroup QSPI_MatchMode QSPI Match Mode
  * @{
  */
#define QSPI_MATCH_MODE_AND            0x00000000U                /*!<AND match mode between unmasked bits*/
#define QSPI_MATCH_MODE_OR             ((uint32_t)QUADSPI_CR_PMM) /*!<OR match mode between unmasked bits*/
/**
  * @}
  */

/** @defgroup QSPI_AutomaticStop QSPI Automatic Stop
  * @{
  */
#define QSPI_AUTOMATIC_STOP_DISABLE    0x00000000U                 /*!<AutoPolling stops only with abort or QSPI disabling*/
#define QSPI_AUTOMATIC_STOP_ENABLE     ((uint32_t)QUADSPI_CR_APMS) /*!<AutoPolling stops as soon as there is a match*/
/**
  * @}
  */

/** @defgroup QSPI_TimeOutActivation QSPI Timeout Activation
  * @{
  */
#define QSPI_TIMEOUT_COUNTER_DISABLE   0x00000000U                 /*!<Timeout counter disabled, nCS remains active*/
#define QSPI_TIMEOUT_COUNTER_ENABLE    ((uint32_t)QUADSPI_CR_TCEN) /*!<Timeout counter enabled, nCS released when timeout expires*/
/**
  * @}
  */



/** @defgroup QSPI_Interrupts QSPI Interrupts
  * @{
  */
#define QSPI_IT_TO                     QUADSPI_CR_TOIE /*!<Interrupt on the timeout flag*/
#define QSPI_IT_SM                     QUADSPI_CR_SMIE /*!<Interrupt on the status match flag*/
#define QSPI_IT_FT                     QUADSPI_CR_FTIE /*!<Interrupt on the fifo threshold flag*/
#define QSPI_IT_TC                     QUADSPI_CR_TCIE /*!<Interrupt on the transfer complete flag*/
#define QSPI_IT_TE                     QUADSPI_CR_TEIE /*!<Interrupt on the transfer error flag*/

/* Exported macros -----------------------------------------------------------*/
/** @defgroup QSPI_Exported_Macros QSPI Exported Macros
  * @{
  */
/**
  * @brief  Check whether the selected QSPI flag is set or not.
  * @param  __HANDLE__ : specifies the QSPI Register.
  * @param  __FLAG__ : specifies the QSPI flag to check.
  *          This parameter can be one of the following values:
  *            @arg QSPI_FLAG_BUSY: QSPI Busy flag
  *            @arg QSPI_FLAG_TO:   QSPI Timeout flag
  *            @arg QSPI_FLAG_SM:   QSPI Status match flag
  *            @arg QSPI_FLAG_FT:   QSPI FIFO threshold flag
  *            @arg QSPI_FLAG_TC:   QSPI Transfer complete flag
  *            @arg QSPI_FLAG_TE:   QSPI Transfer error flag
  * @retval None
  */
#define _QSPI_GET_FLAG(__HANDLE__, __FLAG__)           ((READ_BIT((__HANDLE__), (__FLAG__)) != 0U) ? SET : RESET)

/* Exported functions --------------------------------------------------------*/ 
uint8_t  qspi_config (void);


uint8_t  qspi_WriteMemory(uint8_t* pData, uint32_t WriteAddr, uint32_t Size);
uint8_t  qspi_ReadMemory(uint8_t* pData, uint32_t ReadAddr, uint32_t Size);
void  qspi_Erase_Chip(void);
uint8_t  qspi_EraseSector(uint32_t Sector);

#endif

