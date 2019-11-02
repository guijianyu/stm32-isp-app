/**
  ******************************************************************************
  * @file    STM32F0xx_IAP/src/menu.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    29-May-2012
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "common.h"
#include "flash_if.h"
#include "menu.h"
#include "ymodem.h"

/** @addtogroup STM32F0xx_IAP
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
pFunction Jump_To_Application;
uint32_t JumpAddress;
extern uint32_t UserMemoryMask;
uint32_t FlashProtection = 0;
uint8_t tab_1024[1024] = {0};
uint8_t FileName[FILE_NAME_LENGTH];

/* Private function prototypes -----------------------------------------------*/
int32_t SerialDownload(void);
void SerialUpload(void);
void DisableWriteProtect(uint32_t pages);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Download a file via serial port
  * @param  None
  * @retval None
  */
int32_t SerialDownload(void)
{
  uint8_t Number[10] = {0};
  int32_t Size = 0;

  SerialPutString("Waiting for the file to be sent ... (press 'a' to abort)\n\r");
  Size = Ymodem_Receive(&tab_1024[0]);
  if (Size > 0)
  {
    SerialPutString("\n\n\r Programming Completed Successfully!\n\r--------------------------------\r\n Name: ");
    SerialPutString(FileName);
    Int2Str(Number, Size);
    SerialPutString("\n\r Size: ");
    SerialPutString(Number);
    SerialPutString(" Bytes\r\n");
    SerialPutString("-------------------\n");
  }
  else if (Size == -1)
  {
    SerialPutString("\n\n\rThe image size is higher than the allowed space memory!\n\r");
  }
  else if (Size == -2)
  {
    SerialPutString("\n\n\rVerification failed!\n\r");
  }
  else if (Size == -3)
  {
    SerialPutString("\r\n\nAborted by user.\n\r");
  }
  else
  {
    SerialPutString("\n\rFailed to receive the file!\n\r");
  }
  return Size;
}

/**
  * @brief  Upload a file via serial port.
  * @param  None
  * @retval None
  */
void SerialUpload(void)
{
  uint8_t status = 0 ; 

  SerialPutString("\n\n\rSelect Receive File\n\r");

  if (GetKey() == CRC16)
  {
    /* Transmit the flash image through ymodem protocol */
    status = Ymodem_Transmit((uint8_t*)APPLICATION_ADDRESS, (const uint8_t*)"UploadedFlashImage.bin", USER_FLASH_SIZE);

    if (status != 0) 
    {
      SerialPutString("\n\rError Occurred while Transmitting File\n\r");
    }
    else
    {
      SerialPutString("\n\rFile uploaded successfully \n\r");
    }
  }
}

/**
  * @brief  解除flash页的写保护
  * @param  None
  * @retval None
  */
void DisableWriteProtect(uint32_t pages)
{
	uint8_t writeprotect = 0;
	
	/* Disable the write protection */
	writeprotect = FLASH_If_DisableWriteProtection(pages);

	switch (writeprotect)
	{
		case 0:
		{
			SerialPutString("Write Protection disabled...\r\n");
			SerialPutString("...and a System Reset will be generated to reload the new option bytes\r\n");

			/* Launch loading new option bytes */
			FLASH_OB_Launch();
			break;
		}
		case 1:
		{
			SerialPutString("Error: Flash write unprotection failed...\r\n");
			break;
		}
		case 2:
		{
			SerialPutString("Flash memory not write protected\r\n");
			break;
		}
		default:
		{

		}
	}
}


/**
  * @brief  Display the Main Menu on HyperTerminal
  * @param  None
  * @retval None
  */
void Main_Menu(void)
{
  uint8_t key = 0;
  uint8_t flag[4] = {0};
  uint8_t i = 0;

  //SerialPutString("\r\n======================================================================");
  //SerialPutString("\r\n=              (C) COPYRIGHT 2012 STMicroelectronics                 =");
  //SerialPutString("\r\n=                                                                    =");
  //SerialPutString("\r\n=  STM32F0xx In-Application Programming Application  (Version 1.0.0) =");
  //SerialPutString("\r\n=                                                                    =");
  //SerialPutString("\r\n=                                   By MCD Application Team          =");
  //SerialPutString("\r\n======================================================================");
  //SerialPutString("\r\n\r\n");

  /* Test if any sector of Flash memory where user application will be loaded is write protected */
  if (FLASH_If_GetWriteProtectionStatus(FLASH_PROTECTED_PAGES) != 0){
	FlashProtection = 1;
	DisableWriteProtect(FLASH_PROTECTED_PAGES);
  }else{
	FlashProtection = 0;
  }

  if (FLASH_If_GetWriteProtectionStatus(ENV_FLASH_PROTECTED_PAGES) != 0){
	DisableWriteProtect(ENV_FLASH_PROTECTED_PAGES);
	SerialPutString("  Disable the env write protection \r\n\n");
  }

  while (1)
  {
    SerialPutString("\r\n================== Main Menu ============================\r\n\n");
    SerialPutString("  Download Image To the STM32F0xx Internal Flash ------- 1\r\n\n");
    SerialPutString("  Upload Image From the STM32F0xx Internal Flash ------- 2\r\n\n");
    SerialPutString("  Execute The New Program ------------------------------ 3\r\n\n");

    if(FlashProtection != 0)
      SerialPutString("  Disable the app write protection ------------------------- 4\r\n\n");

    //SerialPutString("==========================================================\r\n\n");

	/*0x08003000处的标志位*/
	//for(i = 0; i < 4; i++){
	//	flag[i] = *(uint8_t*)(ENV_ADDRESS + i);
	//	SerialPutChar(flag[i]);
	//}
	
    /* Receive key */
	SerialPutString("  Input key:");
    //key = GetKey();
    key = *(uint8_t*)(ENV_ADDRESS);
	SerialPutChar(key);
	SerialPutString("\r\n");

	switch(key){
	case 0x31:
	case 0xFF:
	{
		uint32_t destination = ENV_ADDRESS;
		uint32_t env= 0x33;
		
		/* Download user application in the Flash */
		if(SerialDownload() > 0){
			FLASH_If_Erase(ENV_ADDRESS, ENV_FLASH_END_ADDRESS);
			FLASH_If_Write(&destination, &env, 1);
		}
		break;
	}
	
	case 0x32:
		/* Upload user application from the Flash */
		SerialUpload();
		break;

	case 0x33:
		__disable_irq();
		
		/* Jump to user application */
		JumpAddress = *(__IO uint32_t*) (APPLICATION_ADDRESS + 4);
		Jump_To_Application = (pFunction) JumpAddress;
		
		/* Initialize user application's Stack Pointer */
		__set_MSP(*(__IO uint32_t*) APPLICATION_ADDRESS);
		
		/* Jump to application */
		Jump_To_Application();
		break;
	
	case 0x34:
		if(FlashProtection == 1){
			DisableWriteProtect(FLASH_PROTECTED_PAGES);
	    }else{
			if (FlashProtection == 0)
				SerialPutString("Invalid Number ! ==> The number should be either 1, 2 or 3\r");
			else
				SerialPutString("Invalid Number ! ==> The number should be either 1, 2, 3 or 4\r");
	    }
		break;
		
	default:
		break;
	}
  }
}


/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
