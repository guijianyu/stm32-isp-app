/**
  ******************************************************************************
  * @file    STM32F0xx_IAP/inc/flash_if.h 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    29-May-2012
  * @brief   Header for main.c module
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FLASH_IF_H
#define __FLASH_IF_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define USER_FLASH_LAST_PAGE_ADDRESS  0x08003C00
#define USER_FLASH_END_ADDRESS        0x08003FFF /* 4 KBytes*/
#define FLASH_PAGE_SIZE               0x400      /* 1 Kbytes */

/* define the address from where env will be loaded,
   the application address should be a start sector address */
#define APPLICATION_ADDRESS     (uint32_t)0x08003000

/* Get the number of Sector from where the env will be loaded */
#define  FLASH_PAGE_NUMBER      (uint32_t)((APPLICATION_ADDRESS - 0x08000000) >> 12)

/* Compute the mask to test if the Flash memory, where the env will be
   loaded, is write protected */
#define  FLASH_PROTECTED_PAGES   ((uint32_t)~((1 << FLASH_PAGE_NUMBER) - 1))

/* define the env size */
#define USER_FLASH_SIZE   (USER_FLASH_END_ADDRESS - APPLICATION_ADDRESS + 1)

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void FLASH_If_Init(void);
uint32_t FLASH_If_Erase(uint32_t StartSector);
uint32_t FLASH_If_Write(__IO uint32_t* FlashAddress, uint32_t* Data, uint16_t DataLength);
uint32_t FLASH_If_DisableWriteProtection(uint32_t pages);
uint32_t FLASH_If_GetWriteProtectionStatus(uint32_t pages);
void FLASH_If_GetEnv(__IO uint32_t FlashAddress, uint8_t* data, uint8_t length);

#endif  /* __FLASH_IF_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

