#ifndef __ISO_I2C_GPIO_EE_H
#define	__ISO_I2C_GPIO_EE_H

#include "stm32f0xx.h"

/* 
 * AT24C02 2kb = 2048bit = 2048/8 B = 256 B
 * 32 pages of 8 bytes each
 *
 * Device Address
 * 1 0 1 0 A2 A1 A0 R/W
 * 1 0 1 0 0  0  0  0 = 0XA0
 * 1 0 1 0 0  0  0  1 = 0XA1 
 */

/* AT24C01/02每页有8个字节 
 * AT24C04/08A/16A每页有16个字节 
 */

#define EE_DEV_ADDR				0x7C			/* 24xx02的设备地址 */
#define EE_PAGE_SIZE			8			  	/* 24xx02的页面大小 */
#define EE_SIZE				 	256			  	/* 24xx02总容量 */

/*函数定义*/
uint8_t ee_CheckOk(void);
uint8_t ee_ReadBytes(uint8_t *_pReadBuf, uint16_t _usAddress, uint16_t _usSize);
uint8_t ee_WriteBytes(uint8_t *_pWriteBuf, uint16_t _usAddress, uint16_t _usSize);
void I2C_GPIO_EEPROM_24C0x_WriteAndRead(void);

#endif /* __I2C_EE_H */
