/*
 * ads1115.h
 *
 *  Created on: Mar 2, 2023
 *      Author: dung
 */

#ifndef INC_ADS1115_H_
#define INC_ADS1115_H_

#include "stm32f1xx_hal.h"

#define SLAVE_ADDRESS_ADS1115       (0x48 << 1)
#define ADS1115_CONVERSION_REG      0x00
#define ADS1115_CONFIG_REG          0x01
#define ADS1115_LO_THRESH_REG       0x02
#define ADS1115_HI_THRESH_REG       0x03

void ads_write(uint8_t reg_addr, uint8_t *data_send, int len);
void ads_read(uint8_t reg_addr, uint8_t *data_read);
void ads_init(void);
void ads_start_conv(void);

#endif /* INC_ADS1115_H_ */
