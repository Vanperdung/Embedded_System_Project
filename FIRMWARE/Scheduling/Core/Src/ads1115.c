/*
 * ads1115.c
 *
 *  Created on: Mar 2, 2023
 *      Author: dung
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <time.h>

#include "ads1115.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c2;

void ads_write(uint8_t reg_addr, uint8_t *data_send, int len)
{
	uint8_t *data = (uint8_t *)calloc(len + 2, sizeof(uint8_t));
	data[0] = reg_addr;
	memcpy((char *)&data[1], (char *)data_send, len);
	HAL_I2C_Master_Transmit(&hi2c2, SLAVE_ADDRESS_ADS1115, data, strlen((char *)data), 1000);
	free(data);
}

void ads_read(uint8_t reg_addr, uint8_t *data_read)
{
	HAL_I2C_Master_Transmit(&hi2c2, SLAVE_ADDRESS_ADS1115, &reg_addr, 1, 1000);
	HAL_I2C_Master_Receive(&hi2c2, SLAVE_ADDRESS_ADS1115, data_read, 2, 1000);
}

void ads_init(void)
{
	uint8_t data_init[3] = {0x46, 0x20, 0x00};
	ads_write(ADS1115_CONFIG_REG, data_init, 2);
}

void ads_start_conv(void)
{
	uint8_t data_read[3];
	ads_read(ADS1115_CONFIG_REG, data_read);
	if(!(data_read[0] & 0x80))
	{
		data_read[0] |= 0x80;
		ads_write(ADS1115_CONFIG_REG, data_read, 2);
	}
	HAL_Delay(1);
}
