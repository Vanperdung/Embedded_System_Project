/*
 * lcd.h
 *
 *  Created on: Mar 2, 2023
 *      Author: dung
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_

#include "stm32f1xx_hal.h"

#define SLAVE_ADDRESS_LCD (0x27 << 1)

void lcd_init(void);
void lcd_send_cmd(char cmd);
void lcd_send_data(char data);
void lcd_send_string(char *str);
void lcd_clear(void);

#endif /* INC_LCD_H_ */
