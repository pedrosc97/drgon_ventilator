/*
 * lcd_display_api.h
 *
 *  Created on: Apr 10, 2020
 *      Author: Pedro Alonso Sanchez Cruz
 */

#ifndef INC_LCD_DISPLAY_API_H_
#define INC_LCD_DISPLAY_API_H_

#define LCD_I2C_ADDRESS 			0x4E
#define LCD_DISP_4_BYTE_MODE 		4
#define LCD_DISP_I2C_TIMEOUT_MS		100

typedef struct LCDDisplay_S
{
	I2C_HandleTypeDef 	*i2c_handler;
	uint8_t 			i2c_address;
	uint16_t 			i2c_timeout;
	uint8_t				byte_mode;
} LCDDisplay_S;

void LCDSendCmd(LCDDisplay_S *lcd_disp_, char cmd_);
void LCDSendData(LCDDisplay_S *lcd_disp_, char data);

void LCDInit(LCDDisplay_S *lcd_disp_, I2C_HandleTypeDef *i2c_handler_);
void LCDClear(LCDDisplay_S *lcd_disp_);
void LCDSetCursorPos(LCDDisplay_S *lcd_disp_, uint8_t row, uint8_t col);
void LCDSendString(LCDDisplay_S *lcd_disp_, char *str);

#endif /* INC_LCD_DISPLAY_API_H_ */
