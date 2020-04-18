/*
 * lcd_display_api.c
 *
 *  Created on: Apr 10, 2020
 *      Author: Pedro Alonso Sanchez Cruz
 */

#include "stm32f1xx_hal.h"
#include "lcd_display_api.h"

void LCDSendCmd(LCDDisplay_S *lcd_disp_, char cmd)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0

	HAL_I2C_Master_Transmit(lcd_disp_->i2c_handler, lcd_disp_->i2c_address, (uint8_t *) data_t, lcd_disp_->byte_mode, lcd_disp_->i2c_timeout);
}

void LCDSendData(LCDDisplay_S *lcd_disp_, char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=0
	data_t[1] = data_u|0x09;  //en=0, rs=0
	data_t[2] = data_l|0x0D;  //en=1, rs=0
	data_t[3] = data_l|0x09;  //en=0, rs=0

	HAL_I2C_Master_Transmit(lcd_disp_->i2c_handler, lcd_disp_->i2c_address, (uint8_t *) data_t, lcd_disp_->byte_mode, lcd_disp_->i2c_timeout);
}

void LCDClear(LCDDisplay_S *lcd_disp_)
{
	LCDSendCmd(lcd_disp_, 0x80);
	for (int i = 0; i < 70; i++)
	{
		LCDSendData(lcd_disp_, ' ');
	}
}

void LCDSetCursorPos(LCDDisplay_S *lcd_disp_, uint8_t row, uint8_t col)
{
	uint16_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
	LCDSendCmd(lcd_disp_, (0x80 | (row_offsets[row] + col)) );
}

void LCDInit(LCDDisplay_S *lcd_disp_, I2C_HandleTypeDef *i2c_handler_)
{
	lcd_disp_->i2c_handler = i2c_handler_;
	lcd_disp_->byte_mode = LCD_DISP_4_BYTE_MODE;
	lcd_disp_->i2c_address = LCD_I2C_ADDRESS;
	lcd_disp_->i2c_timeout = LCD_DISP_I2C_TIMEOUT_MS;

	HAL_Delay(50);
	LCDSendCmd(lcd_disp_, 0x30);
	HAL_Delay(5);
	LCDSendCmd(lcd_disp_, 0x30);
	HAL_Delay(1);
	LCDSendCmd(lcd_disp_, 0x30);
	HAL_Delay(10);
	LCDSendCmd(lcd_disp_, 0x20);
	HAL_Delay(10);
	LCDSendCmd(lcd_disp_, 0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	HAL_Delay(1);
	LCDSendCmd(lcd_disp_, 0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
	HAL_Delay(1);
	LCDSendCmd(lcd_disp_, 0x01); // clear display
	HAL_Delay(1);
	LCDSendCmd(lcd_disp_, 0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	HAL_Delay(1);
	LCDSendCmd(lcd_disp_, 0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
	HAL_Delay(100);
	LCDClear(lcd_disp_);
}

void LCDSendString(LCDDisplay_S *lcd_disp_, char *str)
{
	while (*str)
	{
		LCDSendData(lcd_disp_, *str++);
	}
}
