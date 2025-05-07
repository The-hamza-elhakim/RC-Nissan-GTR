/*
 * i2c_lcd.c
 *
 * Created: 30-11-2023 19:15:57
 *  Author: arunr
 */ 
#define F_CPU 5000000
#include <avr/io.h>
#include <util/delay.h>
#include "i2c_lcd.h"
#include "I2C_Master.h"

#define SLAVE_ADDRESS_LCD 0x27 // change this according to your setup

void lcd_send_cmd (char cmd)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
	I2C_Write (SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4);
}

void lcd_send_data (char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=1
	data_t[1] = data_u|0x09;  //en=0, rs=1
	data_t[2] = data_l|0x0D;  //en=1, rs=1
	data_t[3] = data_l|0x09;  //en=0, rs=1
	I2C_Write (SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4);
}

void lcd_clear (void)
{
	lcd_send_cmd (0x01);
	_delay_ms(10);
}

void lcd_put_cur(int row, int col)
{
	switch (row)
	{
		case 0:
		col |= 0x80;
		break;
		case 1:
		col |= 0xC0;
		break;
	}

	lcd_send_cmd (col);
}


void lcd_init (void)
{
	I2C_Init();
	_delay_ms(100);
	// 4 bit initialisation
	_delay_ms(50);  // wait for >40ms
	lcd_send_cmd (0x30);
	_delay_ms(5);  // wait for >4.1ms
	lcd_send_cmd (0x30);
	_delay_ms(1);  // wait for >100us
	lcd_send_cmd (0x30);
	_delay_ms(10);
	lcd_send_cmd (0x20);  // 4bit mode
	_delay_ms(10);

	// dislay initialisation
	lcd_send_cmd (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	_delay_ms(1);
	lcd_send_cmd (0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
	_delay_ms(1);
	lcd_send_cmd (0x01);  // clear display
	_delay_ms(10);
	lcd_send_cmd (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	_delay_ms(1);
	lcd_send_cmd (0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}

void lcd_send_string (char *str)
{
	while (*str) lcd_send_data (*str++);
}
