/*
 * i2c_lcd.h
 *
 * Created: 30-11-2023 19:16:58
 *  Author: arunr
 */ 


#ifndef I2C_LCD_H_
#define I2C_LCD_H_

void lcd_init (void);   // initialize lcd

void lcd_send_cmd (char cmd);  // send command to the lcd

void lcd_send_data (char data);  // send data to the lcd

void lcd_send_string (char *str);  // send string to the lcd

void lcd_put_cur(int row, int col);  // put cursor at the entered position row (0 or 1), col (0-15);

void lcd_clear (void);



#endif /* I2C_LCD_H_ */