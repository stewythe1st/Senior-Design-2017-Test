/************************************************************************************************************
* LCD Interface
*
* Author:
* Stuart Miller
* Missouri University of Science & Technology
* Computer Engineering
* 2017
*
************************************************************************************************************/
#ifndef _LCD_H_
#define _LCD_H_


/***********************************************************
* Headers
************************************************************/
#include "driverlib.h"
#include "device.h"


/***********************************************************
* GPIO pin definitions
************************************************************/
#define LCD_EN  4
#define LCD_RS  62
#define LCD_DB0 21
#define LCD_DB1 20
#define LCD_DB2 17
#define LCD_DB3 16
#define LCD_DB4 15
#define LCD_DB5 14
#define LCD_DB6 13
#define LCD_DB7 12


/***********************************************************
* LCD Functions
************************************************************/
void lcd_setup();
void lcd_write(bool regSelect, uint16_t LCD_data);
void lcd_db_write( uint16_t LCD_data );
void lcd_print( char* text );
void lcd_clear();
void pin_write( uint16_t port, bool val );


#endif
