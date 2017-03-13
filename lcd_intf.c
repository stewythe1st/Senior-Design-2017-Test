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


/***********************************************************
* Headers
************************************************************/
#include "lcd_intf.h"

/***********************************************************
* LCD Setup
************************************************************/
void lcd_setup() {
    volatile uint32_t j;

    // set all LCD pin modes to output
    //GPIO_setPadConfig(LCD_EN, GPIO_PIN_TYPE_PULLUP);
    GPIO_setDirectionMode(LCD_EN, GPIO_DIR_MODE_OUT);
    //GPIO_setPadConfig(LCD_RS, GPIO_PIN_TYPE_PULLUP);
    GPIO_setDirectionMode(LCD_RS, GPIO_DIR_MODE_OUT);
    //GPIO_setPadConfig(LCD_DB0, GPIO_PIN_TYPE_PULLUP);
    GPIO_setDirectionMode(LCD_DB0, GPIO_DIR_MODE_OUT);
    //GPIO_setPadConfig(LCD_DB1, GPIO_PIN_TYPE_PULLUP);
    GPIO_setDirectionMode(LCD_DB1, GPIO_DIR_MODE_OUT);
    //GPIO_setPadConfig(LCD_DB2, GPIO_PIN_TYPE_PULLUP);
    GPIO_setDirectionMode(LCD_DB2, GPIO_DIR_MODE_OUT);
    //GPIO_setPadConfig(LCD_DB3, GPIO_PIN_TYPE_PULLUP);
    GPIO_setDirectionMode(LCD_DB3, GPIO_DIR_MODE_OUT);
    //GPIO_setPadConfig(LCD_DB4, GPIO_PIN_TYPE_PULLUP);
    GPIO_setDirectionMode(LCD_DB4, GPIO_DIR_MODE_OUT);
    //GPIO_setPadConfig(LCD_DB5, GPIO_PIN_TYPE_PULLUP);
    GPIO_setDirectionMode(LCD_DB5, GPIO_DIR_MODE_OUT);
    //GPIO_setPadConfig(LCD_DB6, GPIO_PIN_TYPE_PULLUP);
    GPIO_setDirectionMode(LCD_DB6, GPIO_DIR_MODE_OUT);
    //GPIO_setPadConfig(LCD_DB7, GPIO_PIN_TYPE_PULLUP);
    GPIO_setDirectionMode(LCD_DB7, GPIO_DIR_MODE_OUT);

    // delay for setup
    for(j=10000; j>0; j--);

    //
    lcd_write(0, 0x34);
    for(j=100; j>0; j--);

    // set display on, cursor off, blink off
    lcd_write(0, 0x0C);
    for(j=100; j>0; j--);

    // clear display
    lcd_write(0, 0x01);
    for(j=100; j>0; j--);

    lcd_write(0, 0x06);
    for(j=1000; j>0; j--);
}


/***********************************************************
* LCD Clear
************************************************************/
void lcd_clear() {
    volatile uint32_t j;
    lcd_write( 0, 0x01 );
    for(j=10000; j>0; j--);
}


/***********************************************************
* LCD Write
************************************************************/
void lcd_write( bool regSelect, uint16_t LCD_data) {
    pin_write( LCD_RS, regSelect );
    pin_write( LCD_EN, 1 );
    lcd_db_write( LCD_data );
    pin_write( LCD_EN, 0 );
    lcd_db_write( 0xFF );
    pin_write( LCD_RS, 1 );
}


/***********************************************************
* LCD Write Char to Data Bus
************************************************************/
void lcd_db_write( uint16_t LCD_data ) {
    pin_write( LCD_DB0, LCD_data & 0x01 );
    pin_write( LCD_DB1, LCD_data & 0x02 );
    pin_write( LCD_DB2, LCD_data & 0x04 );
    pin_write( LCD_DB3, LCD_data & 0x08 );
    pin_write( LCD_DB4, LCD_data & 0x10 );
    pin_write( LCD_DB5, LCD_data & 0x20 );
    pin_write( LCD_DB6, LCD_data & 0x40 );
    pin_write( LCD_DB7, LCD_data & 0x80 );
}


/***********************************************************
* LCD High-Level Print
************************************************************/
void lcd_print( char* text )
    {
    volatile uint32_t i = 0;
    while(text[i] != '\0')
        {
        lcd_write( 1, text[ i ] );
        i++;
        }
    }


/***********************************************************
* LCD Write to GPIO Pin
************************************************************/
void pin_write( uint16_t port, bool val ) {
    GPIO_writePin( port, val );
}
