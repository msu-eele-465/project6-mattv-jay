/**
 * @file lcd_driver.c
 * @brief Contains code required to use LCD display with MSP430.
 */

#include "intrinsics.h"
#include <msp430fr2310.h>
#include <stdbool.h>

#define RS_HIGH P2OUT |= BIT6
#define RS_LOW P2OUT &= ~BIT6
#define E_HIGH P2OUT |= BIT7
#define E_LOW P2OUT &= ~BIT7

#define LCD_DATA P1OUT
#define DB4 BIT4
#define DB5 BIT5
#define DB6 BIT6
#define DB7 BIT7

void enable_lcd(void);
void send_cmd(unsigned char cmd);
void send_char(unsigned char character);
void send_string(const char *str);

/**
 * Initializes LCD display.
 */
void init_lcd(void)
{
    __delay_cycles(30000); // Wait 30 ms
    // Function set
    LCD_DATA |= DB5;
    enable_lcd();
    send_cmd(0x28);
    __delay_cycles(100); // Wait 100 micro seconds
    // Display on
    send_cmd(0x0C);
    __delay_cycles(100); // Wait 100 micro seconds
    // Display clear
    send_cmd(0x01);
    __delay_cycles(1600); // Wait 1.6 ms
    // Entry mode set
    send_cmd(0x06);
}

/**
 * Toggle LCD enable bit.
 */
void enable_lcd(void)
{
    E_HIGH;
    E_LOW;
}

/**
 * Send an instruction to LCD.
 */
void send_cmd(unsigned char cmd)
{
    RS_LOW;
    LCD_DATA = (LCD_DATA & 0x0F) | (cmd & 0xF0);
    enable_lcd();
    LCD_DATA = (LCD_DATA & 0x0F) | ((cmd & 0x0F) << 4);
    enable_lcd();
}

/**
 * Send a character to LCD.
 */
void send_char(unsigned char character)
{
    RS_HIGH;
    LCD_DATA = (LCD_DATA & 0x0F) | (character & 0xF0);
    enable_lcd();
    LCD_DATA = (LCD_DATA & 0x0F) | ((character & 0x0F) << 4);
    enable_lcd();
}

/**
 * Send a string to LCD.
 */
void send_string(const char *str)
{
    unsigned int i;
    for (i = 0; str[i] != 0; i++)
    {
        send_char(str[i]);
    }
}
