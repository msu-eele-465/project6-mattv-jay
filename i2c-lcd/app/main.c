/**
 * @file main.c
 * @brief Main file to run all code.
 */

#include <msp430fr2310.h>
#include <stdint.h>
#include <stdbool.h>
#include "intrinsics.h"
#include "src/lcd_driver.h"

#define I2C_ADDR 0x49

#define RS_HIGH P2OUT |= BIT6
#define RS_LOW P2OUT &= ~BIT6
#define E_HIGH P2OUT |= BIT7
#define E_LOW P2OUT &= ~BIT7

#define LCD_DATA P1OUT
#define DB4 BIT4
#define DB5 BIT5
#define DB6 BIT6
#define DB7 BIT7

bool unlocked = false;
char key = '0';

unsigned int time_since_active = 3;

uint8_t buffer[3] = {0};
unsigned int index;

char temp_out[] = "00.0";

/**
 * Initializes all GPIO ports.
 */
void init_gpio(void)
{
    // Set ports 1.4-1.7, 2.0, 2.6, 2.7 as outputs
    P1DIR |= BIT4 | BIT5 | BIT6 | BIT7;
    P2DIR |= BIT0 | BIT6 | BIT7;

    // Set GPIO outputs to zero
    P1OUT &= ~(BIT4 | BIT5 | BIT6 | BIT7);
    P2OUT &= ~(BIT0 | BIT6 | BIT7);

    // I2C pins
    P1SEL0 |= BIT2 | BIT3;
    P1SEL1 &= ~(BIT2 | BIT3);

    // Disable the GPIO power-on default high-impedance mdoe to activate
    // previously configure port settings
    PM5CTL0 &= ~LOCKLPM5;
}

/**
 * Initializes all timers.
 */
void init_timer(void)
{
    TB1CTL = TBSSEL__ACLK | MC_2 | TBCLR | ID__8 | CNTL_1; // ACLK, continuous mode, clear TBR, divide by 8, length
                                                           // 12-bit
    TB1CTL &= ~TBIFG; // Clear CCR0 Flag
    TB1CTL |= TBIE; // Enable TB1 Overflow IRQ
}

/**
 * Sets all I2C parameters.
 */
void init_i2c(void)
{
    UCB0CTLW0 = UCSWRST; // Software reset enabled
    UCB0CTLW0 |= UCMODE_3 | UCSYNC; // I2C mode, sync mode
    UCB0I2COA0 = I2C_ADDR | UCOAEN; // Own Address and enable
    UCB0CTLW0 &= ~UCSWRST; // clear reset register
    UCB0IE |= UCRXIE; // Enable I2C read interrupt
}

/**
 * Main function.
 *
 * Starts by initializing subsystems and ports. Handles
 * I2C data after being received and outputs to LCD display.
 */
int main(void)
{
    // uint8_t cursor = 0b00001100;

    const char *PATTERNS[] = { "static",       "toggle",        "up counter",     "in and out",
                               "down counter", "rotate 1 left", "rotate 7 right", "fill left" };

    WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timer

    // Initialize ports and other subsystems
    init_gpio();
    init_timer();
    init_i2c();

    init_lcd();
    send_cmd(0x01);
    __delay_cycles(10000);

    __enable_interrupt(); // Enable Maskable IRQs

    unsigned int state = 0; //0 = normal, 1 = window size input, 2 = LED pattern input 
    unsigned int pattern_num = 0;
    char n = '3';

    while (true)
    {
        if (unlocked && key != '\0')
        {
            if (state == 0)
            {
                if (key == '*') 
                {
                    state = 1;
                    __disable_interrupt();
                    send_cmd(0x01);
                    __delay_cycles(10000); // Wait 1.6 ms
                    send_string("set window size");
                    __enable_interrupt();
                }
                else if (key == '#') 
                {
                    state = 2;
                    __disable_interrupt();
                    send_cmd(0x01);
                    __delay_cycles(10000); // Wait 1.6 ms
                    send_string("set pattern");
                    __enable_interrupt();
                }
            }
            else if (state == 1) 
            {
                if (key > '0' && key <= '9')
                {
                    n = key;
                    __disable_interrupt();
                    send_cmd(0x01);
                    __delay_cycles(10000); // Wait 1.6 ms
                    send_string(PATTERNS[pattern_num]);
                    __enable_interrupt();
                    state = 0;
                }
            } 
            else if (state == 2) 
            {
                if (key >= '0' && key < '8') 
                {
                    pattern_num = (unsigned int)(key - '0');
                    __disable_interrupt();
                    send_cmd(0x01);
                    __delay_cycles(10000); // Wait 1.6 ms
                    send_string(PATTERNS[pattern_num]);
                    __enable_interrupt();
                    state = 0;
                }
            }

            send_cmd(0xC0);
            send_string("T=");
            send_string(temp_out);
            send_char(0xDF);
            send_char('C');
            send_cmd(0xCD);
            send_string("N=");
            send_char(n);
            key = '\0';
        }
        if (unlocked)
        {
            send_cmd(0xC0);
            send_string("T=");
            send_string(temp_out);
            send_char(0xDF);
            send_char('C');
        }
        buffer[0] = 0;
        buffer[1] = 0;
        buffer[2] = 0;
        index = 0;
    }
}

/**
 * Timer B1 Overflow Interrupt.
 *
 * Runs every second. Starts flashing status LED
 * 3 seconds after receiving something over I2C.
 */
#pragma vector = TIMER1_B1_VECTOR
__interrupt void ISR_TB1_OVERFLOW(void)
{
    if (time_since_active >= 3)
    {
        P2OUT ^= BIT0;
    }
    time_since_active++;

    TB1CTL &= ~TBIFG; // Clear CCR0 Flag
}

/**
 * I2C RX Interrupt.
 *
 * Stores value received over I2C in global var "key".
 * If 'U' is received over I2C, set the "unlocked" var.
 */
uint8_t data_in;
#pragma vector = EUSCI_B0_VECTOR
__interrupt void EUSCI_B0_I2C_ISR(void)
{
    data_in = UCB0RXBUF;
    buffer[index++] = data_in;
    if (buffer[2] != 0 && unlocked) 
    {
        temp_out[0] = (char)buffer[0];
        temp_out[1] = (char)buffer[1];
        temp_out[3] = (char)buffer[2];
    }
    else if (data_in == 'U')
    {
        unlocked = true;
    }
    else
    {
        key = (char)data_in;
    }
    P2OUT |= BIT0;
    time_since_active = 0;
}