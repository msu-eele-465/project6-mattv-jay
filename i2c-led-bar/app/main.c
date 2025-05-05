/**
 * @file main.c
 * @brief Main file to run all code.
 */

#include <msp430fr2310.h>
#include <stdint.h>
#include <stdbool.h>

#define I2C_ADDR 0x48

unsigned int pattern_num = 0; // Tracks which pattern is active (0 = off, 1 =
                              // fill left, 2 = fill right)
uint8_t pattern = 0b00000000; // For manipulating active pattern

char key = '\0';

unsigned int time_since_active = 3;

/**
 * Initializes all GPIO ports.
 */
void initGPIO(void)
{
    // Set ports 1.0, 1.1, 1.4-1.7, 2.0, 2.6, 2.7 as outputs
    P1DIR |= BIT0 | BIT1 | BIT4 | BIT5 | BIT6 | BIT7;
    P2DIR |= BIT0 | BIT6 | BIT7;

    // Set GPIO outputs to zero
    P1OUT &= ~(BIT0 | BIT1 | BIT4 | BIT5 | BIT6 | BIT7);
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
void initTimer(void)
{
    // ACLK, continuous mode, clear TBR, divide by 4, length 12-bit
    TB0CTL = TBSSEL__ACLK | MC_2 | TBCLR | ID__4 | CNTL_1;
    TB0CTL &= ~TBIFG; // Clear CCR0 Flag
    TB0CTL |= TBIE; // Enable TB0 Overflow IRQ

    // ACLK, continuous mode, clear TBR, divide by 8, length 12-bit
    TB1CTL = TBSSEL__ACLK | MC_2 | TBCLR | ID__8 | CNTL_1;
    TB1CTL &= ~TBIFG; // Clear CCR0 Flag
    TB1CTL |= TBIE; // Enable TB1 Overflow IRQ
}

/**
 * Sets all I2C parameters.
 */
void initI2C(void)
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
 * A longer description, with more discussion of the function
 * that might be useful to those using or modifying it.
 */
int main(void)
{
    const uint8_t PATTERNS[] = { { 0b00000000 }, { 0b10000000 }, { 0b00000001 } };

    WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timer

    // Initialize ports and other subsystems
    initGPIO();
    initTimer();
    initI2C();

    __enable_interrupt(); // Enable Maskable IRQs

    while (true)
    {
        if (key != '\0')
        {
            if (key == 'D') // off
            {
                pattern_num = 0;
                pattern = PATTERNS[pattern_num];
            }
            else if (key == 'A') // fill right
            {
                pattern_num = 1;
                pattern = PATTERNS[pattern_num];
            }
            else if (key == 'B') // fill left
            {
                pattern_num = 2;
                pattern = PATTERNS[pattern_num];
            }

            // Set LED bar outputs
            P1OUT = (P1OUT & 0b11111100) | (pattern & 0b00000011);
            P1OUT = (P1OUT & 0b00001111) | ((pattern & 0b00111100) << 2);
            P2OUT = (P2OUT & 0b00111111) | (pattern & 0b11000000);

            key = '\0';
        }
    }
}

/**
 * Timer B0 Overflow Interrupt.
 *
 * Runs every 0.5 seconds. Updates LED bar
 * display based on currently selected pattern.
 */
#pragma vector = TIMER0_B1_VECTOR
__interrupt void ISR_TB0_OVERFLOW(void)
{
    switch (pattern_num)
    {
        case 1: // fill right
            if (pattern == 0b11111111)
            {
                pattern = ~pattern;
            }
            pattern = pattern >> 1;
            pattern ^= 0b10000000;
            break;
        case 2: // fill left
            if (pattern == 0b11111111)
            {
                pattern = ~pattern;
            }
            pattern = pattern << 1;
            pattern ^= 0b00000001;
            break;
        default: // off
            pattern = 0b00000000;
            break;
    }

    // Set LED bar outputs
    P1OUT = (P1OUT & 0b11111100) | (pattern & 0b00000011);
    P1OUT = (P1OUT & 0b00001111) | ((pattern & 0b00111100) << 2);
    P2OUT = (P2OUT & 0b00111111) | (pattern & 0b11000000);

    TB0CTL &= ~TBIFG; // Clear CCR0 Flag
}

/**
 * Timer B1 Overflow Interrupt.
 *
 * Runs every second. Starts flashing status LED
 * 3 seconds after receiving anything over I2C.
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
 */
#pragma vector = EUSCI_B0_VECTOR
__interrupt void EUSCI_B0_I2C_ISR(void)
{
    key = UCB0RXBUF;
    P2OUT |= BIT0;
    time_since_active = 0;
}
