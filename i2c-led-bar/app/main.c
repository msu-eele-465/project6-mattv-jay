/**
 * @file main.c
 * @brief Main file to run all code.
 */

#include <msp430fr2310.h>
#include <stdint.h>
#include <stdbool.h>

#define I2C_ADDR 0x48

unsigned int pattern_num = 0; // Tracks which pattern is active
uint8_t pattern = 0b00000000; // For manipulating active pattern

bool unlocked = false;
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
    TB0CTL = TBSSEL__ACLK | MC_1 | TBCLR | ID__2; // ACLK, up mode, clear TBR, divide by 2
    TB0CCR0 = 16384; // Set up 1.0s period
    TB0CCTL0 &= ~CCIFG; // Clear CCR0 Flag
    TB0CCTL0 |= CCIE; // Enable TB0 CCR0 Overflow IRQ

    TB1CTL = TBSSEL__ACLK | MC_2 | TBCLR | ID__8 | CNTL_1; // ACLK, continuous mode, clear TBR, divide by 8, length
                                                           // 12-bit
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
    int trans_period = 16384;
    const float PER_FRACTIONS[] = { 1.0, 1.0, 0.5, 0.5, 0.25, 1.5, 0.5, 1.0 };

    uint8_t pattern_store[] = { { 0b10101010 }, { 0b10101010 }, { 0b00000000 }, { 0b00011000 },
                                { 0b11111111 }, { 0b00000001 }, { 0b01111111 }, { 0b00000000 } };

    const uint8_t PATTERNS[] = { { 0b10101010 }, { 0b10101010 }, { 0b00000000 }, { 0b00011000 },
                                 { 0b11111111 }, { 0b00000001 }, { 0b01111111 }, { 0b00000000 } };

    WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timer

    // Initialize ports and other subsystems
    initGPIO();
    initTimer();
    initI2C();

    __enable_interrupt(); // Enable Maskable IRQs

    while (true)
    {
        if (unlocked)
        {
            if (key == 'A')
            {
                if (trans_period != 4096)
                {
                    trans_period -= 4096;
                    key = '\0';
                }
            }
            else if (key == 'B')
            {
                trans_period += 4096;
                key = '\0';
            }
            else if ((key - '0') >= 0 && (key - '0') < 8)
            {
                if (pattern_num == key - '0')
                {
                    pattern = PATTERNS[(unsigned int)(key - '0')];
                }
                else
                {
                    if (pattern_num != 0 && pattern != 0b00000000) // Handles initial condition
                    {
                        pattern_store[pattern_num] = pattern;
                    }
                    pattern = pattern_store[(unsigned int)(key - '0')];
                    pattern_num = (unsigned int)(key - '0');
                }
                // Set LED bar outputs
                P1OUT = (P1OUT & 0b11111100) | (pattern & 0b00000011);
                P1OUT = (P1OUT & 0b00001111) | ((pattern & 0b00111100) << 2);
                P2OUT = (P2OUT & 0b00111111) | (pattern & 0b11000000);
                key = '\0';
            }
            TB0CCR0 = (unsigned int)(trans_period * PER_FRACTIONS[pattern_num]);
        }
    }
}

/**
 * Timer B0 Compare Interrupt.
 *
 * Runs periodically according to the transistion
 * period ("trans_period") and the corresponding
 * patterns fractional period. Updates LED bar
 * display pattern based on currently selected
 * pattern.
 */
bool dir_out = false;
#pragma vector = TIMER0_B0_VECTOR
__interrupt void ISR_TB0_CCR0(void)
{
    switch (pattern_num)
    {
        case 1:
            pattern = ~pattern;
            break;
        case 2:
            pattern++;
            break;
        case 3:
            if (pattern == 0b00011000 || pattern == 0b10000001)
            {
                dir_out = !dir_out;
            }
            if (dir_out)
            {
                pattern = ((pattern & 0b11110000) << 1) | ((pattern & 0b00001111) >> 1);
            }
            else
            {
                pattern = ((pattern & 0b11110000) >> 1) | ((pattern & 0b00001111) << 1);
            }
            break;
        case 4:
            pattern--;
            break;
        case 5:
            if (pattern == 0b10000000)
            {
                pattern ^= 0b10000001;
            }
            else
            {
                pattern = pattern << 1;
            }
            break;
        case 6:
            if (pattern == 0b11111110)
            {
                pattern = 0b01111111;
            }
            else
            {
                pattern = pattern >> 1;
                pattern ^= 0b10000000;
            }
            break;
        case 7:
            if (pattern == 0b11111111)
            {
                pattern = ~pattern;
            }
            pattern = pattern << 1;
            pattern ^= 0b00000001;
            break;
    }

    // Set LED bar outputs
    P1OUT = (P1OUT & 0b11111100) | (pattern & 0b00000011);
    P1OUT = (P1OUT & 0b00001111) | ((pattern & 0b00111100) << 2);
    P2OUT = (P2OUT & 0b00111111) | (pattern & 0b11000000);

    TB0CCTL0 &= ~CCIFG; // Clear CCR0 Flag
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
#pragma vector = EUSCI_B0_VECTOR
__interrupt void EUSCI_B0_I2C_ISR(void)
{
    key = UCB0RXBUF;
    if (key == 'U')
    {
        unlocked = true;
    }
    P2OUT |= BIT0;
    time_since_active = 0;
}