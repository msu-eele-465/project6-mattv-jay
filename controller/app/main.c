/**
 * @file main.c
 * @brief Main file to run all code.
 */

#include "msp430fr2355.h"
#include <msp430.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// I2C control pins
#define SDA_PIN BIT2
#define SCL_PIN BIT3

// I2C Addresses
#define LED_PERIPHERAL_ADDR 0x48
#define LCD_PERIPHERAL_ADDR 0x49
#define RTC_PERIPHERAL_ADDR 0x68

// ADC constants
#define ADC_CHANNEL BIT7  // LM19 connected to P1.7 (A7)
#define SAMPLE_WINDOW 3   // Moving average window size

// I2C TX and RX Buffers
volatile uint8_t i2c_tx_data[4];
volatile unsigned int i2c_tx_index = 0;
volatile uint8_t i2c_rx_data[2];
volatile unsigned int i2c_rx_index = 0;


// ADC vars
volatile uint16_t temp_samples[SAMPLE_WINDOW] = {0};
volatile uint8_t sample_index = 0;
volatile uint8_t samples_collected = 0;
volatile uint8_t display_fahrenheit = 0;

unsigned int time_in_mode = 0; // in seconds
unsigned int five_min_count = 0;

unsigned int mode = 0; // 0 = off, 1 = heat, 2 = cool, 3 = match

unsigned int refresh_rtc = 0;


void adc_init(void);

void i2c_init(void);
void i2c_write_byte(uint8_t addr, uint8_t byte);
void i2c_write_string(uint8_t addr, const uint8_t data[], unsigned int data_len);
void i2c_read(uint8_t addr, uint8_t reg, unsigned int bytes);
void lcd_write(uint8_t byte);
void send_to_led(uint8_t pattern);

void read_rtc(unsigned int bytes);
void reset_rtc(void);

void gpio_init(void);
uint8_t keypad_get_key(void);
void handle_keypress(uint8_t key);

void timer_init(void);

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD; // Stop watchdog

    i2c_init(); // Initialize I2C with int
    gpio_init(); // Initialize Keypad
    adc_init(); // Initialize ADC
    timer_init(); // Initialize any timers

    __enable_interrupt(); // Enable global interrupts

    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configure port settings
    PM5CTL0 &= ~LOCKLPM5;

    unsigned int peltier_off = 0;
    while (1)
    {
        uint8_t key = keypad_get_key();
        if (key != 0xFF)
        {
            handle_keypress(key); // Handle any detected keypress
        }
        __delay_cycles(100000); // delay

        if (refresh_rtc == 1)
        {
            read_rtc(2);
            refresh_rtc = 0;
        }
        
        if (five_min_count >= 300 && peltier_off == 0)
        {
            mode = 0;
            send_to_led(key);
            lcd_write(key);
            P2OUT &= ~(BIT6 | BIT7);
            peltier_off = 1;
        }

        // if (mode == 3) 
        // {
        //     if (plant_temp > 5 + ambient_temp)
        //     {
        //         P2OUT &= ~BIT7;
        //         P2OUT |= BIT6;
        //     }
        //     else if (plant_temp < 5 + ambient_temp)
        //     {
        //         P2OUT &= ~BIT7;
        //         P2OUT |= BIT6;
        //     }
        //     else
        //     {
        //         P2OUT &= ~(BIT6 | BIT7);
        //     }
        // }
    }
}

void adc_init(void) {
    P1SEL0 |= ADC_CHANNEL;
    P1SEL1 |= ADC_CHANNEL;
    
    ADCCTL0 = ADCSHT_2 | ADCON;
    ADCCTL1 = ADCSHP;
    ADCCTL2 = ADCRES_2;
    ADCMCTL0 = ADCINCH_7;
    
    ADCIE |= ADCIE0;
}

void timer_init(void) {
    TB0CCTL0 = CCIE;
    TB0CCR0 = 16384;  // Approx. 0.5s delay using ACLK
    TB0CTL = TBSSEL_1 | MC_1 | ID_0;

    TB1CTL = TBSSEL__ACLK | MC_2 | TBCLR | ID__8 | CNTL_1; // ACLK, continuous mode, clear TBR, divide by 8, length
                                                           // 12-bit
    TB1CTL &= ~TBIFG; // Clear CCR0 Flag
    TB1CTL |= TBIE; // Enable TB1 Overflow IRQ
}

void i2c_init(void)
{
    P1SEL1 &= ~(SDA_PIN | SCL_PIN);
    P1SEL0 |= SDA_PIN | SCL_PIN; // Set SDA and SCL pins

    UCB0CTLW0 = UCSWRST; // Put eUSCI_B0 in reset mode

    UCB0CTLW0 |= UCMODE_3 | UCMST | UCSSEL_3 | UCTR; // I2C master mode
    UCB0BRW = 10; // Set I2C clock prescaler
    UCB0CTLW1 |= UCASTP_2;      // Auto STOP when UCB0TBCNT reached
    UCB0TBCNT = 0x01;

    UCB0CTLW0 &= ~UCSWRST; // Release eUSCI_B0 from reset

    UCB0IE |= UCTXIE0 | UCRXIE0; // Enable TX and RX interrupts
}

void gpio_init(void)
{
    // Keypad GPIO
    P3DIR |= BIT0 | BIT1 | BIT2 | BIT3; // Rows as outputs
    P3OUT |= BIT0 | BIT1 | BIT2 | BIT3; // Set all rows high (inactive)

    P3DIR &= ~(BIT4 | BIT5 | BIT6 | BIT7); // Columns as inputs
    P3REN |= BIT4 | BIT5 | BIT6 | BIT7; // Enable pull-up resistors
    P3OUT |= BIT4 | BIT5 | BIT6 | BIT7;

    // Peltier device outputs
    P2DIR |= BIT6 | BIT7; // P2.6 is cool relay, P2.7 is heat relay
    P2OUT &= ~(BIT6 | BIT7);
}

void lcd_write(uint8_t byte)
{
    i2c_write_byte(LCD_PERIPHERAL_ADDR, byte); // Use interrupt-based I2C
}

void send_to_led(uint8_t pattern)
{
    i2c_write_byte(LED_PERIPHERAL_ADDR, pattern);
}

void reset_rtc(void)
{
    uint8_t buffer[] = {0x00, 0x00, 0x00}; //0x00 is the subaddress, 0 seconds, 0 minutes
    int buff_size = sizeof(buffer);
    i2c_write_string(0x68, buffer, buff_size);
}

void read_rtc(unsigned int bytes)
{
    i2c_read(RTC_PERIPHERAL_ADDR, 0x00, bytes);

    char buffer[5] = "";
    if (mode != 0) // If current Peltier device mode isn't off 
    {

        unsigned int min = ((i2c_rx_data[1] & 0xF0) >> 4) * 10 + (i2c_rx_data[1] & 0x0F);
        unsigned int sec = ((i2c_rx_data[0] & 0xF0) >> 4) * 10 + (i2c_rx_data[0] & 0x0F);
        time_in_mode = min * 60 + sec;

        if (time_in_mode <= 999)
        {
            snprintf(buffer, sizeof(buffer), "%d", time_in_mode);
            if  (time_in_mode < 10)
            {
                buffer[3] = buffer[0];
                buffer[2] = 0x30;
                buffer[1] = 0x30;
            } 
            else if (time_in_mode < 100) 
            {
                buffer[3] = buffer[1];
                buffer[2] = buffer[0];
                buffer[1] = 0x30;
            }
            else 
            {
                buffer[3] = buffer[2];
                buffer[2] = buffer[1];
                buffer[1] = buffer[0];
            }
            buffer[0] = 'S'; // put S to indicate "seconds spent in mode" to LCD driver
        }
        else 
        {
            buffer[3] = '9';
            buffer[2] = '9';
            buffer[1] = '9';
            buffer[0] = 'S';
        }
    }
    else 
    {
            buffer[3] = '0';
            buffer[2] = '0';
            buffer[1] = '0';
            buffer[0] = 'S';
    }
    i2c_write_string(LCD_PERIPHERAL_ADDR, (uint8_t *)buffer, sizeof(buffer));
}

uint8_t keypad_get_key(void)
{
    const char keys[4][4] = {
        { '1', '2', '3', 'A' }, { '4', '5', '6', 'B' }, { '7', '8', '9', 'C' }, { '*', '0', '#', 'D' }
    };

    int row;
    for (row = 0; row < 4; row++)
    {
        P3OUT |= BIT0 | BIT1 | BIT2 | BIT3; // Set all rows high
        P3OUT &= ~(1 << row); // Pull current row low

        __delay_cycles(1000); // delay

        int col;
        for (col = 0; col < 4; col++)
        {
            if (!(P3IN & (BIT4 << col)))
            { // If key is pressed
                __delay_cycles(10000); // Debounce delay
                while (!(P3IN & (BIT4 << col)))
                    ; // Wait for key release
                return keys[row][col];
            }
        }
    }
    return 0;
}

void handle_keypress(uint8_t key)
{
    switch(key)
    {
        case 'D':
            mode = 0;
            send_to_led(key);
            lcd_write(key);
            P2OUT &= ~(BIT6 | BIT7);
            break;
        case 'A':
            if (five_min_count < 300)
            {
                mode = 1;
                reset_rtc();
                send_to_led(key);
                lcd_write(key);
                P2OUT &= ~BIT6;
                P2OUT |= BIT7;
            }
            break;
        case 'B':
            if (five_min_count < 300)
            {
                mode = 2;
                reset_rtc(); 
                send_to_led(key);
                lcd_write(key);
                P2OUT &= ~BIT7;
                P2OUT |= BIT6;
            }
            break;
        case 'C':
            if (five_min_count < 300)
            {
                mode = 3;
                reset_rtc();
                lcd_write(key);
            }
            break;
        case '*':
            read_rtc(2);
            break;
        case '#':
            reset_rtc();
            break;
    }
}

void i2c_write_byte(uint8_t addr, uint8_t byte)
{
    i2c_tx_data[0] = byte;
    i2c_tx_index = 0;

    UCB0I2CSA = addr; // Set I2C slave address
    UCB0TBCNT = 0x01;
    UCB0CTLW0 |= UCTR | UCTXSTT; // Set to transmit mode and send start condition

    while((UCB0IFG & UCSTPIFG) == 0){}  // Wait for STOP
    UCB0IFG &= ~UCSTPIFG;
}

void i2c_write_string(uint8_t addr, const uint8_t data[], unsigned int data_len)
{
    if (data_len > 4)
    {
        data_len = 4;
    }

    unsigned int i;
    for (i = 0; i < data_len; i++)
    {
        i2c_tx_data[i] = data[i];
    }
    i2c_tx_index = 0;

    UCB0I2CSA = addr; // Set I2C slave address
    UCB0TBCNT = data_len;
    UCB0CTLW0 |= UCTR | UCTXSTT; // Set to transmit mode and send start condition

    while((UCB0IFG & UCSTPIFG) == 0){}  // Wait for STOP
    UCB0IFG &= ~UCSTPIFG;
}

void i2c_read(uint8_t addr, uint8_t reg, unsigned int bytes)
{
    i2c_rx_index = 0;
    i2c_write_byte(addr, reg);

    UCB0CTLW0 &= ~UCTR;
    UCB0TBCNT = bytes;
    UCB0CTLW0 |= UCTXSTT; // Set to read mode and send start condition

    while((UCB0IFG & UCSTPIFG) == 0){}  // Wait for STOP
    UCB0IFG &= ~UCSTPIFG;
}

#pragma vector = EUSCI_B0_VECTOR
__interrupt void EUSCI_B0_I2C_ISR(void)
{
    switch (__even_in_range(UCB0IV, USCI_I2C_UCBIT9IFG))
    {
        case USCI_NONE:
            break;
        case USCI_I2C_UCTXIFG0:
            UCB0TXBUF = i2c_tx_data[i2c_tx_index++]; // Load TX buffer
            break;

        case USCI_I2C_UCRXIFG0:
            i2c_rx_data[i2c_rx_index++] = UCB0RXBUF; // Read received byte
            break;

        default:
            break;
    }
}

#pragma vector = TIMER0_B0_VECTOR
__interrupt void TIMER_ISR(void) {
    ADCCTL0 |= ADCENC | ADCSC;
}

#pragma vector = TIMER1_B1_VECTOR
__interrupt void ISR_TB1_OVERFLOW(void)
{
    refresh_rtc = 1;
    five_min_count = five_min_count + 1;
    
    TB1CTL &= ~TBIFG;
}

#pragma vector = ADC_VECTOR
__interrupt void ADC_ISR(void) {
    uint16_t adc_value = ADCMEM0;
    
    temp_samples[sample_index] = adc_value;
    sample_index = (sample_index + 1) % SAMPLE_WINDOW;
    
    if (samples_collected < SAMPLE_WINDOW) {
        samples_collected++;
    }
}
