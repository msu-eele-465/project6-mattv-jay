#include "msp430fr2355.h"
#include <msp430.h>
#include <stdint.h>

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
volatile uint8_t i2c_tx_data[2];
volatile uint8_t i2c_tx_index = 0;
volatile uint8_t i2c_rx_data = 0;

// ADC vars
volatile uint16_t temp_samples[SAMPLE_WINDOW] = {0};
volatile uint8_t sample_index = 0;
volatile uint8_t samples_collected = 0;
volatile uint8_t display_fahrenheit = 0;

unsigned int mode = 0; // 0 = off, 1 = heat, 2 = cool, 3 = match 

void adc_init(void);

void i2c_init(void);
void i2c_write_byte_interrupt(uint8_t addr, uint8_t byte);
void i2c_read_interrupt(uint8_t addr, uint8_t byte);
void lcd_write(uint8_t byte);
void send_to_led(uint8_t pattern);
void read_rtc(void);

void keypad_init(void);
uint8_t keypad_get_key(void);
void handle_keypress(uint8_t key);

void timer_init(void);

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD; // Stop watchdog

    i2c_init(); // Initialize I2C with int
    keypad_init(); // Initialize Keypad
    adc_init(); // Initialize ADC
    timer_init(); // Initialize any timers

    __enable_interrupt(); // Enable global interrupts

    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configure port settings
    PM5CTL0 &= ~LOCKLPM5;

    while (1)
    {
        uint8_t key = keypad_get_key();
        if (key != 0xFF)
        {
            handle_keypress(key); // Handle any detected keypress
        }
        __delay_cycles(100000); // delay
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
}

void i2c_init(void)
{
    P1SEL1 &= ~(SDA_PIN | SCL_PIN);
    P1SEL0 |= SDA_PIN | SCL_PIN; // Set SDA and SCL pins
    UCB0CTLW0 = UCSWRST; // Put eUSCI_B0 in reset mode
    UCB0CTLW0 |= UCMODE_3 | UCMST | UCSSEL_3 | UCTR; // I2C master mode
    UCB0BRW = 10; // Set I2C clock prescaler
    UCB0CTLW0 &= ~UCSWRST; // Release eUSCI_B0 from reset

    UCB0IE |= UCTXIE0 | UCRXIE0; // Enable TX and RX interrupts
}

void keypad_init(void)
{
    P3DIR |= BIT0 | BIT1 | BIT2 | BIT3; // Rows as outputs
    P3OUT |= BIT0 | BIT1 | BIT2 | BIT3; // Set all rows high (inactive)

    P3DIR &= ~(BIT4 | BIT5 | BIT6 | BIT7); // Columns as inputs
    P3REN |= BIT4 | BIT5 | BIT6 | BIT7; // Enable pull-up resistors
    P3OUT |= BIT4 | BIT5 | BIT6 | BIT7;
}

void lcd_write(uint8_t byte)
{
    i2c_write_byte_interrupt(LCD_PERIPHERAL_ADDR, byte); // Use interrupt-based I2C
}

void send_to_led(uint8_t pattern)
{
    i2c_write_byte_interrupt(LED_PERIPHERAL_ADDR, pattern);
}

void read_rtc(void)
{
    i2c_read_interrupt(RTC_PERIPHERAL_ADDR, 0x00);
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
            break;
        case 'A':
            mode = 1;
            send_to_led(key);
            lcd_write(key);
            break;
        case 'B':
            mode = 2; 
            send_to_led(key);
            lcd_write(key);
            break;
        case 'C':
            mode = 3;
            lcd_write(key);
            break;
        case '*':
            read_rtc();
            break;
    }
}

void i2c_write_byte_interrupt(uint8_t addr, uint8_t byte)
{
    i2c_tx_data[0] = addr;
    i2c_tx_data[1] = byte;
    i2c_tx_index = 1;

    UCB0I2CSA = addr; // Set I2C slave address
    UCB0CTLW0 |= UCTR | UCTXSTT; // Set to transmit mode and send start condition
}

void i2c_read_interrupt(uint8_t addr, uint8_t byte)
{
    i2c_write_byte_interrupt(addr, byte);

    while((UCB0IFG & UCSTPIFG) == 0){}  // Wait for STOP
    UCB0IFG &= ~UCSTPIFG;               // Clear STOP flag

    UCB0I2CSA = addr; // Set I2C slave address
    UCB0CTLW0 &= ~UCTR;
    UCB0CTLW0 |= UCTXSTT; // Set to read mode and send start condition
}

#pragma vector = EUSCI_B0_VECTOR
__interrupt void EUSCI_B0_I2C_ISR(void)
{
    switch (__even_in_range(UCB0IV, USCI_I2C_UCBIT9IFG))
    {
        case USCI_NONE:
            break;
        case USCI_I2C_UCTXIFG0:
            if (i2c_tx_index < 2)
            {
                UCB0TXBUF = i2c_tx_data[i2c_tx_index++]; // Load TX buffer
            }
            else
            {
                UCB0CTLW0 |= UCTXSTP; // Send stop condition
                UCB0IFG &= ~UCTXIFG0; // Clear TX interrupt flag
            }
            break;

        case USCI_I2C_UCRXIFG0:
            i2c_rx_data = UCB0RXBUF; // Read received byte
            UCB0CTLW0 |= UCTXSTP; // Send stop condition
            UCB0IFG &= ~UCRXIFG0; // Clear TX interrupt flag
            break;

        default:
            break;
    }
}

#pragma vector = TIMER0_B0_VECTOR
__interrupt void TIMER_ISR(void) {
    ADCCTL0 |= ADCENC | ADCSC;
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
