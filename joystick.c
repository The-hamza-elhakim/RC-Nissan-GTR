#define F_CPU 20000000UL  // Intended 20 MHz, but if your clock is actually 8MHz, this mapping uses a 1MHz tick (8MHz/8)

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

// -----------------------
// TM1637 Display Settings
// -----------------------
#define TM1637_CLK_PIN 2  // PA2
#define TM1637_DIO_PIN 3  // PA3

#define TM1637_BRIGHTNESS 0x0F
#define TM1637_CMD_SET_DATA 0x40
#define TM1637_CMD_SET_ADDR 0xC0
#define TM1637_CMD_DISPLAY_CTRL 0x88

// 7-segment lookup for digits 0-9
const uint8_t digitToSegment[10] = {
    0x3F, 0x06, 0x5B, 0x4F, 0x66,
    0x6D, 0x7D, 0x07, 0x7F, 0x6F
};

// -----------------------
// Function Prototypes
// -----------------------
void tm1637_start(void);
void tm1637_stop(void);
void tm1637_write_byte(uint8_t b);
void tm1637_init(void);
void tm1637_display(uint8_t segments[]);
void display_value(uint16_t value);

void adc_init(void);
uint16_t read_adc(void);

void servo_init(void);
void manual_servo_test(void);  // Uncomment to test PWM manually

// -----------------------
// Main
// -----------------------
int main(void) {
    // --- CLOCK CONFIGURATION (Intended 20 MHz) ---
    CCP = 0xD8;  // Unlock protected registers
    CLKCTRL.OSCHFCTRLA = 0b00010100;  // Set internal oscillator to 20 MHz
    while (CLKCTRL.MCLKSTATUS & 0x01) { ; }  // Wait for clock to stabilize

    // --- SERVO PWM INIT (TCA0 on PA0: WO0) ---
    servo_init();
    // Configure PA0 as output for servo PWM
    PORTA.DIRSET = (1 << 0);  // PA0

    // --- TM1637 Display INIT (using PA2 & PA3) ---
    PORTA.DIRSET |= (1 << TM1637_CLK_PIN) | (1 << TM1637_DIO_PIN);
    PORTA.OUTSET |= (1 << TM1637_CLK_PIN) | (1 << TM1637_DIO_PIN);
    tm1637_init();

    // --- ADC INIT (using PD2 as ADC input) ---
    PORTD.DIRCLR = (1 << 2);  // PD2 as input
    adc_init();

    _delay_ms(500);  // Allow peripherals to settle
    display_value(0);

    // Uncomment the next line to manually cycle the PWM value for testing:
    // manual_servo_test();

    // Main loop: update servo PWM based on ADC reading.
    while (1) {
        uint16_t adc_value = read_adc();  // Expected range roughly 0 to 4095
        
        // Compensate for 3.3V vs 5V difference (Scale up by ~1.5x)
        // This multiplies by 3 and divides by 2 to get 1.5x scaling
        uint32_t compensated_value = (adc_value * 3UL) / 2UL;
        
        // Clamp to maximum ADC range
        if (compensated_value > 4095UL) {
            compensated_value = 4095UL;
        }
        
        // Scale compensated ADC value from 0-4095 to 0-255
        uint8_t scaled_value = (uint8_t)((compensated_value * 255UL) / 4095UL);

        // Update the display with the scaled ADC value (0-255)
        display_value(scaled_value);

        // Map ADC reading to servo pulse width with compensation applied:
        uint16_t servoPulse = 500 + ((compensated_value * 2000UL) / 4095UL);
        TCA0.SINGLE.CMP0 = servoPulse;

        _delay_ms(100);
    }
    
    return 0;
}

// -----------------------
// ADC Functions
// -----------------------

void adc_init(void) {
    SREG = 0b10000000;              // Enable global interrupts
    VREF.ADC0REF = 0b10000101;        // Set ADC reference to VDD
    ADC0.INTCTRL = 0b00000001;        // Enable ADC interrupt flag polling
    ADC0.MUXPOS = 0x02;               // Use PD2 (AIN2)
    ADC0.CTRLC = 0x00;                // Minimum ADC clock division
    ADC0.CTRLA = 0b00000011;          // 12-bit resolution, free-running mode
    ADC0.COMMAND = 0x01;              // Start conversion
}

uint16_t read_adc(void) {
    while (!(ADC0.INTFLAGS & 0x01)) { ; }  // Wait for conversion complete
    uint16_t value = ADC0.RES;
    ADC0.INTFLAGS = 0x01;                  // Clear ADC flag
    return value;
}

// -----------------------
// TM1637 Display Functions
// -----------------------
void tm1637_init(void) {
    _delay_ms(50);
    tm1637_start();
    tm1637_write_byte(TM1637_CMD_SET_DATA);
    tm1637_stop();
    tm1637_start();
    tm1637_write_byte(TM1637_CMD_DISPLAY_CTRL | (TM1637_BRIGHTNESS & 0x07));
    tm1637_stop();
    uint8_t clear_segments[4] = {0, 0, 0, 0};
    tm1637_display(clear_segments);
}

void display_value(uint16_t value) {
    uint8_t digits[4];
    digits[0] = (value / 1000) % 10;
    digits[1] = (value / 100) % 10;
    digits[2] = (value / 10) % 10;
    digits[3] = value % 10;
    uint8_t segments[4];
    for (uint8_t i = 0; i < 4; i++) {
        segments[i] = digitToSegment[digits[i]];
    }
    if (value < 1000) segments[0] = 0;
    if (value < 100) segments[1] = 0;
    if (value < 10) segments[2] = 0;
    tm1637_display(segments);
}

void tm1637_display(uint8_t segments[]) {
    tm1637_start();
    tm1637_write_byte(TM1637_CMD_SET_ADDR);
    for (uint8_t i = 0; i < 4; i++) {
        tm1637_write_byte(segments[i]);
    }
    tm1637_stop();
}

void tm1637_start(void) {
    PORTA.OUTSET = (1 << TM1637_DIO_PIN);
    PORTA.OUTSET = (1 << TM1637_CLK_PIN);
    _delay_us(2);
    PORTA.OUTCLR = (1 << TM1637_DIO_PIN);
    _delay_us(2);
    PORTA.OUTCLR = (1 << TM1637_CLK_PIN);
    _delay_us(2);
}

void tm1637_stop(void) {
    PORTA.OUTCLR = (1 << TM1637_CLK_PIN);
    _delay_us(2);
    PORTA.OUTCLR = (1 << TM1637_DIO_PIN);
    _delay_us(2);
    PORTA.OUTSET = (1 << TM1637_CLK_PIN);
    _delay_us(2);
    PORTA.OUTSET = (1 << TM1637_DIO_PIN);
    _delay_us(2);
}

void tm1637_write_byte(uint8_t b) {
    for (uint8_t i = 0; i < 8; i++) {
        PORTA.OUTCLR = (1 << TM1637_CLK_PIN);
        _delay_us(2);
        if (b & 0x01)
            PORTA.OUTSET = (1 << TM1637_DIO_PIN);
        else
            PORTA.OUTCLR = (1 << TM1637_DIO_PIN);
        _delay_us(2);
        PORTA.OUTSET = (1 << TM1637_CLK_PIN);
        _delay_us(2);
        b >>= 1;
    }
    PORTA.OUTCLR = (1 << TM1637_CLK_PIN);
    PORTA.DIRCLR = (1 << TM1637_DIO_PIN);
    _delay_us(5);
    PORTA.OUTSET = (1 << TM1637_CLK_PIN);
    _delay_us(2);
    PORTA.OUTCLR = (1 << TM1637_CLK_PIN);
    PORTA.DIRSET = (1 << TM1637_DIO_PIN);
    _delay_us(2);
}

// -----------------------
// Servo (PWM) Functions
// -----------------------
void servo_init(void) {
    // For a 50Hz PWM at 1MHz tick (8MHz clock with DIV8):
    // Period = 20ms * 1e6 = 20000 ticks.
    TCA0.SINGLE.PER = 20000;
    // Enable compare channel 0 (WO0 on PA0) in single-slope PWM mode.
    TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP0EN_bm | TCA_SINGLE_WGMODE_SINGLESLOPE_gc;
    // Start with a pulse width of ~1ms (1000 ticks).
    TCA0.SINGLE.CMP0 = 1000;
    // Start TCA0 with DIV8 prescaler.
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV8_gc | TCA_SINGLE_ENABLE_bm;
}

// Uncomment and call this function to manually test PWM movement.
// It cycles the PWM pulse from 1ms to 2ms.
void manual_servo_test(void) {
    while(1) {
        for (uint16_t pwm = 1000; pwm <= 2000; pwm += 100) {
            TCA0.SINGLE.CMP0 = pwm;
            _delay_ms(500);
        }
        for (uint16_t pwm = 2000; pwm >= 1000; pwm -= 100) {
            TCA0.SINGLE.CMP0 = pwm;
            _delay_ms(500);
        }
    }
}