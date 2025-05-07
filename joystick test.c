#define F_CPU 4000000UL
#include <avr/io.h>
#include <util/delay.h>

// === Input Pins ===
#define MOTOR_JOYSTICK_CHANNEL 1   // PD1 = ADC1 (Y-axis)
#define SERVO_JOYSTICK_CHANNEL 3   // PD3 = ADC3 (X-axis)
#define SERVO_BUTTON_PIN 2         // PD2 (digital input)

// === Output Pins (on PORTA)
#define CON_SERVO_RIGHT_PIN 1
#define CON_SERVO_LEFT_PIN  2
#define CON_MOTOR_FORWARD_PIN 3
#define CON_MOTOR_BACKWARD_PIN 4
#define CON_PORT PORTA

void adc_init(void) {
    VREF.ADC0REF = VREF_REFSEL_VDD_gc;  // Use VDD as reference
    ADC0.CTRLC = ADC_PRESC_DIV4_gc;     // ADC clock prescaler
    ADC0.CTRLA = ADC_ENABLE_bm;         // Enable ADC
}

uint16_t adc_read(uint8_t channel) {
    ADC0.MUXPOS = channel;
    ADC0.COMMAND = ADC_STCONV_bm;
    while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));
    return ADC0.RES;
}

void init_pins(void) {
    // Set button as input with pull-up
    PORTD.DIRCLR = (1 << SERVO_BUTTON_PIN);
    PORTD.PIN2CTRL = PORT_PULLUPEN_bm;

    // Set all output pins on PORTA
    CON_PORT.DIRSET = (1 << CON_MOTOR_FORWARD_PIN) | (1 << CON_MOTOR_BACKWARD_PIN)
                    | (1 << CON_SERVO_LEFT_PIN) | (1 << CON_SERVO_RIGHT_PIN);
}

int main(void) {
    adc_init();
    init_pins();

    while (1) {
        // === Read Inputs ===
        uint16_t motor_joy = adc_read(MOTOR_JOYSTICK_CHANNEL);   // 0?1023
        uint16_t servo_joy = adc_read(SERVO_JOYSTICK_CHANNEL);   // 0?1023
        uint8_t servo_button = !(PORTD.IN & (1 << SERVO_BUTTON_PIN));  // Active LOW

        // === Motor Logic ===
        if (motor_joy > 700) {
            CON_PORT.OUTSET = (1 << CON_MOTOR_FORWARD_PIN);
            CON_PORT.OUTCLR = (1 << CON_MOTOR_BACKWARD_PIN);
        } else if (motor_joy < 300) {
            CON_PORT.OUTSET = (1 << CON_MOTOR_BACKWARD_PIN);
            CON_PORT.OUTCLR = (1 << CON_MOTOR_FORWARD_PIN);
        } else {
            // Neutral zone: stop
            CON_PORT.OUTCLR = (1 << CON_MOTOR_FORWARD_PIN) | (1 << CON_MOTOR_BACKWARD_PIN);
        }

        // === Servo Logic (only when button held)
        if (servo_button) {
            if (servo_joy > 700) {
                CON_PORT.OUTSET = (1 << CON_SERVO_RIGHT_PIN);
                CON_PORT.OUTCLR = (1 << CON_SERVO_LEFT_PIN);
            } else if (servo_joy < 300) {
                CON_PORT.OUTSET = (1 << CON_SERVO_LEFT_PIN);
                CON_PORT.OUTCLR = (1 << CON_SERVO_RIGHT_PIN);
            } else {
                CON_PORT.OUTCLR = (1 << CON_SERVO_LEFT_PIN) | (1 << CON_SERVO_RIGHT_PIN);
            }
        } else {
            // No input when button is not pressed
            CON_PORT.OUTCLR = (1 << CON_SERVO_LEFT_PIN) | (1 << CON_SERVO_RIGHT_PIN);
        }

        _delay_ms(10);  // Optional: light delay to reduce jitter
    }
}
