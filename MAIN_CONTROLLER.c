#define F_CPU 4000000UL
#include <avr/io.h>
#include <util/delay.h>

// === ADC Channel Assignments ===
#define MOTOR_ADC_CHANNEL 1   // PD1
#define SERVO_ADC_CHANNEL 3   // PD3

// === Output Pins (on PORTD)
#define MOTOR_FORWARD_PIN   7
#define MOTOR_BACKWARD_PIN  6
#define SERVO_RIGHT_PIN     5
#define SERVO_LEFT_PIN      4
#define OUT_PORT            PORTD

// === ADC Settings ===
#define DEADZONE_LOW 100
#define DEADZONE_HIGH 150

void adc_init(void) {
    VREF.ADC0REF = VREF_REFSEL_VDD_gc;         // VDD = 5V reference
    ADC0.CTRLC = ADC_PRESC_DIV4_gc;            // ADC Clock = F_CPU / 4
    ADC0.CTRLA = ADC_ENABLE_bm | ADC_RESSEL_12BIT_gc;  // 12-bit, enabled
}

uint16_t read_adc_12bit(uint8_t channel) {
    ADC0.MUXPOS = channel;               // Select channel (1 for PD1, 3 for PD3)
    ADC0.COMMAND = ADC_STCONV_bm;       // Start conversion
    while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));
    ADC0.INTFLAGS = ADC_RESRDY_bm;      // Clear flag
    return ADC0.RES;                    // 12-bit result (0?4095)
}

uint8_t read_joystick(uint8_t channel) {
    uint16_t value = read_adc_12bit(channel);
    return (uint8_t)((value * 255UL) / 4095UL);
}


void init_pins(void) {
    OUT_PORT.DIRSET = (1 << MOTOR_FORWARD_PIN) | (1 << MOTOR_BACKWARD_PIN)
                    | (1 << SERVO_RIGHT_PIN) | (1 << SERVO_LEFT_PIN);
    OUT_PORT.OUTSET = (1 << MOTOR_FORWARD_PIN) | (1 << MOTOR_BACKWARD_PIN)
                    | (1 << SERVO_RIGHT_PIN) | (1 << SERVO_LEFT_PIN);
}

int main(void) {
    adc_init();
    init_pins();
    
    _delay_ms(500);  // Wait for joystick to stabilize
    
    while (1) {
        // === Read and filter joystick values ===
        uint16_t motor_value = read_joystick(MOTOR_ADC_CHANNEL);
        uint16_t servo_value = read_joystick(SERVO_ADC_CHANNEL);
        
        // === Motor Logic ===
        if (motor_value > DEADZONE_HIGH) {
            OUT_PORT.OUTCLR = (1 << MOTOR_FORWARD_PIN);
            OUT_PORT.OUTSET = (1 << MOTOR_BACKWARD_PIN);
        } else if (motor_value < DEADZONE_LOW) {
            OUT_PORT.OUTCLR = (1 << MOTOR_BACKWARD_PIN);
            OUT_PORT.OUTSET = (1 << MOTOR_FORWARD_PIN);
        } else {
            // In deadzone - turn both pins off
            OUT_PORT.OUTSET = (1 << MOTOR_FORWARD_PIN) | (1 << MOTOR_BACKWARD_PIN);
        }
        
        // === Servo Logic ===
        if (servo_value > DEADZONE_HIGH) {
            OUT_PORT.OUTCLR = (1 << SERVO_RIGHT_PIN);
            OUT_PORT.OUTSET = (1 << SERVO_LEFT_PIN);
        } else if (servo_value < DEADZONE_LOW) {
            OUT_PORT.OUTCLR = (1 << SERVO_LEFT_PIN);
            OUT_PORT.OUTSET = (1 << SERVO_RIGHT_PIN);
        } else {
            // In deadzone - turn both pins off
            OUT_PORT.OUTSET = (1 << SERVO_RIGHT_PIN) | (1 << SERVO_LEFT_PIN);
        }
        
        _delay_ms(10);
     
    }
    
}