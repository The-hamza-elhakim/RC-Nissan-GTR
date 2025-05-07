#define F_CPU 4000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>

// === Pin Definitions ===
#define TM1637_CLK_PIN 7 // PD7
#define TM1637_DIO_PIN 6 // PD6
#define TM1637_PORT PORTD

#define HC_SR04_TRIG_PIN 1 // PD1
#define HC_SR04_ECHO_PIN 4 // PD4

#define MOTOR_ENA_PIN 0  // PA0 (PWM)
#define MOTOR_IN1_PIN 1  // PA1
#define MOTOR_IN2_PIN 2  // PA2
#define MOTOR_PORT PORTA

#define RF_FORWARD_MOTOR_PIN 0   // PC0
#define RF_BACKWARDS_MOTOR_PIN 1 // PC1
#define RF_PORT PORTC

#define SERVO_RIGHT_PIN 3 // PA3
#define SERVO_LEFT_PIN 4 // PA4
#define SERVO_PWM_PIN 5 // PA5
#define SERVO_PORT PORTA


#define TM1637_BRIGHTNESS 0x0F
#define TM1637_CMD_SET_DATA 0x40
#define TM1637_CMD_SET_ADDR 0xC0
#define TM1637_CMD_DISPLAY_CTRL 0x88

#define METAL_INPUT_PIN   PIN6_bm   // PA6 input from metal detector
#define BUZZER_OUTPUT_PIN PIN2_bm   // PD2 output to buzzer
#define THRESHOLD_DIFF      1

#define SERVO_PWM_PIN 5      // PA5

// For 4MHz system clock
#define TIMER_PRESCALER      2       // Clock divider
#define TIMER_FREQ           (F_CPU / TIMER_PRESCALER)  // 2 MHz
#define MS_TO_TICKS(ms)      ((uint16_t)((ms) * (TIMER_FREQ / 1000)))  // Convert ms to ticks

// Servo pulse parameters (in timer ticks)
#define SERVO_PERIOD         MS_TO_TICKS(20)    // 20ms period (50Hz)
#define SERVO_MIN_PULSE      MS_TO_TICKS(0.85)     // 1ms pulse (0 degrees)
#define SERVO_MID_PULSE      MS_TO_TICKS(1.53)   // 1.5ms pulse (90 degrees)
#define SERVO_MAX_PULSE      MS_TO_TICKS(2.35)     // 2ms pulse (180 degrees)

#define FILTER_SIZE 5

#define COUNT_WINDOW_TICKS 62500UL

// ===== DEFINITIONS ===== //

#ifndef TCB_ENABLE_bm
#define TCB_ENABLE_bm     (1 << 0)
#endif

#ifndef TCB_CLKSEL_CLKDIV64_gc
#define TCB_CLKSEL_CLKDIV64_gc  (0x03 << 1)
#endif

#ifndef TCB_INT_OVF_bm
#define TCB_INT_OVF_bm    (1 << 0)
#endif

typedef enum {
    SERVO_PULSE_HIGH,
    SERVO_PULSE_LOW
} servo_state_t;

typedef enum {
    MOTOR_FORWARD,
    MOTOR_REVERSE
} motor_dir_t;

volatile uint16_t edge_count = 0;
volatile uint16_t baseline_count = 0;
volatile uint8_t  counting_enabled = 1;
volatile uint8_t  baseline_set = 0;
volatile uint8_t  startup_counter = 0;
volatile uint16_t echo_end = 0;
volatile uint8_t echo_done = 0;
uint16_t distance_buffer[FILTER_SIZE] = {0};
uint8_t buffer_index = 0;
uint8_t buffer_filled = 0;

volatile bool motor_override = false;
volatile bool motor_direction = true;

volatile servo_state_t servo_state = SERVO_PULSE_HIGH;
volatile uint16_t servo_pulse_width = SERVO_MID_PULSE;

const uint8_t digitToSegment[10] = {
  0x3F, 0x06, 0x5B, 0x4F,
  0x66, 0x6D, 0x7D, 0x07,
  0x7F, 0x6F
};

// ===== Display Functions ===== //

void tm1637_start(void);
void tm1637_stop(void);
void tm1637_write_byte(uint8_t b);
void tm1637_display(uint8_t segments[]);
void display_distance(uint16_t value);

void tm1637_init(void) {
  _delay_ms(50);
  tm1637_start();
  tm1637_write_byte(TM1637_CMD_SET_DATA);
  tm1637_stop();
  tm1637_start();
  tm1637_write_byte(TM1637_CMD_DISPLAY_CTRL | (TM1637_BRIGHTNESS & 0x07));
  tm1637_stop();
}

void display_distance(uint16_t value) {
  uint8_t digits[4] = {
    (value / 1000) % 10,
    (value / 100) % 10,
    (value / 10) % 10,
    value % 10
  };

  uint8_t segments[4];
  for (uint8_t i = 0; i < 4; i++) {
    segments[i] = digitToSegment[digits[i]];
  }

  if (value < 1000) segments[0] = 0;
  if (value < 100) segments[1] = 0;
  if (value < 10)  segments[2] = 0;

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
  TM1637_PORT.OUTSET = (1 << TM1637_DIO_PIN);
  TM1637_PORT.OUTSET = (1 << TM1637_CLK_PIN);
  _delay_us(2);
  TM1637_PORT.OUTCLR = (1 << TM1637_DIO_PIN);
  _delay_us(2);
  TM1637_PORT.OUTCLR = (1 << TM1637_CLK_PIN);
  _delay_us(2);
}

void tm1637_stop(void) {
  TM1637_PORT.OUTCLR = (1 << TM1637_CLK_PIN);
  _delay_us(2);
  TM1637_PORT.OUTCLR = (1 << TM1637_DIO_PIN);
  _delay_us(2);
  TM1637_PORT.OUTSET = (1 << TM1637_CLK_PIN);
  _delay_us(2);
  TM1637_PORT.OUTSET = (1 << TM1637_DIO_PIN);
  _delay_us(2);
}

void tm1637_write_byte(uint8_t b) {
  for (uint8_t i = 0; i < 8; i++) {
    TM1637_PORT.OUTCLR = (1 << TM1637_CLK_PIN);
    _delay_us(2);
    if (b & 0x01)
      TM1637_PORT.OUTSET = (1 << TM1637_DIO_PIN);
    else
      TM1637_PORT.OUTCLR = (1 << TM1637_DIO_PIN);
    _delay_us(2);
    TM1637_PORT.OUTSET = (1 << TM1637_CLK_PIN);
    _delay_us(2);
    b >>= 1;
  }

  TM1637_PORT.OUTCLR = (1 << TM1637_CLK_PIN);
  TM1637_PORT.DIRCLR = (1 << TM1637_DIO_PIN);
  _delay_us(5);
  TM1637_PORT.OUTSET = (1 << TM1637_CLK_PIN);
  _delay_us(2);
  TM1637_PORT.OUTCLR = (1 << TM1637_CLK_PIN);
  TM1637_PORT.DIRSET = (1 << TM1637_DIO_PIN);
  _delay_us(2);
}

// ===== Motor & Input Handling ===== //

void init_pwm(void) {
  PORTMUX.TCAROUTEA = PORTMUX_TCA0_PORTA_gc;
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV64_gc | TCA_SINGLE_ENABLE_bm;
  TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_SINGLESLOPE_gc | TCA_SINGLE_CMP0EN_bm;
  TCA0.SINGLE.PER = 255;
  TCA0.SINGLE.CMP0 = 0;
}

void rf_input_init(void) {
  // Set PA3, PA4 as inputs
  PORTC.DIRCLR = (1 << RF_FORWARD_MOTOR_PIN) | (1 << RF_BACKWARDS_MOTOR_PIN);

  // Enable internal pull-ups
  PORTC.PIN3CTRL = PORT_PULLUPEN_bm;
  PORTC.PIN4CTRL = PORT_PULLUPEN_bm;
  PORTC.PIN5CTRL = PORT_PULLUPEN_bm;
  PORTC.PIN6CTRL = PORT_PULLUPEN_bm;
  
  // Set PA5, PA6 as inputs
  PORTA.DIRCLR = (1 << SERVO_RIGHT_PIN) | (1 << SERVO_LEFT_PIN);
  
  // Enable internal pull-ups
  PORTA.PIN3CTRL = PORT_PULLUPEN_bm;
  PORTA.PIN4CTRL = PORT_PULLUPEN_bm;
}

motor_dir_t last_motor_dir = MOTOR_FORWARD;

void controlMotor(void) {
  
  if (motor_override) return;  
  
  if (PORTC.IN & (1 << RF_FORWARD_MOTOR_PIN)) {
    MOTOR_PORT.OUTCLR = (1 << MOTOR_IN1_PIN);
    MOTOR_PORT.OUTSET = (1 << MOTOR_IN2_PIN);
    TCA0.SINGLE.CMP0 = 255;
    last_motor_dir = MOTOR_FORWARD;
    
  } else if (PORTC.IN & (1 << RF_BACKWARDS_MOTOR_PIN)) {
    MOTOR_PORT.OUTSET = (1 << MOTOR_IN1_PIN);
    MOTOR_PORT.OUTCLR = (1 << MOTOR_IN2_PIN);
    TCA0.SINGLE.CMP0 = 255;
    last_motor_dir = MOTOR_REVERSE;
    
  } else {
    MOTOR_PORT.OUTCLR = (1 << MOTOR_IN1_PIN) | (1 << MOTOR_IN2_PIN);
    TCA0.SINGLE.CMP0 = 0;
  }
}

void drive_motor_in_last_direction(void) {
    if (last_motor_dir == MOTOR_FORWARD) {
        MOTOR_PORT.OUTCLR = (1 << MOTOR_IN1_PIN);
        MOTOR_PORT.OUTSET = (1 << MOTOR_IN2_PIN);
        TCA0.SINGLE.CMP0 = 255;
    } else if (last_motor_dir == MOTOR_REVERSE) {
        MOTOR_PORT.OUTSET = (1 << MOTOR_IN1_PIN);
        MOTOR_PORT.OUTCLR = (1 << MOTOR_IN2_PIN);
        TCA0.SINGLE.CMP0 = 255;
    } else {
        // Stay off if direction is unknown
        TCA0.SINGLE.CMP0 = 0;
    }
}

// ===== METAL DETECTOR ===== //

void PORT_init(void) {
    // PA6: input from 555 timer
    PORTA.DIRCLR = METAL_INPUT_PIN;
    PORTA.PIN6CTRL = PORT_ISC_RISING_gc;  // Trigger ISR on rising edge

    // Optional: pull-down resistor for stability
    // PORTA.PIN6CTRL |= PORT_PULLUPEN_bm;

    // PD2: output to buzzer
    PORTD.DIRSET = BUZZER_OUTPUT_PIN;
    PORTD.OUTCLR = BUZZER_OUTPUT_PIN;
}

void TCB2_init(void) {
    TCB2.CNT = 0;
    TCB2.CCMP = COUNT_WINDOW_TICKS;
    TCB2.CTRLA = TCB_CLKSEL_CLKDIV64_gc | TCB_ENABLE_bm;
    TCB2.CTRLB = 0;
    TCB2.INTCTRL = TCB_INT_OVF_bm;
}



ISR(PORTA_PORT_vect) {
    if ((PORTA.INTFLAGS & METAL_INPUT_PIN) && counting_enabled) {
        PORTA.INTFLAGS = METAL_INPUT_PIN;
        edge_count++;
    }
}


ISR(TCB2_INT_vect) {
    TCB2.INTFLAGS = TCB_INT_OVF_bm;
    counting_enabled = 0;

    if (!baseline_set) {
        startup_counter++;
        if (startup_counter >= 5) {
            baseline_count = edge_count;
            baseline_set = 1;
        }
        edge_count = 0;
        counting_enabled = 1;
        return;
    }

    if (edge_count >= baseline_count + THRESHOLD_DIFF) {
        PORTD.OUTSET = BUZZER_OUTPUT_PIN;
    } else if (edge_count <= baseline_count) {
        PORTD.OUTCLR = BUZZER_OUTPUT_PIN;
    }

    edge_count = 0;
    counting_enabled = 1;
}


// === Servo Input & Handling === //

// === Initialize Servo ===
void init_servo(void) {
    // Set up output pin
    PORTA.DIRSET = (1 << SERVO_PWM_PIN);
    PORTA.OUTCLR = (1 << SERVO_PWM_PIN);
    
    // Configure TCB1 for compare interrupt mode
    TCB1.CTRLA = TCB_CLKSEL_DIV2_gc;       // Use prescaled clock (2MHz)
    
    // Since we don't have a DIV64 option directly, we'll adjust our calculations
    #undef TIMER_PRESCALER
    #define TIMER_PRESCALER 2       // Actual prescaler value
    TCB1.CTRLB = TCB_CNTMODE_INT_gc;       // Periodic interrupt mode
    TCB1.CCMP = servo_pulse_width;         // Start with mid pulse
    TCB1.INTCTRL = TCB_CAPT_bm;            // Enable capture interrupt
    
    // Clear interrupt flag and enable timer
    TCB1.INTFLAGS = TCB_CAPT_bm;
    TCB1.CTRLA |= TCB_ENABLE_bm;
}

// Set servo position (0-180 degrees)
void set_servo_position(uint8_t degrees) {
    // Constrain input to 0-180 degrees
    if (degrees > 180) degrees = 180;
    
    // Map degrees to pulse width
    if (degrees == 0) {
        servo_pulse_width = SERVO_MIN_PULSE;
    } else if (degrees == 90) {
        servo_pulse_width = SERVO_MID_PULSE;
    } else if (degrees == 180) {
        servo_pulse_width = SERVO_MAX_PULSE;
    } else if (degrees < 90) {
        // Interpolate between 0 and 90 degrees
        servo_pulse_width = SERVO_MIN_PULSE + 
            ((uint32_t)(degrees) * (SERVO_MID_PULSE - SERVO_MIN_PULSE)) / 90;
    } else {
        // Interpolate between 90 and 180 degrees
        servo_pulse_width = SERVO_MID_PULSE + 
            ((uint32_t)(degrees - 90) * (SERVO_MAX_PULSE - SERVO_MID_PULSE)) / 90;
    }
}

// Update servo from input pins
void update_servo_from_pins(void) {
    if (PORTA.IN & (1 << SERVO_RIGHT_PIN)) {
        set_servo_position(0);     // Right position
        drive_motor_in_last_direction();
        motor_override = true;
        
    } else if (PORTA.IN & (1 << SERVO_LEFT_PIN)) {
        set_servo_position(180);   // Left position
        drive_motor_in_last_direction();
        motor_override = true;
        
    } else {
        set_servo_position(90);    // Center position
        motor_override = false;
    }
}


// Interrupt handler for servo timing
ISR(TCB1_INT_vect) {
    // Clear interrupt flag
    TCB1.INTFLAGS = TCB_CAPT_bm;
    
    switch (servo_state) {
        case SERVO_PULSE_HIGH:
            // End of HIGH pulse
            PORTA.OUTCLR = (1 << SERVO_PWM_PIN);    // Set pin LOW
            TCB1.CCMP = SERVO_PERIOD - servo_pulse_width;  // Set time for LOW period
            servo_state = SERVO_PULSE_LOW;
            break;
            
        case SERVO_PULSE_LOW:
            // End of LOW pulse (end of cycle)
            PORTA.OUTSET = (1 << SERVO_PWM_PIN);    // Set pin HIGH
            TCB1.CCMP = servo_pulse_width;          // Set time for next HIGH period
            servo_state = SERVO_PULSE_HIGH;
            break;
    }
}


// ===== SENSOR HELPERS ===== //

ISR(TCB0_INT_vect) {
  TCB0.INTFLAGS = TCB_OVF_bm;
  echo_done = 1;
}

ISR(PORTD_PORT_vect) {
  if (PORTD.INTFLAGS & (1 << HC_SR04_ECHO_PIN)) {
    if (PORTD.IN & (1 << HC_SR04_ECHO_PIN)) {
      TCB0.CNT = 0;
      TCB0.CTRLA |= TCB_ENABLE_bm;
    } else {
      echo_end = TCB0.CNT;
      TCB0.CTRLA &= ~TCB_ENABLE_bm;
      echo_done = 1;
    }
    PORTD.INTFLAGS = (1 << HC_SR04_ECHO_PIN);
  }
}


void timer_init(void) {
  TCB0.CTRLA = TCB_CLKSEL_DIV2_gc;
  TCB0.CTRLB = TCB_CNTMODE_INT_gc;
  TCB0.CCMP = 60000;
  TCB0.INTCTRL = TCB_OVF_bm;
}

void trigger_measurement(void) {
  TCB0.CTRLA &= ~TCB_ENABLE_bm;
  TCB0.INTFLAGS = TCB_OVF_bm;
  PORTD.INTFLAGS = (1 << HC_SR04_ECHO_PIN);
  PORTD.OUTSET = (1 << HC_SR04_TRIG_PIN);
  _delay_us(10);
  PORTD.OUTCLR = (1 << HC_SR04_TRIG_PIN);
}

uint16_t calculate_distance(void) {
  if (echo_end == 0) return 0;
  return echo_end / 117;
}

uint16_t apply_moving_average(uint16_t new_distance) {
  distance_buffer[buffer_index] = new_distance;
  buffer_index = (buffer_index + 1) % FILTER_SIZE;
  if (buffer_index == 0) buffer_filled = 1;
  uint32_t sum = 0;
  uint8_t count = buffer_filled ? FILTER_SIZE : buffer_index;
  for (uint8_t i = 0; i < count; i++) sum += distance_buffer[i];
  return sum / (count ? count : 1);
}

void init_pins(void) {
  MOTOR_PORT.DIRSET = (1 << MOTOR_ENA_PIN) | (1 << MOTOR_IN1_PIN) | (1 << MOTOR_IN2_PIN);
  PORTD.DIRSET = (1 << TM1637_CLK_PIN) | (1 << TM1637_DIO_PIN) | (1 << HC_SR04_TRIG_PIN);
  PORTD.DIRCLR = (1 << HC_SR04_ECHO_PIN);
  PORTD.PIN4CTRL = PORT_PULLUPEN_bm | PORT_ISC_BOTHEDGES_gc;
  PORTD.DIRSET = (1 << 5) | (1 << 3);  // Set PD5 and PD3 as output

}

// ===== LED's ===== //

void update_motor_leds(void) {
    if (last_motor_dir == MOTOR_FORWARD) {
        PORTD.OUTSET = (1 << 5);
        PORTD.OUTCLR = (1 << 3);
    } else if (last_motor_dir == MOTOR_REVERSE) {
        PORTD.OUTSET = (1 << 3);
        PORTD.OUTCLR = (1 << 5);
    }
}


// ===== Main ===== //
int main(void) {
  CLKCTRL.MCLKCTRLA = CLKCTRL_CLKSEL_OSCHF_gc;
  CLKCTRL.MCLKCTRLB = CLKCTRL_PDIV_4X_gc | CLKCTRL_PEN_bm;

  init_pins();
  rf_input_init();
  init_pwm();
  timer_init();
  tm1637_init();
  init_servo();
  PORT_init();
  TCB2_init();

    
  sei();
  
  while (1) {
      
    // ===== MOTOR AND SERVO ===== //
    controlMotor();
    update_servo_from_pins();
    
    // ===== LED ===== //
    update_motor_leds();

    // ===== DISTANCE ===== //
    
    trigger_measurement();
    while (!echo_done);
    uint16_t distance = calculate_distance();
    distance = apply_moving_average(distance);
    echo_done = 0;
    
    if (distance > 200) distance = 200;   // cap at 200cm

    display_distance(distance);
    _delay_ms(100);
  }
}
