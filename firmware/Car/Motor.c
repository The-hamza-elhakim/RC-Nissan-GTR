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

#define JOYSTICK_PIN 2   // PD2
#define JOYSTICK_PORT PORTD

#define TM1637_BRIGHTNESS 0x0F
#define TM1637_CMD_SET_DATA 0x40
#define TM1637_CMD_SET_ADDR 0xC0
#define TM1637_CMD_DISPLAY_CTRL 0x88

#define FILTER_SIZE 5
#define DEADZONE_LOW 115
#define DEADZONE_HIGH 132

// === Globals ===
volatile uint16_t echo_end = 0;
volatile uint8_t echo_done = 0;
uint16_t distance_buffer[FILTER_SIZE] = {0};
uint8_t buffer_index = 0;
uint8_t buffer_filled = 0;

const uint8_t digitToSegment[10] = {
  0x3F, 0x06, 0x5B, 0x4F,
  0x66, 0x6D, 0x7D, 0x07,
  0x7F, 0x6F
};

// === Display Functions ===
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

// === Joystick & Motor ===
void adc_init(void) {
  VREF.ADC0REF = 0b10000101;
  ADC0.MUXPOS = JOYSTICK_PIN;
  ADC0.CTRLC = 0x00;
  ADC0.CTRLA = 0b00000011;
  ADC0.COMMAND = 0x01;
  JOYSTICK_PORT.DIRCLR = (1 << JOYSTICK_PIN);
}

uint16_t readJoystick(void) {
  while (!(ADC0.INTFLAGS & 0x01));
  uint16_t value = ADC0.RES;
  ADC0.INTFLAGS = 0x01;
  return (uint8_t)((value * 255UL) / 4095UL);
}

void init_pwm(void) {
  PORTMUX.TCAROUTEA = PORTMUX_TCA0_PORTA_gc;
  TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV64_gc | TCA_SINGLE_ENABLE_bm;
  TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_SINGLESLOPE_gc | TCA_SINGLE_CMP0EN_bm;
  TCA0.SINGLE.PER = 255;
  TCA0.SINGLE.CMP0 = 0;
}

void controlMotor(uint16_t joystickADC) {
  uint8_t duty = 0;
  if (joystickADC < DEADZONE_LOW) {
    MOTOR_PORT.OUTSET = (1 << MOTOR_IN1_PIN);
    MOTOR_PORT.OUTCLR = (1 << MOTOR_IN2_PIN);
    duty = (DEADZONE_LOW - joystickADC) * 255 / DEADZONE_LOW;
  } else if (joystickADC > DEADZONE_HIGH) {
    MOTOR_PORT.OUTCLR = (1 << MOTOR_IN1_PIN);
    MOTOR_PORT.OUTSET = (1 << MOTOR_IN2_PIN);
    duty = (joystickADC - DEADZONE_HIGH) * 255 / (255 - DEADZONE_HIGH);
  } else {
    MOTOR_PORT.OUTCLR = (1 << MOTOR_IN1_PIN) | (1 << MOTOR_IN2_PIN);
    duty = 0;
  }
  TCA0.SINGLE.CMP0 = duty;
}

// === Sensor ===
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
  return echo_end / 117; // adjust based on 4 MHz clock
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

// === Setup Pins ===
void init_pins(void) {
  MOTOR_PORT.DIRSET = (1 << MOTOR_ENA_PIN) | (1 << MOTOR_IN1_PIN) | (1 << MOTOR_IN2_PIN);
  PORTD.DIRSET = (1 << TM1637_CLK_PIN) | (1 << TM1637_DIO_PIN) | (1 << HC_SR04_TRIG_PIN);
  PORTD.DIRCLR = (1 << HC_SR04_ECHO_PIN);
  PORTD.PIN4CTRL = PORT_PULLUPEN_bm | PORT_ISC_BOTHEDGES_gc;
}

// === Main ===
int main(void) {
  CLKCTRL.MCLKCTRLA = CLKCTRL_CLKSEL_OSCHF_gc;
  CLKCTRL.MCLKCTRLB = CLKCTRL_PDIV_4X_gc | CLKCTRL_PEN_bm;

  init_pins();
  adc_init();
  init_pwm();
  timer_init();
  tm1637_init();
  sei();

  while (1) {
    uint8_t joy = readJoystick();
    controlMotor(joy);

    trigger_measurement();
    while (!echo_done);
    uint16_t distance = calculate_distance();
    distance = apply_moving_average(distance);
    echo_done = 0;

    display_distance(distance);
    _delay_ms(100);
  }
}