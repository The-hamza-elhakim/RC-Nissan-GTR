#define F_CPU 4000000UL

#include <avr/io.h>
#include <avr/interrupt.h>

// === CONFIG ===
#define METAL_INPUT_PIN   PIN6_bm   // PA6 input from metal detector
#define BUZZER_OUTPUT_PIN PIN2_bm   // PD2 output to buzzer

// Use CLKDIV64: 4 MHz / 64 = 62.5 kHz ? 62,500 ticks = 1 second
#define COUNT_WINDOW_TICKS 62500UL

#define THRESHOLD_DIFF      1         // Detect significant frequency change

// === GLOBALS ===
volatile uint16_t edge_count = 0;
volatile uint16_t baseline_count = 0;
volatile uint8_t  counting_enabled = 1;
volatile uint8_t  baseline_set = 0;
volatile uint8_t  startup_counter = 0;


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

// === PORT SETUP ===
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

// === TIMER SETUP (TCB0) ===
void TCB0_init(void) {
    TCB0.CNT = 0;
    TCB0.CCMP = COUNT_WINDOW_TICKS;
    TCB0.CTRLA = TCB_CLKSEL_CLKDIV64_gc | TCB_ENABLE_bm;
    TCB0.CTRLB = 0;
    TCB0.INTCTRL = TCB_INT_OVF_bm;  // Enable overflow interrupt
}

// === EDGE COUNT ISR ===
ISR(PORTA_PORT_vect) {
    if ((PORTA.INTFLAGS & METAL_INPUT_PIN) && counting_enabled) {
        PORTA.INTFLAGS = METAL_INPUT_PIN;  // Clear interrupt flag
        edge_count++;
    }
}

// === 1s TIMER WINDOW ISR ===
ISR(TCB0_INT_vect) {
    TCB0.INTFLAGS = TCB_INT_OVF_bm;  // Clear interrupt
    counting_enabled = 0;

    // First 5 seconds = establish baseline
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

    // Detection logic: frequency has increased
    if (edge_count >= baseline_count + THRESHOLD_DIFF) {
        PORTD.OUTSET = BUZZER_OUTPUT_PIN;  // Metal detected ? buzzer ON
    } else if (edge_count <= baseline_count) {
        PORTD.OUTCLR = BUZZER_OUTPUT_PIN;  // No detection ? buzzer OFF
    }

    edge_count = 0;
    counting_enabled = 1;
}

// === MAIN LOOP ===
int main(void) {
    PORT_init();
    TCB0_init();
    sei();  // Enable global interrupts

    while (1) {
        // All logic handled in ISRs
    }
}
