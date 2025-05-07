#define F_CPU 4000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>


#define LED_PORT PORTD
#define LED_PIN 5 //PD5


int main(void) {
    
    // Set PD5 as output
    LED_PORT.DIRSET = (1 << LED_PIN);
    
    LED_PORT.OUTCLR = (1 << LED_PIN);
    
    while (1) {
        
    }
}
