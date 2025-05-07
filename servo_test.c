#include <avr/io.h>

int main(void) {
    // === Clock Setup ===
    CCP = CCP_IOREG_gc;
    CLKCTRL.MCLKCTRLB = 0x00; 
    CLKCTRL.OSCHFCTRLA = 0x00; 
    while (!(CLKCTRL.MCLKSTATUS & (1 << 1))); // Wait for OSCHFRDY

    // === Pin Setup ===
    PORTA.DIRSET = PIN5_bm;

    // === Timer Setup ===
    TCB1.CTRLA = TCB_CLKSEL_DIV2_gc;         // 8 MHz
    TCB1.CTRLB = TCB_CNTMODE_INT_gc;         // Manual counting
    TCB1.CCMP = 0xFFFF;

    // === Servo Timing ===
    uint16_t T_high = 2000;          // 1 ms
    uint16_t T_low  = 800000 - T_high ;        // 19 ms (total 160,000 ticks = 20 ms)

    while (1) {
        // HIGH for 1ms
        PORTA.OUTSET = PIN5_bm;
        TCB1.CNT = 0;
        TCB1.CTRLA |= TCB_ENABLE_bm;
        while (TCB1.CNT < T_high);
        TCB1.CTRLA &= ~TCB_ENABLE_bm;

        // LOW for 19ms
        PORTA.OUTCLR = PIN5_bm;
        TCB1.CNT = 0;
        TCB1.CTRLA |= TCB_ENABLE_bm;
        while (TCB1.CNT < T_low);
        TCB1.CTRLA &= ~TCB_ENABLE_bm;
    }
}






/*
int main(void) {
  

    CCP = 0xd8;
    CLKCTRL.OSCHFCTRLA = 0b00010100;
    while( CLKCTRL.MCLKSTATUS & 0b00000001 ){
        ;
    }
    
    TCA0.SINGLE.CTRLA = 0b00001001;
    
    // We will manually check the timer and reset the timer
    // so set the period to its max value to avoid an automatic
    // reset.
    TCA0.SINGLE.PER = 0xffff;

    unsigned int T_high = 100;
    unsigned int T_low = 10000;
    

    PORTA.DIRSET = 0b00100000;

    while (1) {

        PORTA.OUT &= 0b11011111;
        
        while( TCA0.SINGLE.CNT <= T_low) ;        
        TCA0.SINGLE.CNT = 0;

        PORTA.OUT |= 0b00100000;
        
        while( TCA0.SINGLE.CNT <= T_high) ;        
        TCA0.SINGLE.CNT = 0;
        
        
    }
    
}
*/