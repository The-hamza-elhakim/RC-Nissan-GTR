
#define F_CPU 5000000
#include <avr/io.h>
#include "I2C_Master.h"

void I2C_Init (void)
{
    TWI0.MCTRLA = TWI_ENABLE_bm;  // Enable TWI
    TWI0.MCTRLB = TWI_ACKACT_NACK_gc;  // send NACK
    TWI0.MSTATUS = 0xFF;  // clear status register
    TWI0.MBAUD = 18;  // 100000 Fclk
    
    PORTMUX.TWIROUTEA = 0x00; // default pins SDA/SCL on PA2/PA3
}

uint8_t I2C_Write (uint8_t slaveAddr7b, uint8_t *data, uint8_t size)
{
    uint8_t writeaddress = (slaveAddr7b << 1) & 0xFE;
    TWI0.MADDR = writeaddress;  //  send slave address with write bit
    while (((TWI0.MSTATUS)&TWI_WIF_bm) == 0);  // wait for the write to complete
    
    if (((TWI0.MSTATUS)& TWI_RXACK_bm) == 0)  // if received ACK from slave
    {
        uint8_t indx =0;
        while ((((TWI0.MSTATUS)&TWI_RXACK_bm) == 0) && (indx < size))  // if recvd ACK and index < data t size
        {
            TWI0.MDATA = data[indx++];
            while (((TWI0.MSTATUS)&TWI_WIF_bm) == 0);  // wait for the write to complete
            
            if (indx == size)
            {
                TWI0.MCTRLB = TWI_MCMD_STOP_gc;  // stop transaction
                return 0;  // success
            }
        }
    }
    
    else 
    {
        TWI0.MCTRLB = TWI_MCMD_STOP_gc;  // stop transaction
        return 1;  // failed
    }
    
    
}
