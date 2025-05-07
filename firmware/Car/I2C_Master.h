/* 
 * File:   I2C_Master.h
 * Author: arunrawat
 *
 * Created on December 9, 2023, 12:33 PM
 */

#ifndef I2C_MASTER_H
#define	I2C_MASTER_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <avr/io.h>

    void I2C_Init (void);
    uint8_t I2C_Write (uint8_t slaveAddr7b, uint8_t *data, uint8_t size);


#ifdef	__cplusplus
}
#endif

#endif	/* I2C_MASTER_H */

