/*
    Ararat Mesrobian
    I2C file for stm32f401
    2/8/2024
*/

#include "stm32f401re.h"

#ifndef __I2C_H
#define __I2C_H

#pragma once

static inline void I2C_init(void){
    //setup I2C pins
    //Set up PB8, Alternate function, I2C1 connected
    uint16_t SCL = PIN('B', 8);
    pinMode(SCL, GPIO_MODE_AF);
    setAF(SCL, 4);
    GPIOB -> OTYPER |= GPIO_OTYPER_OT8; //set to open drain
    //Set up PB8, Alternate function, I2C1 connected
    uint16_t SDA = PIN('B', 9);
    pinMode(SDA, GPIO_MODE_AF);
    setAF(SDA, 4);
    GPIOB -> OTYPER |= GPIO_OTYPER_OT9; //open drain

    //I2C Clock
    RCC -> APB1ENR |= RCC_APB1ENR_I2C1EN;

    //Software reset, 18.6.1 
    I2C1 -> CR1 |= I2C_CR1_SWRST;
    I2C1 -> CR1 &= ~I2C_CR1_SWRST;

    //disable peripheral for adjustments
    I2C1 -> CR1 &= ~I2C_CR1_PE;

    //according to 18.6.8 in RF, Clock control register can only be adjusted while I2C is disabled (see above line)
    I2C1 -> CCR &= ~I2C_CCR_FS; //clear F/S bit to set in SM mode (slow)
    I2C1 -> CCR &= ~I2C_CCR_DUTY; //fm mode duty cycle (will keep low since its not in use)

    //Set own address to 0, 18.6.3, Bit 14 in OAR1 should be kept at 1
    I2C1 -> OAR1 |= 0x0000;
    I2C1 -> OAR1 |= (1 << 14);

    //Timing setup TODO: Figure this out...
    //18.6.8, 100kHz
    I2C1 -> CR2 |= 8;
    I2C1 -> CCR |= 0x28;


    I2C1 -> TRISE |= 0xF;

    I2C1 -> CR1 |= I2C_CR1_START; //set start generation

    //enable peripheral 
    I2C1 -> CR1 |= I2C_CR1_PE;
}


#endif