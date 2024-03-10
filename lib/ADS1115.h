/*
    Ararat Mesrobian 
    ADS1115 Address and Register definitions
*/
#include "stm32f401re.h"

#ifndef __ADS1115_H
#define __ADS1115_H

#pragma once

//Address
#define ADS1115_DEFAULT_ADDR 0x48 //if addr is tied to ground
#define ADS1115_VDD_ADDR 0x49 //if addr is tied to ground
#define ADS1115_SDA_ADDR 0x4A //if addr is tied to ground
#define ADS1115_SCL_ADDR 0x4B //if addr is tied to ground

//ADS1115 Registerse
#define ADS1115_REG_CONVERSION 0x00
#define ADS1115_REG_CONFIG 0x01

//Data
#define ADS1115_DEFAULT_CONFIG_MSB 0x84 /*Based on the datasheet, 10.1.7 */
#define ADS1115_DEFAULT_CONFIG_LSB 0x83

//functions
void ADS1115_Init(uint8_t address, uint8_t configaddr, uint8_t msb, uint8_t lsb); //Initiliazes the ads1115 with configuration (msb, lsb)s
void ADS1115_setConvReg(uint8_t address, uint8_t convaddr); //sets the ads1152 to the conversion register address pointer
unsigned int ADS1115_getConversion(uint8_t address);
unsigned int ADS1115_getSamples(uint8_t address, uint8_t sampleSize);


#endif