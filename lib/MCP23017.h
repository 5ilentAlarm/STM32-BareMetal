#ifndef __MCP23017_H
#define __MCP23017_H

#pragma once

#include "stm32f401re.h"

//Definitions
#define MCP23017_DEFAULT_ADDR   0x20 //Denoted as OPCODE in datasheet

//Registers
//Direction
#define MCP23017_REG_IODIRA		0x00
#define MCP23017_REG_IODIRB		0x01

//R/W to GPIO pin
#define MCP23017_REG_GPIOA		0x12
#define MCP23017_REG_GPIOB		0x13

//Other
#define MCP23017_REG_IOCON      0x05





//Functions
void MCP23017_init(uint8_t address);
void MCP23017_SET_POINTER(uint8_t address, uint8_t reg);
void MCP23017_CONFIGURE(uint8_t address, uint8_t config);
void MCP23017_PIN_DIRECTION(uint8_t address, uint8_t port, uint8_t pin);
void MCP23017_PIN_WRITE(uint8_t address, uint8_t port, uint8_t writePins);
uint8_t MCP23017_READ(uint8_t address, uint8_t reg);


//Debugging
//static inline void MCP23017_CHECK_ADDRESS(uint8_t address); //Confirm ACK of address

#endif