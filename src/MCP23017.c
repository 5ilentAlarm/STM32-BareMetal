#include "lib/MCP23017.h"

void MCP23017_init(uint8_t address) {
    //init MCP23017
    I2C1_Master_Send_Address(address); //"OPCODE"
    I2C1_Master_Stop();
    //printf("Device acknowledged!\r\n");
}

void MCP23017_CONFIGURE(uint8_t address, uint8_t config) {
    I2C1_Master_Send_Address(address); //"OPCODE"
    I2C1_Master_Send_Data(MCP23017_REG_IOCON); //Access config register
    I2C1_Master_Send_Data(config); //Turn off sequential increment
    I2C1_Master_Stop();
    //printf("Configured!\r\n");
}

void MCP23017_PIN_DIRECTION(uint8_t address, uint8_t port, uint8_t setPins) {
    //Set IODIRA as outputs(all zeroes)
    I2C1_Master_Send_Address(address); //"OPCODE"
    I2C1_Master_Send_Data(port); //Access port
    I2C1_Master_Send_Data(setPins); //Set direction
    I2C1_Master_Stop();
    //printf("Direction Set!\r\n");
}

void MCP23017_PIN_WRITE(uint8_t address, uint8_t gpioPort, uint8_t writePins) {
    //Set IO0 as 1
    I2C1_Master_Send_Address(address); //"OPCODE"
    I2C1_Master_Send_Data(gpioPort); //Access GPIOA
    I2C1_Master_Send_Data(writePins); //TODO: Read current value of register, save, then add the pin that is wanting to be set, same goes for direction
    I2C1_Master_Stop();
    //printf("IO are set!\r\n");
}

void MCP23017_SET_POINTER(uint8_t address, uint8_t reg) {
    I2C1_Master_Send_Address(address); //"OPCODE"
    I2C1_Master_Send_Data(reg); //Write to address pointer
    //I2C1_Master_Stop();
}

uint8_t MCP23017_READ(uint8_t address, uint8_t reg) {
    //Point to correct register
    MCP23017_SET_POINTER(address, reg);

    //Now read data
    I2C1 -> CR1 &= ~I2C_CR1_POS;
    I2C1 -> CR1 |= I2C_CR1_ACK;
    I2C1_Master_Start();
    I2C1 -> DR = (uint8_t)(address << 1) | 0x01; //store slave address and 1 for read
    while(!(I2C1 -> SR1 & I2C_SR1_ADDR)){} //wait until address bit is set(end of address transmission), set after ACK
    (void)I2C1->SR1; //dummy read to clear these flags
    (void)I2C1->SR2;

    //int data;
    uint8_t data1, data2;
    while(!(I2C1 -> SR1 & I2C_SR1_RXNE)){};
    //data = (uint8_t)I2C1->DR;
    I2C1 -> CR1 |= I2C_CR1_ACK;
    data1 = (uint8_t)I2C1->DR;
    spin(200);
    data2 = (uint8_t)I2C1->DR;
    spin(200);
    (void)data2;
    (void)data1;
    //printf("Here 4, %x is data 1, %x is data2\r\n", data1, data2);
    //data = (uint8_t)I2C1->DR;
    spin(200);

    I2C1_Master_FixBusy();
    
    I2C1_Master_Stop();
    return data1;
}