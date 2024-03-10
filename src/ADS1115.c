#include "lib/ADS1115.h"
#include "stm32f401re.h"
void ADS1115_Init(uint8_t address, uint8_t configaddr, uint8_t msb, uint8_t lsb) {
  I2C1_Master_Send_Address(address);
  //printf("Address sent\r\n");
  I2C1_Master_Send_Data(configaddr);
  //printf("Config sent!\r\n");
  I2C1_Master_Send_Data(msb);
  //printf("Config MSB sent!\r\n");
  I2C1_Master_Send_Data(lsb);
  //printf("Data LSB sent!\r\n");
  I2C1_Master_Stop();
}

void ADS1115_setConvReg(uint8_t address, uint8_t convaddr) {
I2C1_Master_Send_Address(address);
  //printf("Address sent\r\n");
  //delay(1);
  I2C1_Master_Send_Data(convaddr);
  //printf("Address Pointer sent!\r\n");
}

unsigned int ADS1115_getConversion(uint8_t address) {
    I2C1 -> CR1 &= ~I2C_CR1_POS;
    I2C1 -> CR1 |= I2C_CR1_ACK;
    I2C1_Master_Start();
    I2C1 -> DR = (uint8_t)((address << 1) | 0x01); //store slave address and 1 for read
    while(!(I2C1 -> SR1 & I2C_SR1_ADDR)){} //wait until address bit is set(end of address transmission), set after ACK
    (void)I2C1->SR1; //dummy read to clear these flags
    (void)I2C1->SR2;

    unsigned int data,data1, data2; //create data to be read
    while(!(I2C_SR1_RXNE)){};
    data = (uint8_t)I2C1->DR;
    spin(200);
    I2C1 -> CR1 |= I2C_CR1_ACK;
    data1 = (uint8_t)I2C1->DR; //this is right
    spin(200);
    data2 = (uint8_t)I2C1->DR;//thisis right
    spin(200);
    I2C1_Master_Stop();
    (void)data;
    data = (uint8_t)I2C1->DR;
    spin(200);

    //Convert two bytes in a 16-bit in, add in adjustment value. 
    unsigned int returnVal = 0;
    returnVal = (data1 << 8);
    returnVal = (returnVal | data2) + 3350; //TODO: Figure how to get better readings

    if(returnVal > 40000){ //When is pin is grounded or left floating, readings are at double Vref for some reason. ADC output can't be > Vref.
         return 0;  //TODO:Figure this issue out
     }
    return returnVal;
}

unsigned int ADS1115_getSamples(uint8_t address, uint8_t sampleSize) {
    unsigned int sum = 0;
    for(int i = 0; i < sampleSize; i++) {
        sum += ADS1115_getConversion(address);
    }

    return sum / sampleSize;
}