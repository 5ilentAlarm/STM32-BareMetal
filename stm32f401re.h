/*
    Header file to go with STM32f401 functions
    Based off bare metal programming guide on Github
*/
#pragma once

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>

#include "stm32f401xe.h"

#define FREQ 16000000  // CPU frequency, 16 Mhz
#define BIT(x) (1UL << (x))
#define PIN(bank, num) ((((bank) - 'A') << 8) | (num))
#define PINNO(pin) (pin & 255)
#define PINBANK(pin) (pin >> 8)

static inline void spin(volatile uint32_t count) {
  while (count--) asm("nop");
}

#define GPIO(bank) ((GPIO_TypeDef *) (GPIOA_BASE + 0x400U * (bank)))
enum { GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG };

static inline void pinMode(uint16_t pin, uint8_t mode) {
  GPIO_TypeDef *gpio = GPIO(PINBANK(pin));  // GPIO bank
  int n = PINNO(pin);                       // Pin number
  RCC->AHB1ENR |= BIT(PINBANK(pin));        // Enable GPIO clock
  spin(500);
  gpio->MODER &= ~(3U << (n * 2));          // Clear existing setting
  gpio->MODER |= (mode & 3U) << (n * 2);    // Set new mode
}

static inline void setAF(uint16_t pin, uint8_t af_num) {
  GPIO_TypeDef *gpio = GPIO(PINBANK(pin));  // GPIO bank
  int n = PINNO(pin);                       // Pin number
  gpio->AFR[n >> 3] &= ~(15UL << ((n & 7) * 4));
  gpio->AFR[n >> 3] |= ((uint32_t) af_num) << ((n & 7) * 4);
}

static inline void writePin(uint16_t pin, bool val) {
  GPIO_TypeDef *gpio = GPIO(PINBANK(pin));
  gpio->BSRR = (1U << PINNO(pin)) << (val ? 0 : 16);
}

static inline bool readPin(uint16_t pin) { //TODO: Switch back to bool
    GPIO_TypeDef *gpio = GPIO(PINBANK(pin)); //grab the GPIO register for the pin
    return ((((gpio -> IDR) & 0xFFFF) >> PINNO(pin)) & 1);
}

static inline uint16_t readIDR(uint16_t pin) { //TODO: Switch back to bool
    GPIO_TypeDef *gpio = GPIO(PINBANK(pin)); //grab the GPIO register for the pin
    return ((gpio -> IDR) & 0xFFFF);
}

//================================================================================================================================================
/*
    USART1(Might be used for USB): Tx: PA6/9 Rx: PA10
    USART2 (ST-Link): Tx: A2 Rx: A3
    USART6: Tx: PC6/PA11 Rx: PA12/PC7
*/
struct uart {
    volatile uint32_t SR, DR, BRR, CR1, CR2, CR3, GTPR;
};
#define UART1 USART1
#define UART2 USART2
#define UART6 USART6

static inline void uart2_init(unsigned long baud) {
    uint8_t af = 7;
    uint16_t rx = 0, tx = 0;

    RCC -> APB1ENR |= BIT(17); //6.3.11
    tx = PIN('A', 2);
    rx = PIN('A', 3);

    pinMode(tx, GPIO_MODE_AF);
    setAF(tx, af);

    pinMode(rx, GPIO_MODE_AF);
    setAF(rx, af);


    UART2 -> CR1 = 0; //disable uart
    UART2 -> BRR = FREQ / baud;
    UART2-> CR1 |= BIT(13) | BIT(2) | BIT(3); //set ue, re, te  
}

static inline int uartRead_ready(USART_TypeDef *uart) {
    return uart -> SR & BIT(5);
}

static inline uint8_t uartRead(USART_TypeDef *uart) {
    return (uint8_t) (uart -> DR & 255); //mask 8 bits of data register and return that data
}

static inline void uartWrite_byte(USART_TypeDef *uart, uint8_t byte) {
    uart -> DR = byte; //store byte into data register
    while ((uart -> SR & BIT(7)) == 0) spin(1); //wait for status register to signall that traansmission ended 
}

static inline void uartWrite_buf(USART_TypeDef *uart, char * buf, size_t len) {
    while(len-- > 0){
        uartWrite_byte(uart, *(uint8_t *) buf++); //keep sending a byte from buf until the end
    }
}
//===================================================================================================================================================
/*
    Set up PWM mode if needed
    TIMx_ARR -> Determines frequency 
    TIMx_CCRx -> Determines duty cycle
    PB6 PWM4/1 -> Timer 4 channel 1
    TIM4_ARR -> Set freq ?
    TIM4_CCR1 -> Set duty
    he corresponding preload register must be enabled by setting the
OCxPE bit in the TIMx_CCMRx register, and eventually the auto-reload preload register (in
upcounting or center-aligned modes) by setting the ARPE bit in the TIMx_CR1 register.
    OC1PE -> set auto-reload; inside TIM4_CCMR1
    ARPE -> Auto-reload(if upcounting) in TIM4_CR1


• Counter Register (TIMx_CNT)
• Prescaler Register (TIMx_PSC):
• Auto-Reload Register (TIMx_ARR)

    Edge-Aligned and upcounting


analogWrite(pin, value)
Value is the duty cycle, which is adjusted through the CCR1 register

PWM
TMR4_CNT is the current value the timer is at
CCR1 is the value set to determing when the clock will switch
ARR sets frequency, so when a full period will pass


*/
//#define TIMER4 TIM4
//change name, this will be an RGB LED Driver
static inline void rgb_led_tim4channel1_2_3(void) {
    RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOBEN; //enable B ports 
    RCC -> APB1ENR |= RCC_APB1ENR_TIM4EN; //Enable timer 4

    GPIOB -> MODER |= GPIO_MODER_MODE6_1; //set afmode
    GPIOB -> AFR[0] |= GPIO_AFRL_AFSEL6_1; //set to PWM mode which is 0010 AFR[0] low 16-bits

    GPIOB -> MODER |= GPIO_MODER_MODE7_1; //set afmode
    GPIOB -> AFR[0] |= GPIO_AFRL_AFSEL7_1; //set to PWM mode which is 0010 AFR[0] low 16-bits

    GPIOB -> MODER |= GPIO_MODER_MODE8_1; //set afmode
    GPIOB -> AFR[1] |= GPIO_AFRH_AFSEL8_1; //set to PWM mode which is 0010 AFR[0] low 16-bits

    GPIOB -> MODER |= GPIO_MODER_MODE3_0; //set PB3 to output
    

    //timer4 config
    TIM4 -> CCER &= ~TIM_CCER_CC1P; //Active high
    //TIM4 -> ARR = 100; //period
    //TIM4 -> PSC = 15999; //1kHz 16Mhz clock is divided by this value
    TIM4 -> ARR = 100; //period
    TIM4 -> PSC = 300;
    TIM4 -> CR1 |= TIM_CR1_ARPE;
    TIM4 -> CR1 |= TIM_CR1_CEN; //
    TIM4 -> EGR |= TIM_EGR_UG; //re-initalize

    //channel config
    TIM4 -> CCMR1 &= ~TIM_CCMR1_CC1S; //clear CCS bits to be safe   
    TIM4 -> CCMR2 &= ~TIM_CCMR2_CC3S;
    TIM4 -> CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1; //PWM 1 mode
    TIM4 -> CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1;

    //duty cycles
    TIM4 -> CCR1 = 100; //PB6 R
    TIM4 -> CCR2 = 90; //PB7 B
    TIM4 -> CCR3 = 10; //PB8 G


    TIM4 -> CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE; //Output compare for channel preload enable
    TIM4 -> CCMR2 |= TIM_CCMR2_OC3PE;
    TIM4 -> CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E; 
}

static inline void PB6PWM_init(void){ //Uses timer 4 channel 1 to output a PWM signal
    RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOBEN; //enable B ports 
    RCC -> APB1ENR |= RCC_APB1ENR_TIM4EN; //Enable timer 4

    GPIOB -> MODER |= GPIO_MODER_MODE6_1; //set afmode
    GPIOB -> AFR[0] |= GPIO_AFRL_AFSEL6_1; //set to PWM mode which is 0010 AFR[0] low 16-bits

    //timer4 config
    TIM4 -> CCER &= ~TIM_CCER_CC1P; //Active high
    TIM4 -> ARR = 100; //period
    TIM4 -> PSC = 323; //(16Mhz * 10^-5)/PSC = Freq(in Khz) this generates ~490Hz
    TIM4 -> CR1 |= TIM_CR1_ARPE;
    TIM4 -> CR1 |= TIM_CR1_CEN; //
    TIM4 -> EGR |= TIM_EGR_UG; //re-initalize

    //channel config
    TIM4 -> CCMR1 &= ~TIM_CCMR1_CC1S; //clear CCS bits to be safe   
    TIM4 -> CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1; //PWM 1 mode

     //duty cycles
    TIM4 -> CCR1 = 50; //PB6 R

    TIM4 -> CCMR1 |= TIM_CCMR1_OC1PE; //Output compare for channel preload enable
    TIM4 -> CCER |= TIM_CCER_CC1E; //enable PWM output
}


static inline void PB6analogWrite(uint32_t value){
    //timer4 config
    TIM4 -> CCR1 = value;
}

//-------------------------------------------------------------------------------------------------------------------
//I2C stuff
static inline void i2c1_init(void) {
    //peripheral clocks
    RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOBEN; //enable B ports 
    RCC -> APB1ENR |= RCC_APB1ENR_I2C1EN; //Enable i2c1

    //Connect pins to peripheral
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

    //I2C1 peripheral settings
    I2C1 -> CR1 &= ~I2C_CR1_PE; //turn off peripheral
    I2C1 -> CR1 |= I2C_CR1_SWRST; //toggle reset
    I2C1 -> CR1 &= ~I2C_CR1_SWRST;
    I2C1 -> OAR1 = 0x0022;//microcontroller address(master)
    I2C1 -> OAR1 |= (1 << 14); //Needs to be set by software
    I2C1 -> CR2 = 0x10; //peripheral clock to match apb clock of 16Mhz
    I2C1 -> CCR = 0x50; //set to 80, which is for 5000ns
    I2C1 -> TRISE = 0x11; //Trise max is 1000ns, follows example
    I2C1 -> CR1 |= I2C_CR1_PE;
}

static inline void I2C1_Master_Start(void) {
    I2C1 -> CR1 &= ~I2C_CR1_START;
    I2C1 -> CR1 |= I2C_CR1_START; //toggle start condition
    while(!(I2C1 -> SR1 & I2C_SR1_SB)){} //wait for start condition to be on
    (void)I2C1->SR1;
}

static inline void I2C1_Master_Stop(void) {
    I2C1 -> CR1 |= I2C_CR1_STOP; //set stop condition to let bus float 
    while(!(I2C_SR2_BUSY)){};
}

//TODO: add I2C1 -> SR1 check into while statements to make sure no errors are returned, but for now we dont care about that, just making sure we can send data
static inline void I2C1_Master_Send_Address(uint8_t address) {
    I2C1_Master_Start();
    I2C1 -> DR = (address << 1) | 0x00; //store slave address and 0 for write
    while(!(I2C1 -> SR1 & I2C_SR1_ADDR)){} //wait until address bit is set(end of address transmission), set after ACK
    (void)I2C1->SR1; //dummy read to clear these flags
    (void)I2C1->SR2;
    //I2C1_Master_Stop();
}

static inline void I2C1_Master_Send_Data(char data) {
    //I2C1_Master_Start(); //set start condition to begin transfer
    I2C1 -> DR = data; //load data into register to begin transmission
    while(!(I2C1 -> SR1 & I2C_SR1_TXE)){};
    //I2C1_Master_Stop();
}

static inline uint8_t I2C1_Master_Receive(uint8_t address) {
    I2C1 -> CR1 &= ~I2C_CR1_POS;
    I2C1 -> CR1 |= I2C_CR1_ACK;
    I2C1_Master_Start();
    I2C1 -> DR = (uint8_t)(address << 1) | 0x01; //store slave address and 1 for read
    while(!(I2C1 -> SR1 & I2C_SR1_ADDR)){} //wait until address bit is set(end of address transmission), set after ACK
    (void)I2C1->SR1; //dummy read to clear these flags
    (void)I2C1->SR2;

    //int data;
    uint8_t data1, data2;
    while(!(I2C_SR1_RXNE)){};
    //data = (uint8_t)I2C1->DR;
    //spin(200);
    printf("Here 1\r\n");
    I2C1 -> CR1 |= I2C_CR1_ACK;
    data1 = (uint8_t)I2C1->DR;
    printf("Here 2\r\n");
    //spin(200);
    data2 = (uint8_t)I2C1->DR;
    printf("Here 3\r\n");
    //spin(200);
    I2C1_Master_Stop();
    (void)data2;
    (void)data1;
    printf("Here 4, %x is data 1, %x is data2\r\n", data1, data2);
    //data = (uint8_t)I2C1->DR;
    spin(200);
    
    return data1;
}

static inline void I2C1_Master_FixBusy(void) {
    uint16_t SCL = PIN('B', 8);
    uint16_t SDA = PIN('B', 9);
    if((readPin(SCL) == 0) && (readPin(SDA) == 0)) {
        pinMode(SCL, GPIO_MODE_OUTPUT);
        //pinMode(SDA, GPIO_MODE_OUTPUT);
        while((readPin(SCL) == 0) && (readPin(SDA) == 0)) {
            for(int cycles = 0; cycles < 25; cycles++) {
                writePin(SCL, 1);
                spin(10);
                writePin(SCL, 0);
                spin(10);
            }
        }
    }
    //reset the pins
    pinMode(SCL, GPIO_MODE_AF);
    setAF(SCL, 4);
    GPIOB -> OTYPER |= GPIO_OTYPER_OT8; //set to open drain
    //Set up PB8, Alternate function, I2C1 connected
    pinMode(SDA, GPIO_MODE_AF);
    setAF(SDA, 4);
    GPIOB -> OTYPER |= GPIO_OTYPER_OT9; //open drain
    
}