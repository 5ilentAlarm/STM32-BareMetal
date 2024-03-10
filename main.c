#include "stm32f401re.h"
#include "lib/ADS1115.h"
#include "lib/MCP23017.h"

#define BAUD 115200

static volatile uint32_t s_ticks;
void SysTick_Handler(void) {
  s_ticks++;
}

uint32_t SystemCoreClock = FREQ;
void SystemInit(void) {
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;    // Enable SYSCFG
  SysTick_Config(SystemCoreClock / 1000);  // Tick every 1 ms
}
 
uint32_t millis(void) {
    return s_ticks;
}

static inline void delay(uint32_t time) {
    uint32_t currentTime = millis();
    while(currentTime + time > s_ticks) { //if we want to delay for 500ms, current s_ticks is 20, we want to wait untill 520ms
        //do nothing
    }
}

int main(void) {
  uart2_init(BAUD); //enable uart for printf
  delay(1);
  i2c1_init();
  printf("I2C init finished\r\n");

  MCP23017_init(MCP23017_DEFAULT_ADDR);
  MCP23017_CONFIGURE(MCP23017_DEFAULT_ADDR, 0x20);
  MCP23017_PIN_DIRECTION(MCP23017_DEFAULT_ADDR, MCP23017_REG_IODIRA, 0xF0);
  MCP23017_PIN_WRITE(MCP23017_DEFAULT_ADDR, MCP23017_REG_GPIOA, 0x02);
  printf("Setup Complete!\r\n");

  //Read IODIR register
  uint8_t mcpReg = 0;
  mcpReg = MCP23017_READ(MCP23017_DEFAULT_ADDR, MCP23017_REG_IODIRA);
  printf("Value of specified register: %x\r\n", mcpReg);
  printf("Read complete!\r\n");


  for (;;) {
  }
  return 0;
}

