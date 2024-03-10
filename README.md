# What is this?

I try my best to learn new things, and one of those things is writing drivers for various other things... In this case I like to add new discoveries to this template, which I use for other projects. 

# Progress Tracker

ADS1115: Basic functions work, need to add more definitions for internal registers to make it easier to configure. 

MCP23017: Working.

# Debugging

I've been using the GDB-multiarch debugger to step through code and make sure registers are all good, but printf debugging works as well, I've installed tio on my machine to connect to the serial port. 

# Mentions
CPQ's tutorial [Bare Metal Programming Guide](https://github.com/cpq/bare-metal-programming-guide) greatly helped me set up the environment to start programming the STM32. Just needed to make sure that the specs for the specific chip transfers over to the linker script (such as flash memory, etc.).
