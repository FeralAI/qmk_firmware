#pragma once

#include "i2c_master.h"

#ifndef _MCP23017_H_
#define _MCP23017_H_

/**
 * MCP23017 defintions
 */

#define MCP23017_ADDRESS 0x20

/**
 * MCP23017 Registers - See MCP23017/MCP23S17 datasheet, TABLE 3-5 for details
 *
 * -------------------------------------------------------------------------------------------------------
 * | REGISTER | ADDR | BIT 7  | BIT 6  | BIT 5  | BIT 4  | BIT3 3 | BIT 2  | BIT 1  | BIT 0  | POR/RST   |
 * -------------------------------------------------------------------------------------------------------
 * | IODIRA   | 00   | IO7    | IO6    | IO5    | IO4    | IO3    | IO2    | IO1    | IO0    | 1111 1111 |
 * | IODIRB   | 01   | IO7    | IO6    | IO5    | IO4    | IO3    | IO2    | IO1    | IO0    | 1111 1111 |
 * | IPOLA    | 02   | IP7    | IP6    | IP5    | IP4    | IP3    | IP2    | IP1    | IP0    | 0000 0000 |
 * | IPOLB    | 03   | IP7    | IP6    | IP5    | IP4    | IP3    | IP2    | IP1    | IP0    | 0000 0000 |
 * | GPINTENA | 04   | GPINT7 | GPINT6 | GPINT5 | GPINT4 | GPINT3 | GPINT2 | GPINT1 | GPINT0 | 0000 0000 |
 * | GPINTENB | 05   | GPINT7 | GPINT6 | GPINT5 | GPINT4 | GPINT3 | GPINT2 | GPINT1 | GPINT0 | 0000 0000 |
 * | DEFVALA  | 06   | DEF7   | DEF6   | DEF5   | DEF4   | DEF3   | DEF2   | DEF1   | DEF0   | 0000 0000 |
 * | DEFVALB  | 07   | DEF7   | DEF6   | DEF5   | DEF4   | DEF3   | DEF2   | DEF1   | DEF0   | 0000 0000 |
 * | INTCONA  | 08   | IOC7   | IOC6   | IOC5   | IOC4   | IOC3   | IOC2   | IOC1   | IOC0   | 0000 0000 |
 * | INTCONB  | 09   | IOC7   | IOC6   | IOC5   | IOC4   | IOC3   | IOC2   | IOC1   | IOC0   | 0000 0000 |
 * | IOCON    | 0A   | BANK   | MIRROR | SEQOP  | DISSLW | HAEN   | ODR    | INTPOL | —      | 0000 0000 |
 * | IOCON    | 0B   | BANK   | MIRROR | SEQOP  | DISSLW | HAEN   | ODR    | INTPOL | —      | 0000 0000 |
 * | GPPUA    | 0C   | PU7    | PU6    | PU5    | PU4    | PU3    | PU2    | PU1    | PU0    | 0000 0000 |
 * | GPPUB    | 0D   | PU7    | PU6    | PU5    | PU4    | PU3    | PU2    | PU1    | PU0    | 0000 0000 |
 * | INTFA    | 0E   | INT7   | INT6   | INT5   | INT4   | INT3   | INT2   | INT1   | INTO   | 0000 0000 |
 * | INTFB    | 0F   | INT7   | INT6   | INT5   | INT4   | INT3   | INT2   | INT1   | INTO   | 0000 0000 |
 * | INTCAPA  | 10   | ICP7   | ICP6   | ICP5   | ICP4   | ICP3   | ICP2   | ICP1   | ICP0   | 0000 0000 |
 * | INTCAPB  | 11   | ICP7   | ICP6   | ICP5   | ICP4   | ICP3   | ICP2   | ICP1   | ICP0   | 0000 0000 |
 * | GPIOA    | 12   | GP7    | GP6    | GP5    | GP4    | GP3    | GP2    | GP1    | GP0    | 0000 0000 |
 * | GPIOB    | 13   | GP7    | GP6    | GP5    | GP4    | GP3    | GP2    | GP1    | GP0    | 0000 0000 |
 * | OLATA    | 14   | OL7    | OL6    | OL5    | OL4    | OL3    | OL2    | OL1    | OL0    | 0000 0000 |
 * | OLATB    | 15   | OL7    | OL6    | OL5    | OL4    | OL3    | OL2    | OL1    | OL0    | 0000 0000 |
 * -------------------------------------------------------------------------------------------------------
 */

// IO Direction register
// IO<7:0> Controls data direction: 0 = Output, 1 = Input
#define MCP23017_IODIRA   0x00
#define MCP23017_IODIRB   0x01

// Input polarity port register
// IP<7:0> Control polarity inversion of input pins: 0 = Same pin logic, 1 = Opposite pin logic
#define MCP23017_IPOLA    0x02
#define MCP23017_IPOLB    0x03

// Interrupt-on-change pins
// GPINT<7:0> GPIO interrupt-on-change flag: 0 = Disabled, 0 = Enabled
#define MCP23017_GPINTENA 0x04
#define MCP23017_GPINTENB 0x05

// Default value register
// DEF<7:0> Set the value to compare for interrupt-on-change: 0, 1
#define MCP23017_DEFVALA  0x06
#define MCP23017_DEFVALB  0x07

// Interrupt-on-change control register
// IOC<7:0> Controls how pin value is compared to trigger interrupt-on-change: 0 = Previous value, 1 = DEFVAL
#define MCP23017_INTCONA  0x08
#define MCP23017_INTCONB  0x09

// I/O expander config register - both addresses access the same register
// BIT7 BANK   - Controls how registers are addressed: 1 = Resisters in different banks
// BIT6 MIRROR - Interrupt pins mirror (shared): 0 = Not connected, 1 = Mirrored
// BIT5 SEQOP  - Sequential operation mode (auto increment address pointer): 0 = Enabled, 1 = Disabled
// BIT4 DISSLW - Slew Rate control for SDA output: 0 = Disabled, 1 = Enabled
// BIT3 HAEN   - Hardware Address Enable bit (MCP23S17 only): 0 = Disabled, 1 = Enabled
// BIT2 ODR    - Configure interrupt pins as open-drain output: 0 = Disabled, 1 = Enabled
// BIT1 INTPOL - Set the polarity of the interrupt pins: 0 = Low, 1 = High
// BIT0        - Unused
#define MCP23017_IOCONA   0x0A
#define MCP23017_IOCONB   0x0B

// GPIO pull-up resistor register
// PU<7:0> Controls the weak pull-up resistors for each pin: 0 = Disabled, 1 = Enabled
#define MCP23017_GPPUA    0x0C
#define MCP23017_GPPUB    0x0D

// Interrupt flag register
// INT<7:0> Holds the current interrupt state: 0 = None, 1 = Interrupt
#define MCP23017_INTFA    0x0E
#define MCP23017_INTFB    0x0F

// Interrupt captured value for port register
// <ICP7:0> Logic level of pins at time of interrupt: 0 = Low, 1 = High
#define MCP23017_INTCAPA  0x10
#define MCP23017_INTCAPB  0x11

// GPIO port register
// <GP7:0> Set logic level of GPIO pins: 0 = Low, 1 = High
#define MCP23017_GPIOA    0x12
#define MCP23017_GPIOB    0x13

// Output latch register
// <OL7:0> Logic level of output latch: 0 = Low, 1 = High
#define MCP23017_OLATA    0x14
#define MCP23017_OLATB    0x15

#define MCP23017_INT_ERR     255
#define MCP23017_I2C_TIMEOUT 100


/**
 * Pin definitions
 */

// GPIO pin definitions
#define GPA0 0x0
#define GPA1 0x1
#define GPA2 0x2
#define GPA3 0x3
#define GPA4 0x4
#define GPA5 0x5
#define GPA6 0x6
#define GPA7 0x7
#define GPB0 0x8
#define GPB1 0x9
#define GPB2 0xA
#define GPB3 0xB
#define GPB4 0xC
#define GPB5 0xD
#define GPB6 0xE
#define GPB7 0xF


// Port state definitions
# ifndef LOW
# define LOW 0
# endif

# ifndef HIGH
# define HIGH 1
# endif

# ifndef INPUT
# define INPUT 1
# endif

# ifndef OUTPUT
# define OUTPUT 0
# endif

# ifndef CHANGE
# define CHANGE 1
# endif

# ifndef FALLING
# define FALLING 2
# endif

# ifndef RISING
# define RISING 3
# endif


// Initialization functions
void mcp23017_begin(uint8_t addrOffset);
void mcp23017_begin_default(void);

// Read/write register functions
uint8_t mcp23017_readRegister(uint8_t reg, uint8_t *data);
uint8_t mcp23017_readRegisters(uint8_t reg, uint16_t* data);
uint8_t mcp23017_writeRegister(uint8_t reg, uint8_t data);
uint8_t mcp23017_writeRegisters(uint8_t reg, uint16_t data);

// GPIO single-pin functions
uint8_t mcp23017_digitalRead(uint8_t pin);
void mcp23017_digitalWrite(uint8_t pin, uint8_t value);
void mcp23017_pinMode(uint8_t pin, uint8_t value);
void mcp23017_pullUp(uint8_t pin, uint8_t value);

// GPIO register functions
uint8_t mcp23017_readGPIO(uint8_t port);
uint16_t mcp23017_readGPIOAB(void);
void mcp23017_writeGPIO(uint8_t port, uint8_t value);
void mcp23017_writeGPIOAB(uint16_t ba);

// Interrupt functions
void mcp23017_setupInterrupts(uint8_t mirroring, uint8_t open, uint8_t polarity);
void mcp23017_setupInterruptPin(uint8_t pin, uint8_t mode);
uint8_t mcp23017_getLastInterruptPin(void);
uint8_t mcp23017_getLastInterruptPinValue(void);

#endif
