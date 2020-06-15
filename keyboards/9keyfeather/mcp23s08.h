#pragma once

#ifndef _MCP23S08_H_
#define _MCP23S08_H_

/**
 * MCP23S08 defintions
 */

#define MCP23S08_ADDRESS 0x20

/**
 * MCP23S08 Registers - See MCP23008/MCP23S08 datasheet, TABLE 3-5 for details
 *
 * -------------------------------------------------------------------------------------------------------
 * | REGISTER | ADDR | BIT 7  | BIT 6  | BIT 5  | BIT 4  | BIT3 3 | BIT 2  | BIT 1  | BIT 0  | POR/RST   |
 * -------------------------------------------------------------------------------------------------------
 * | IODIR    | 00   | IO7    | IO6    | IO5    | IO4    | IO3    | IO2    | IO1    | IO0    | 1111 1111 |
 * | IPOL     | 01   | IP7    | IP6    | IP5    | IP4    | IP3    | IP2    | IP1    | IP0    | 0000 0000 |
 * | GPINTEN  | 02   | GPINT7 | GPINT6 | GPINT5 | GPINT4 | GPINT3 | GPINT2 | GPINT1 | GPINT0 | 0000 0000 |
 * | DEFVAL   | 03   | DEF7   | DEF6   | DEF5   | DEF4   | DEF3   | DEF2   | DEF1   | DEF0   | 0000 0000 |
 * | INTCON   | 04   | IOC7   | IOC6   | IOC5   | IOC4   | IOC3   | IOC2   | IOC1   | IOC0   | 0000 0000 |
 * | IOCON    | 05   | -      | -      | SEQOP  | DISSLW | HAEN   | ODR    | INTPOL | —      | --00 000- |
 * | GPPU     | 06   | PU7    | PU6    | PU5    | PU4    | PU3    | PU2    | PU1    | PU0    | 0000 0000 |
 * | INTF     | 07   | INT7   | INT6   | INT5   | INT4   | INT3   | INT2   | INT1   | INTO   | 0000 0000 |
 * | INTCAP   | 08   | ICP7   | ICP6   | ICP5   | ICP4   | ICP3   | ICP2   | ICP1   | ICP0   | 0000 0000 |
 * | GPIO     | 09   | GP7    | GP6    | GP5    | GP4    | GP3    | GP2    | GP1    | GP0    | 0000 0000 |
 * | OLAT     | 0A   | OL7    | OL6    | OL5    | OL4    | OL3    | OL2    | OL1    | OL0    | 0000 0000 |
 * -------------------------------------------------------------------------------------------------------
 */

// IO Direction register
// IO<7:0> Controls data direction: 0 = Output, 1 = Input
#define MCP23S08_IODIR   0x00

// Input polarity port register
// IP<7:0> Control polarity inversion of input pins: 0 = Same pin logic, 1 = Opposite pin logic
#define MCP23S08_IPOL    0x01

// Interrupt-on-change pins
// GPINT<7:0> GPIO interrupt-on-change flag: 0 = Disabled, 0 = Enabled
#define MCP23S08_GPINTEN 0x02

// Default value register
// DEF<7:0> Set the value to compare for interrupt-on-change: 0, 1
#define MCP23S08_DEFVAL  0x03

// Interrupt-on-change control register
// IOC<7:0> Controls how pin value is compared to trigger interrupt-on-change: 0 = Previous value, 1 = DEFVAL
#define MCP23S08_INTCON  0x04

// I/O expander config register - both addresses access the same register
// BIT7        - Unused
// BIT6        - Unused
// BIT5 SEQOP  - Sequential operation mode (auto increment address pointer): 0 = Enabled, 1 = Disabled
// BIT4 DISSLW - Slew Rate control for SDA output: 0 = Disabled, 1 = Enabled
// BIT3 HAEN   - Hardware Address Enable bit (MCP23S17 only): 0 = Disabled, 1 = Enabled
// BIT2 ODR    - Configure interrupt pins as open-drain output: 0 = Disabled, 1 = Enabled
// BIT1 INTPOL - Set the polarity of the interrupt pins: 0 = Low, 1 = High
// BIT0        - Unused
#define MCP23S08_IOCON   0x05

// GPIO pull-up resistor register
// PU<7:0> Controls the weak pull-up resistors for each pin: 0 = Disabled, 1 = Enabled
#define MCP23S08_GPPU    0x06

// Interrupt flag register
// INT<7:0> Holds the current interrupt state: 0 = None, 1 = Interrupt
#define MCP23S08_INTF    0x07

// Interrupt captured value for port register
// <ICP7:0> Logic level of pins at time of interrupt: 0 = Low, 1 = High
#define MCP23S08_INTCAP  0x08

// GPIO port register
// <GP7:0> Set logic level of GPIO pins: 0 = Low, 1 = High
#define MCP23S08_GPIO    0x09

// Output latch register
// <OL7:0> Logic level of output latch: 0 = Low, 1 = High
#define MCP23S08_OLAT    0x0A

#define MCP23S08_INT_ERR 255
#define MCP23S08_TIMEOUT 100


/**
 * Pin definitions
 */

// GPIO pin definitions
#define GP0 0x0
#define GP1 0x1
#define GP2 0x2
#define GP3 0x3
#define GP4 0x4
#define GP5 0x5
#define GP6 0x6
#define GP7 0x7


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
void mcp23s08_begin(uint8_t cp, uint8_t addrOffset);
void mcp23s08_begin_default(uint8_t cp);

// Read/write register functions
uint8_t mcp23s08_readRegister(uint8_t reg, uint8_t *data);
uint8_t mcp23s08_writeRegister(uint8_t reg, uint8_t data);

// GPIO single-pin functions
uint8_t mcp23s08_digitalRead(uint8_t pin);
void mcp23s08_digitalWrite(uint8_t pin, uint8_t value);
void mcp23s08_pinMode(uint8_t pin, uint8_t value);
void mcp23s08_pullUp(uint8_t pin, uint8_t value);

// GPIO register functions
uint8_t mcp23s08_readGPIO(void);
void mcp23s08_writeGPIO(uint8_t value);

// Interrupt functions
void mcp23s08_setupInterrupts(uint8_t open, uint8_t polarity);
void mcp23s08_setupInterruptPin(uint8_t pin, uint8_t mode);
uint8_t mcp23s08_getLastInterruptPin(void);
uint8_t mcp23s08_getLastInterruptPinValue(void);

#endif
