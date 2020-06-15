#include "spi_master.h"
#include "mcp23s08.h"
#include "wait.h"

uint8_t spiaddr; // The bit-shifted (<< 1) I2C address
pin_t slavePin;
bool lsbFirst = false;
uint8_t spiMode = 0;
uint16_t divisor = 2;


/**
 * Helper functions
 */

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

// Bit number associated to a give pin

// Helper to update a single bit of an A/B register.
static void updateRegisterBit(uint8_t pin, uint8_t pValue, uint8_t portAddr) {
    uint8_t regValue;

    // Read current register state
    mcp23s08_readRegister(portAddr, &regValue);

    // Update current state for pin
    bitWrite(regValue, pin, pValue);
    spi_writeReg(spiaddr, portAddr, &regValue, 1, MCP23S08_TIMEOUT);
}


/**
 * Initialization functions
 */

// Initializes the MCP23S08 given its HW selected address, see datasheet for Address selection.
void mcp23s08_begin(uint8_t cp, uint8_t addrOffset) {
    // MCP23S08 addresses from 0x20-0x23, enforce offset constraint
    if (addrOffset > 3)
        addrOffset = 3;

    // Offset the base address by addr value, then left shift 1 to make room for read/write bit
    spiaddr = (MCP23S08_ADDRESS | addrOffset) << 1;
    slavePin = cp;

    spi_init();
    _delay_ms(1000);

    // set defaults!
    // all inputs on port A and B
    mcp23s08_writeRegister(MCP23S08_IODIR, 0xFF);
}

// Initializes the default MCP23S08, with 00 for the configurable part of the address
void mcp23s08_begin_default(uint8_t cp) {
    mcp23s08_begin(cp, 0);
}


/**
 * Read/write register functions
 */

uint8_t mcp23s08_readRegister(uint8_t reg, uint8_t *data) {
    uint8_t ret = 0;

    ret = spi_start(slavePin, lsbFirstspiaddr, MCP23S08_TIMEOUT); if (ret) goto out;
    ret = spi_write(reg, MCP23S08_TIMEOUT);                       if (ret) goto out;
    ret = spi_receive(spiaddr, data, 1, MCP23S08_TIMEOUT);        if (ret) goto out;

out:
    spi_stop();
    return ret;
}

uint8_t mcp23s08_writeRegister(uint8_t reg, uint8_t data) {
    uint8_t ret = 0;

    ret = spi_start(spiaddr, MCP23S08_TIMEOUT);  if (ret) goto out;
    ret = spi_write(reg, MCP23S08_TIMEOUT);      if (ret) goto out;
    ret = spi_write(data, MCP23S08_TIMEOUT);     if (ret) goto out;

out:
    spi_stop();
    return ret;
}


/**
 * GPIO single-pin functions
 */

uint8_t mcp23s08_digitalRead(uint8_t pin) {
    uint8_t gpio;
    mcp23s08_readRegister(MCP23S08_GPIO, &gpio);
    return (gpio >> pin) & 0x1;
}

void mcp23s08_digitalWrite(uint8_t pin, uint8_t value) {
    uint8_t gpio;

    // read the current GPIO output latches
    mcp23s08_readRegister(MCP23S08_OLAT, &gpio);

    // set the pin and direction
    bitWrite(gpio, pin, value);

    // write the new GPIO
    mcp23s08_writeRegister(MCP23S08_GPIO, gpio);
}

// Sets the pin mode to either INPUT or OUTPUT
void mcp23s08_pinMode(uint8_t pin, uint8_t value) {
    updateRegisterBit(pin, value, MCP23S08_IODIR);
}

// Sets the pull-up resistor to either HIGH or LOW
void mcp23s08_pullUp(uint8_t pin, uint8_t value) {
    updateRegisterBit(pin, value, MCP23S08_GPPU);
}


/**
 * GPIO register functions
 */

// Read the GPIO register and return its current 8 bit value.
uint8_t mcp23s08_readGPIO(void) {
    uint8_t data;
    mcp23s08_readRegister(MCP23S08_GPIO, &data);
    return data;
}

// Write a single port, A or B
void mcp23s08_writeGPIO(uint8_t value) {
    mcp23s08_writeRegister(MCP23S08_GPIO, value);
}


/**
 * Interrupt functions
 */

// Configures the interrupt system. both port A and B are assigned the same configuration.
// Mirroring will OR both INTA and INTB pins.
// Opendrain will set the INT pin to value or open drain.
// polarity will set LOW or HIGH on interrupt.
// Default values after Power On Reset are: (false, false, LOW)
// If you are connecting the INTA/B pin to arduino 2/3, you should configure the interupt handling as FALLING with
// the default configuration.
uint8_t mcp23s08_getLastInterruptPin() {
    uint8_t intf;

    mcp23s08_readRegister(MCP23S08_INTF, &intf);
    for (int i = 0; i < 8; i++)
        if (bitRead(intf, i))
            return i;

    return MCP23S08_INT_ERR;

}

uint8_t mcp23s08_getLastInterruptPinValue() {
    uint8_t intPin = mcp23s08_getLastInterruptPin();
    if (intPin == MCP23S08_INT_ERR)
        return MCP23S08_INT_ERR;

    uint8_t val;
    mcp23s08_readRegister(MCP23S08_INTCAP, &val);
    return (val >> intPin) & 0x01;
}

void mcp23s08_setupInterrupts(uint8_t openDrain, uint8_t polarity) {
    uint8_t ioconfValue;

    // configure the port A
    mcp23s08_readRegister(MCP23S08_IOCON, &ioconfValue);
    bitWrite(ioconfValue, 2, openDrain);
    bitWrite(ioconfValue, 1, polarity);
    mcp23s08_writeRegister(MCP23S08_IOCON, ioconfValue);
}

// Set's up a pin for interrupt. uses arduino MODEs: CHANGE, FALLING, RISING.
// Note that the interrupt condition finishes when you read the information about the port / value
// that caused the interrupt or you read the port itself. Check the datasheet can be confusing.
void mcp23s08_setupInterruptPin(uint8_t pin, uint8_t mode) {
    // set the pin interrupt control (0 means change, 1 means compare against given value);
    updateRegisterBit(pin, (mode != CHANGE), MCP23S08_INTCON);
    // if the mode is not CHANGE, we need to set up a default value, different value triggers interrupt

    // In a RISING interrupt the default value is 0, interrupt is triggered when the pin goes to 1.
    // In a FALLING interrupt the default value is 1, interrupt is triggered when pin goes to 0.
    updateRegisterBit(pin, (mode == FALLING), MCP23S08_DEFVAL);

    // enable the pin for interrupt
    updateRegisterBit(pin, HIGH, MCP23S08_GPINTEN);

}
