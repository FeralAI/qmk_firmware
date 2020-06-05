// #include "action.h"
#include "mcp23017.h"
#include "wait.h"

// The bit-shifted (<< 1) I2C address
uint8_t i2caddr;


/**
 * Helper functions
 */

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

// Bit number associated to a give pin
uint8_t bitForPin(uint8_t pin) {
    return pin % 8;
}

// Register address, port dependent, for a given pin
uint8_t regForPin(uint8_t pin, uint8_t portAaddr, uint8_t portBaddr) {
    return (pin < 8) ? portAaddr : portBaddr;
}

// Helper to update a single bit of an A/B register.
void updateRegisterBit(uint8_t pin, uint8_t pValue, uint8_t portAaddr, uint8_t portBaddr) {
    uint8_t regValue;
    uint8_t regAddr = regForPin(pin, portAaddr, portBaddr);

    // Read current register state
    mcp23017_readRegister(regAddr, &regValue);

    // Update current state for pin
    uint8_t bit = bitForPin(pin);
    bitWrite(regValue, bit, pValue);
    i2c_writeReg(i2caddr, regAddr, &regValue, 1, MCP23017_I2C_TIMEOUT);
}


/**
 * Initialization functions
 */

// Initializes the MCP23017 given its HW selected address, see datasheet for Address selection.
void mcp23017_begin(uint8_t addrOffset) {
    // MCP23017 addresses from 0x20-0x27, enforce offset constraint
    if (addrOffset > 7)
        addrOffset = 7;

    // Offset the base address by addr value, then left shift 1 to make room for read/write bit
    i2caddr = (MCP23017_ADDRESS | addrOffset) << 1;

    i2c_init();
    _delay_ms(1000);

    // set defaults!
    // all inputs on port A and B
    mcp23017_writeRegister(MCP23017_IODIRA, 0xFF);
    mcp23017_writeRegister(MCP23017_IODIRB, 0xFF);
}

// Initializes the default MCP23017, with 000 for the configurable part of the address
void mcp23017_begin_default() {
    mcp23017_begin(0);
}


/**
 * Read/write register functions
 */

uint8_t mcp23017_readRegister(uint8_t reg, uint8_t *data) {
    uint8_t ret = 0;

    ret = i2c_start(i2caddr, MCP23017_I2C_TIMEOUT);                   if (ret) goto out;
    ret = i2c_write(reg, MCP23017_I2C_TIMEOUT);                       if (ret) goto out;
    ret = i2c_receive(i2caddr, data, 1, MCP23017_I2C_TIMEOUT);        if (ret) goto out;

out:
    i2c_stop();
    return ret;
}

uint8_t mcp23017_readRegisters(uint8_t reg, uint16_t *data) {
    uint8_t ret = 0;
    uint8_t values[2];

    ret = i2c_start(i2caddr, MCP23017_I2C_TIMEOUT);                   if (ret) goto out;
    ret = i2c_write(reg, MCP23017_I2C_TIMEOUT);                       if (ret) goto out;
    ret = i2c_receive(i2caddr, values, 2, MCP23017_I2C_TIMEOUT); if (ret) goto out;

    *data = values[1];
    *data <<= 8;
    *data |= values[0];

out:
    i2c_stop();
    return ret;
}

uint8_t mcp23017_writeRegister(uint8_t reg, uint8_t data) {
    uint8_t ret = 0;

    ret = i2c_start(i2caddr, MCP23017_I2C_TIMEOUT);  if (ret) goto out;
    ret = i2c_write(reg, MCP23017_I2C_TIMEOUT);      if (ret) goto out;
    ret = i2c_write(data, MCP23017_I2C_TIMEOUT);     if (ret) goto out;

out:
    i2c_stop();
    return ret;
}

uint8_t mcp23017_writeRegisters(uint8_t reg, uint16_t data) {
    uint8_t ret = 0;
    uint8_t values[2];

    values[0] = data & 0xFF;
    values[1] = data >> 8;

    ret = i2c_start(i2caddr, MCP23017_I2C_TIMEOUT);  if (ret) goto out;
    ret = i2c_write(reg, MCP23017_I2C_TIMEOUT);      if (ret) goto out;

    uint8_t length = sizeof(&values) / sizeof(uint8_t);
    for (int i = 0; i < length; i++) {
        ret = i2c_write(values[i], MCP23017_I2C_TIMEOUT);
        if (ret)
            goto out;
    }

out:
    i2c_stop();
    return ret;
}


/**
 * GPIO single-pin functions
 */

uint8_t mcp23017_digitalRead(uint8_t pin) {
    uint8_t bit = bitForPin(pin);
    uint8_t regAddr = regForPin(pin, MCP23017_GPIOA, MCP23017_GPIOB);
    uint8_t gpio;
    mcp23017_readRegister(regAddr, &gpio);
    return (gpio >> bit) & 0x1;
}

void mcp23017_digitalWrite(uint8_t pin, uint8_t value) {
    uint8_t gpio;
    uint8_t bit = bitForPin(pin);

    // read the current GPIO output latches
    uint8_t regAddr = regForPin(pin, MCP23017_OLATA, MCP23017_OLATB);
    mcp23017_readRegister(regAddr, &gpio);

    // set the pin and direction
    bitWrite(gpio, bit, value);

    // write the new GPIO
    regAddr = regForPin(pin, MCP23017_GPIOA, MCP23017_GPIOB);
    mcp23017_writeRegister(regAddr, gpio);
}

// Sets the pin mode to either INPUT or OUTPUT
void mcp23017_pinMode(uint8_t pin, uint8_t value) {
    updateRegisterBit(pin, value, MCP23017_IODIRA, MCP23017_IODIRB);
}

// Sets the pull-up resistor to either HIGH or LOW
void mcp23017_pullUp(uint8_t pin, uint8_t value) {
    updateRegisterBit(pin, value, MCP23017_GPPUA, MCP23017_GPPUB);
}


/**
 * GPIO register functions
 */

// Read a single port, A or B, and return its current 8 bit value.
// Parameter "port" should be 0 for GPIOA, and 1 for GPIOB.
uint8_t mcp23017_readGPIO(uint8_t port) {
    uint8_t data;
    mcp23017_readRegister((port == 0) ? MCP23017_GPIOA : MCP23017_GPIOB, &data);
    return data;
}

// Reads all 16 pins (port A and B) into a single 16 bit variable.
uint16_t mcp23017_readGPIOAB() {
    uint16_t data;
    mcp23017_readRegisters(MCP23017_GPIOA, &data);
    return data;
}

// Write a single port, A or B
// Parameter "port" should be 0 for GPIOA, and 1 for GPIOB.
void mcp23017_writeGPIO(uint8_t port, uint8_t value) {
    mcp23017_writeRegister((port == 0) ? MCP23017_GPIOA : MCP23017_GPIOB, value);
}

// Writes all the pins in one go. This method is very useful if you are implementing a multiplexed matrix and want to get a decent refresh rate.
void mcp23017_writeGPIOAB(uint16_t ba) {
    mcp23017_writeRegisters(MCP23017_GPIOA, ba);
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
uint8_t mcp23017_getLastInterruptPin() {
    uint8_t intf;

    // try port A
    mcp23017_readRegister(MCP23017_INTFA, &intf);
    for (int i = 0; i < 8; i++)
        if (bitRead(intf, i))
            return i;

    // try port B
    mcp23017_readRegister(MCP23017_INTFB, &intf);
    for (int i = 0; i < 8; i++)
        if (bitRead(intf, i))
            return i+8;

    return MCP23017_INT_ERR;

}

uint8_t mcp23017_getLastInterruptPinValue() {
    uint8_t intPin = mcp23017_getLastInterruptPin();
    if (intPin == MCP23017_INT_ERR)
        return MCP23017_INT_ERR;

    uint8_t intcapreg = regForPin(intPin, MCP23017_INTCAPA, MCP23017_INTCAPB);
    uint8_t bit = bitForPin(intPin);
    uint8_t val;
    mcp23017_readRegister(intcapreg, &val);
    return (val >> bit) & 0x01;
}

void mcp23017_setupInterrupts(uint8_t mirroring, uint8_t openDrain, uint8_t polarity) {
    uint8_t ioconfValue;

    // configure the port A
    mcp23017_readRegister(MCP23017_IOCONA, &ioconfValue);
    bitWrite(ioconfValue, 6, mirroring);
    bitWrite(ioconfValue, 2, openDrain);
    bitWrite(ioconfValue, 1, polarity);
    mcp23017_writeRegister(MCP23017_IOCONA, ioconfValue);

    // Configure the port B
    mcp23017_readRegister(MCP23017_IOCONB, &ioconfValue);
    bitWrite(ioconfValue, 6, mirroring);
    bitWrite(ioconfValue, 2, openDrain);
    bitWrite(ioconfValue, 1, polarity);
    mcp23017_writeRegister(MCP23017_IOCONB, ioconfValue);
}

// Set's up a pin for interrupt. uses arduino MODEs: CHANGE, FALLING, RISING.
// Note that the interrupt condition finishes when you read the information about the port / value
// that caused the interrupt or you read the port itself. Check the datasheet can be confusing.
void mcp23017_setupInterruptPin(uint8_t pin, uint8_t mode) {
    // set the pin interrupt control (0 means change, 1 means compare against given value);
    updateRegisterBit(pin, (mode != CHANGE), MCP23017_INTCONA, MCP23017_INTCONB);
    // if the mode is not CHANGE, we need to set up a default value, different value triggers interrupt

    // In a RISING interrupt the default value is 0, interrupt is triggered when the pin goes to 1.
    // In a FALLING interrupt the default value is 1, interrupt is triggered when pin goes to 0.
    updateRegisterBit(pin, (mode == FALLING), MCP23017_DEFVALA, MCP23017_DEFVALB);

    // enable the pin for interrupt
    updateRegisterBit(pin, HIGH, MCP23017_GPINTENA, MCP23017_GPINTENB);

}
