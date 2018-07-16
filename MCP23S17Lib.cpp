/*
 * Library for MCP23S17 SPI port extender
 * Keith Burston 2017
 */

#include <arduino.h>
#include <SPI.h>
#include <MCP23S17Lib.h>


// private
inline void MCP23S17::writeReg(byte reg, byte val)
{
    // write 8 bit register by address
    digitalWrite(SSline, LOW);
    SPI.transfer(writeAddress); // variable includes address, write code
    SPI.transfer(reg);
    SPI.transfer(val);
    digitalWrite(SSline, HIGH);
}

inline byte MCP23S17::readReg(byte reg)
{
    // read 8 bit register by address
    byte val;
    digitalWrite(SSline, LOW);
    SPI.transfer(readAddress); // variable includes address, read code
    SPI.transfer(reg);
    val = SPI.transfer(0);
    digitalWrite(SSline, HIGH);
    return val;
}

inline byte MCP23S17::setBit(byte orig, byte val, byte bit)
{
    // copies bit 0 of val to bit "bit" of orig and returns result
    return (orig & ~(0x01 << bit)) | ((val & 0x01) << bit);
}

void MCP23S17::initDev(byte SSpin, byte addr)
{
    // SSPin=0 defaults to SS for Arduino type
    if (SSpin == 0)
    {
        SSline = SS;
    }
    else
    {
        SSline = SSpin;         // remember it
    }
    
    address = addr;
    readAddress = 0x41 | (addr << 1);
    writeAddress = 0x40 | (addr << 1);  // address is 0b0100AAAR, A=A0..2, R=R/w bit

    pinMode(SSline, OUTPUT);    // make sure select  pin is configured, if not default
    digitalWrite(SSline, HIGH);

    SPI.begin();
    pinMode(MISO, INPUT);
    writeReg(RegIOCON, controlReg);

    copyAMode = 0xFF;
    copyAInvert = copyAPullup = 0x00;
    copyBMode = 0XFF;
    copyBInvert = copyBPullup = 0x00;
    copyAIEnable = copyAIControl = copyAIPolarity = 0x00;
    copyBIEnable = copyBIControl = copyBIPolarity = 0x00;
}

// public
MCP23S17::MCP23S17()
{
    // creates device and disables address pins (ignore A0..2), sets SSpin to default

    controlReg = ConfAddDisEn;       // set as disable register addressing
    initDev(0,0);
}


MCP23S17::MCP23S17(byte SSpin)
{
    // creates device and disables address pins (ignore A0..2)

    controlReg = ConfAddDisEn;       // set as disable register addressing
    initDev(SSpin,0);
}


MCP23S17::MCP23S17(byte SSpin, byte addr)
{
    controlReg = ConfAddEn;             // set as enable register addressing
    initDev(SSpin,addr & 0x7);
}

void MCP23S17::setupPortA(byte mode, byte pullup, byte invert)
{
    // sets up port A, 8 bits
    copyAMode = mode;
    copyAPullup = pullup;
    copyAInvert = invert;
    writeReg(RegIODIRA, copyAMode);
    writeReg(RegGPPUA, copyAPullup);
    writeReg(RegIPOLA, copyAInvert);
}


void MCP23S17::setupPortB(byte mode, byte pullup, byte invert)
{
    // sets up port B, 8 bits
    copyBMode = mode;
    copyBPullup = pullup;
    copyBInvert = invert;
    writeReg(RegIODIRB, copyBMode);
    writeReg(RegGPPUB, copyBPullup);
    writeReg(RegIPOLB, copyBInvert);


}


void MCP23S17::setupPortA(byte pin, byte mode, byte pullup, byte invert)
{
    // sets up port A, 1 bit
    if (pin > 7)
    {
        return;
    }

    copyAMode = setBit(copyAMode, mode, pin);
    copyAPullup = setBit(copyAPullup, pullup, pin);
    copyAInvert = setBit(copyAInvert, invert, pin);

    writeReg(RegIODIRA, copyAMode);
    writeReg(RegGPPUA, copyAPullup);
    writeReg(RegIPOLA, copyAInvert);
}


void MCP23S17::setupPortB(byte pin, byte mode, byte pullup, byte invert)
{
    // sets up port B, 1 bit
    if (pin > 7)
    {
        return;
    }

    copyBMode = setBit(copyBMode, mode, pin);
    copyBPullup = setBit(copyBPullup, pullup, pin);
    copyBInvert = setBit(copyBInvert, invert, pin);

    writeReg(RegIODIRB, copyBMode);
    writeReg(RegGPPUB, copyBPullup);
    writeReg(RegIPOLB, copyBInvert);
}



byte MCP23S17::readPortA()
{
    // read a byte from port A
    return readReg(RegGPIOA);
}


byte MCP23S17::readPortA(byte pin)
{
    // read a bit from port A
    if (pin > 7)
    {
        return 0;
    }
    return (readReg(RegGPIOA) >> pin) & 0x01;
}


byte MCP23S17::readPortB()
{
    // read a byte from port B
    return readReg(RegGPIOB);
}


byte MCP23S17::readPortB(byte pin)
{
    // read a bit from port B
    if (pin > 7)
    {
        return 0;
    }
    return (readReg(RegGPIOB) >> pin) & 0x01;
}


void MCP23S17::writePortA(byte data)
{
    // write a byte to port A

    copyAData = data;
    writeReg(RegGPIOA, copyAData);
}


void MCP23S17::writePortA(byte pin, byte data)
{
    // write a bit to port A

    copyAData = setBit(copyAData, data, pin);
    writeReg(RegGPIOA, copyAData);
}


void MCP23S17::writePortB(byte data)
{
    // write a byte to port B

    copyBData = data;
    writeReg(RegGPIOB, copyBData);
}


void MCP23S17::writePortB(byte pin, byte data)
{
    // write a bit to port A B
    copyBData = setBit(copyBData, data, pin);
    writeReg(RegGPIOB, copyBData);
}


unsigned int MCP23S17::readAll()
{
    // read 16 bits from ports A and B
    return (readReg(RegGPIOB) << 8) + readReg(RegGPIOA);

}


void MCP23S17::writeAll(unsigned int data)
{
    // write 16 bits to ports A and B
    copyAData = data & 0xff;
    copyBData = (data >> 8) & 0xff;
    writeReg(RegGPIOA, copyAData);
    writeReg(RegGPIOB, copyBData);
}


void MCP23S17::setInterruptsA(byte enable, byte control, byte compare)
{
    // sets up interrupts on port A
    copyAIEnable = enable;
    copyAIControl = control;
    copyAIPolarity = compare;
    writeReg(RegGPINTENA, enable);
    writeReg(RegINTCONA, control);
    writeReg(RegDEFVALA, compare);
}


void MCP23S17::setInterruptsB(byte enable, byte control, byte compare)
{
    // sets up interrupts on port B
    copyBIEnable = enable;
    copyBIControl = control;
    copyBIPolarity = compare;
    writeReg(RegGPINTENB, enable);
    writeReg(RegINTCONB, control);
    writeReg(RegDEFVALB, compare);
}


void MCP23S17::setInterruptsA(byte pin, byte enable, byte control, byte compare)
{
    // sets up interrupts on port A, 1 Pin
    copyAIEnable = setBit(copyAIEnable, enable, pin);
    copyAIControl = setBit(copyAIControl, control, pin);
    copyAIPolarity = setBit(copyAIPolarity, compare, pin);
    writeReg(RegGPINTENA, copyAIEnable);
    writeReg(RegINTCONA, copyAIControl);
    writeReg(RegDEFVALA, copyAIPolarity);
}


void MCP23S17::setInterruptsB(byte pin, byte enable, byte control, byte compare)
{
    // sets up interrupts on port B, 1 Pin
    copyBIEnable = setBit(copyBIEnable, enable, pin);
    copyBIControl = setBit(copyBIControl, control, pin);
    copyBIPolarity = setBit(copyBIPolarity, compare, pin);
    writeReg(RegGPINTENB, copyBIEnable);
    writeReg(RegINTCONB, copyBIControl);
    writeReg(RegDEFVALB, copyBIPolarity);
}


void MCP23S17::configInterrupts(byte mirror, byte polarity, byte oDrain)
{
    // configure interrupt outputs
    controlReg = (controlReg & 10111001) | ((mirror & 0x01) << 6) | ((polarity & 0x01) << 1) | ((oDrain & 0x01) << 1);
    writeReg(RegIOCON, controlReg);
}


byte MCP23S17::getInterruptFlagA()
{
    // returns interrupt flags for port A
    return readReg(RegINTFA);
}


byte MCP23S17::getInterruptFlagB()
{
    // returns interrupt flags for port B
    return readReg(RegINTFB);
}


byte MCP23S17::getInterruptCaptureA()
{
    // get latched port A data at time of interrupt
    return readReg(RegINTCAPA);
}


byte MCP23S17::getInterruptCaptureB()
{
    // get latched port B data at time of interrupt
    return readReg(RegINTCAPB);
}

byte MCP23S17::readPortAOutputLatch()
{
    // read a byte from port A output latch
    return readReg(RegOLATA);
}

byte MCP23S17::readPortBOutputLatch()
{
    // read a byte from port B output latch
    return readReg(RegOLATB);
}


