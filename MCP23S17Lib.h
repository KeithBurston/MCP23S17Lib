/*
 * Library for MCP23S17 SPI port extender
 * Keith Burston 2017
 */

#include <arduino.h>

#ifndef __MCP23S17Lib
#define __MCP23S17Lib

class MCP23S17
{
    public:

        // create MCP23S17 object and initialise hardware
        // SSPin=0 defaults to SS for Arduino type
		// Can use named constant for those with a tidy mind -  "MCP23S17::UseDefaultSS" or "classInstancename.UseDefaultSS"
		static const byte UseDefaultSS=0;		
        // SS for Arduino type (10 for Uno, 53 for Mega etc.) is always set as output, even if not connected, to prevent Arduino going into SPI slave mode
        MCP23S17();                                                // creates device and disables address pins (ignore A0..2), sets SSpin as default for Arduino type
        MCP23S17(byte SSpin);                                      // creates device and disables address pins (ignore A0..2), sets SSpin
        MCP23S17(byte SSpin, byte addr);                           // creates device and enables address pins (A0..2) and sets address (0-7), sets SSpin

        // configure ports
        //      can set ports by byte or bit, but not word (both ports at once)
        //      bit numbers per port are:

        //      PORTA
        //      bit 0 = pin 21 (ls)
        //      bit 1 = pin 22
        //      bit 2 = pin 23
        //      bit 3 = pin 24
        //      bit 4 = pin 25
        //      bit 5 = pin 26
        //      bit 6 = pin 27
        //      bit 7 = pin 28 (ms)

        // PORTB
        //      bit 0 = pin 1 (ls)
        //      bit 1 = pin 2
        //      bit 2 = pin 3
        //      bit 3 = pin 4
        //      bit 4 = pin 5
        //      bit 5 = pin 6
        //      bit 6 = pin 7
        //      bit 7 = pin 8 (ms)

        //      mode:    0= output, 1= input
        //      invert:  0= no inversion, 1 = inversion (of input bit)
        //      pullup:  0= no pullup enabled, 1=pullup enabled

        void setupPortA(byte mode, byte pullup, byte invert);           // sets up port A, 8 bits
        void setupPortB(byte mode, byte pullup, byte invert);           // sets up port B, 8 bits
        void setupPortA(byte pin, byte mode, byte pullup, byte invert); // sets up port A, 1 bit
        void setupPortB(byte pin, byte mode, byte pullup, byte invert); // sets up port B, 1 bit

        // read ports
        //      read a single port or a pin. Bits for pins set as outputs are invalid
        byte readPortA();                                               // read a byte from port A
        byte readPortA(byte pin);                                       // read a bit from port A
        byte readPortB();                                               // read a byte from port B
        byte readPortB(byte pin);                                       // read a bit from port B

        // write ports
        //      write a single port or bit, Bits for pins set as inputs are ignored
        void writePortA(byte data);                                     // write a byte to port A
        void writePortA(byte pin, byte data);                           // write a bit to port A
        void writePortB(byte data);                                     // write a byte to port B
        void writePortB(byte pin, byte data);                           // write a bit to port A B

        // read/write both ports at once to give 16 bit value. Port A is least significant 8 bits
        unsigned int readAll();                                         // read 16 bits from ports A and B
        void writeAll(unsigned int data);                               // write 16 bits to ports A and B

        // read output latches. Values are last value written, not state of port. Not that useful!
        byte readPortAOutputLatch();                                    // read a byte from port A output latch
        byte readPortBOutputLatch();                                    // read a byte from port B output latch


        // control interrupts

        //      configure which Pins can cause interrupt (on input)
        //      by pin or port
        //      enable: 1=causes interrupt, 0=does not cause interrupt
        //      control: 1=causes interrupt if pin DOES NOT match value in "compare", 0=causes interrupt on any change
        //      compare: value to compare with if "control"=1
        void setInterruptsA(byte enable, byte control, byte compare);               // sets up interrupts on port A
        void setInterruptsB(byte enable, byte control, byte compare);               // sets up interrupts on port B
        void setInterruptsA(byte pin, byte enable, byte control, byte compare);     // sets up interrupts on port A, 1 Pin
        void setInterruptsB(byte pin, byte enable, byte control, byte compare);     // sets up interrupts on port B, 1 Pin


        // configure the interrupt outputs (all arguments are true/false)

        //      mirror: true=both interrupt outputs are identical, false=each output corresponds to its respective port only
        //      polarity: true=active high, false =active low
        //      oDrain: true=set as open drain (ignore polarity), false=set as normal output (uses polarity)
        void configInterrupts(byte mirror, byte polarity, byte oDrain);// configure interrupt outputs

        // handle interrupts

        //      get interrupt flags, 1=pin causes interrupt. Clears interrupt
        byte getInterruptFlagA();                                       // returns interrupt flags for port A
        byte getInterruptFlagB();                                       // returns interrupt flags for port B

        //      get pin state, reads value of port latched when interrupt occured
        byte getInterruptCaptureA();                                      // get latched port A data at time of interrupt
        byte getInterruptCaptureB();                                      // get latched port B data at time of interrupt
		


    private:
        byte SSline;        // select line to use
        byte address;       // address of this device (0-7). Not read, reference only?
        // these are derived from above to shorten SPI sequence
        byte readAddress;   // spi command to read from this address
        byte writeAddress;  // spi command to write to this address

        byte controlReg;    // copy of control register

        byte copyAMode = 0x00, copyAInvert = 0x00, copyAPullup = 0x00;          // used to set single port bits quickly
        byte copyBMode = 0x00, copyBInvert = 0x00, copyBPullup = 0x00;          // used to set single port bits quickly

        byte copyAIEnable = 0x00, copyAIControl = 0x00, copyAIPolarity = 0x00;  // used to set up single interrupt control bit
        byte copyBIEnable = 0x00, copyBIControl = 0x00, copyBIPolarity = 0x00;  // used to set up single interrupt control bit
        byte copyAData = 0x00, copyBData = 0x00;                                // last data written to port

        void writeReg(byte reg, byte val);                  // write 8 bit register by address
        byte readReg(byte reg);                             // read 8 bit register by address

        byte setBit(byte orig, byte val, byte bit);         // copies bit 0 of val to bit "bit" of orig and returns result

        void initDev(byte SSpin, byte addr);                // initialise the device and object

        // register numbers as constants
        // configured with BANK=0 in control register to make register addresses alternate between ports
        static const byte RegIODIRA    = 0x00;   // Port A Direction Register
        static const byte RegIODIRB    = 0x01;   // Port B Direction Register
        static const byte RegIPOLA     = 0x02;   // Port A Input Polarity Register
        static const byte RegIPOLB     = 0x03;   // Port B Input Polarity Register
        static const byte RegGPINTENA  = 0x04;   // Port A Interrupt on Change Pin Assignments
        static const byte RegGPINTENB  = 0x05;   // Port B Interrupt on Change Pin Assignments
        static const byte RegDEFVALA   = 0x06;   // Port A Default Compare Register for Interrupt on Change
        static const byte RegDEFVALB   = 0x07;   // Port B Default Compare Register for Interrupt on Change
        static const byte RegINTCONA   = 0x08;   // Port A Interrupt on Change Control Register
        static const byte RegINTCONB   = 0x09;   // Port B Interrupt on Change Control Register
        static const byte RegIOCON     = 0x0A;   // Configuration Register (also at 0xB)
        static const byte RegGPPUA     = 0x0C;   // Port A Pull-Up Resistor
        static const byte RegGPPUB     = 0x0D;   // Port B Pull-Up Resistor
        static const byte RegINTFA     = 0x0E;   // Port A Interrupt Flag Register
        static const byte RegINTFB     = 0x0F;   // Port B Interrupt Flag Register
        static const byte RegINTCAPA   = 0x10;   // Port A Interrupt Captured Value Register
        static const byte RegINTCAPB   = 0x11;   // Port B Interrupt Captured Value Register
        static const byte RegGPIOA     = 0x12;   // Port A GPIO Port Register
        static const byte RegGPIOB     = 0x13;   // Port B GPIO Port Register
        static const byte RegOLATA     = 0x14;   // Port A Output Latch Register
        static const byte RegOLATB     = 0x15;   // Port B Output Latch Register

        // config settings for RegIOCON
        static const byte ConfAddEn = 0b00001000;      // use address pins
        static const byte ConfAddDisEn = 0b00000000;   // dont use address pins

};


#endif
