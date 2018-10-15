#include <mbed.h>
/*
// Part 1 ---------------------------------------------------------------------

DigitalIn pb(p16);
DigitalOut LED(p21);

int main()
{
    while(1){
        if (pb == 0) LED = 1;
        else LED = 0;
    }
}
// End Part 1 -----------------------------------------------------------------
*/

/*
// Part 2 ---------------------------------------------------------------------

InterruptIn pbAdjust(p16);
InterruptIn pbReset(p17);
PwmOut LED(LED1);   // change LED1 to a pin number for external LED

float pwmLevel = 0.0;

void pbAdjust_interrupt(void)
{
    pwmLevel += 0.1;
}

void pbReset_interrupt(void)
{
    pwmLevel -= 0.1;
}

int main()
{
    pbAdjust.mode(PullUp);
    pbReset.mode(PullUp);
    wait(0.01);

    pbAdjust.fall(&pbAdjust_interrupt);
    pbReset.fall(&pbReset_interrupt);

    while(1){
        LED = pwmLevel;
    }
}
// End Part 2 -----------------------------------------------------------------
*/

/*
// Part 3 ---------------------------------------------------------------------

InterruptIn pbUp(p16);
InterruptIn pbDown(p17);

float pwmLevel = 0.2;

class RGBLed{

  private:
    PwmOut _red, _blue, _green;

  public:
    RGBLed(PinName red, PinName green, PinName blue): _red(red), _green(green), _blue(blue)
    {
        _red.period(0.005);
        _green.period(0.005);
        _blue.period(0.005);
    }

    void write(float r, float g, float b)
    {
        _red = r;
        _green = g;
        _blue = b;
    }
};

void pbUp_interrupt(void)
{
    pwmLevel += 0.01;
}

void pbDown_interrupt(void)
{
    pwmLevel -= 0.01;
}

int main()
{
    RGBLed led(p21, p22, p23);

    pbUp.mode(PullUp);
    pbDown.mode(PullUp);

    wait(0.01);

    pbUp.fall(&pbUp_interrupt);
    pbDown.fall(&pbDown_interrupt);

    while(1)
    {
        led.write(pwmLevel, pwmLevel, pwmLevel);
    }
}
// End Part 3 -----------------------------------------------------------------
*/

/*
// Part 4 ---------------------------------------------------------------------

BusOut mbedleds(LED1,LED2,LED3,LED4);
//BusOut/In is faster than multiple DigitalOut/Ins

class Nav_Switch
{
public:
    Nav_Switch(PinName up,PinName down,PinName left,PinName right,PinName fire);
    int read();
//boolean functions to test each switch
    bool up();
    bool down();
    bool left();
    bool right();
    bool fire();
//automatic read on RHS
    operator int ();
//index to any switch array style
    bool operator[](int index) {
        return _pins[index];
    };
private:
    BusIn _pins;

};
Nav_Switch::Nav_Switch (PinName up,PinName down,PinName left,PinName right,PinName fire):
    _pins(up, down, left, right, fire)
{
    _pins.mode(PullUp); //needed if pullups not on board or a bare nav switch is used - delete otherwise
    wait(0.001); //delays just a bit for pullups to pull inputs high
}
inline bool Nav_Switch::up()
{
    return !(_pins[0]);
}
inline bool Nav_Switch::down()
{
    return !(_pins[1]);
}
inline bool Nav_Switch::left()
{
    return !(_pins[2]);
}
inline bool Nav_Switch::right()
{
    return !(_pins[3]);
}
inline bool Nav_Switch::fire()
{
    return !(_pins[4]);
}
inline int Nav_Switch::read()
{
    return _pins.read();
}
inline Nav_Switch::operator int ()
{
    return _pins.read();
}

Nav_Switch myNav( p9, p6, p7, p5, p8); //pin order on Sparkfun breakout

int main()
{
    while(1) {
        //with pullups a button hit is a "0" - "~" inverts data to leds
        mbedleds = ~(myNav & 0x0F); //update leds with nav switch direction inputs
        if(myNav.fire()) mbedleds = 0x0F; //special all leds on case for fire (center button)
        //or use - if(myNav[4]==0) mbedleds = 0x0F; //can index a switch bit like this
        wait(0.02);
    }
}
// End Part 4 -----------------------------------------------------------------
*/

// Part 5: https://os.mbed.com/users/4180_1/code/USBMouse_NavSwitch/

// Part 6: https://os.mbed.com/users/4180_1/notebook/mpr121-i2c-capacitive-touch-sensor/

/*
// Part 7 ---------------------------------------------------------------------
// A simple IO demo using the MCP23S17 library
//
// MCP23S17 Library Copyright (c) 2010 Romilly Cocking
// Released under the MIT License: http://mbed.org/license/mit
//
// See http://mbed.org/users/romilly/notebook/mcp23s17-addressable-16-bit-io-expander-with-spi/
//
//
// MCP23S17 datasheet http://ww1.microchip.com/downloads/en/DeviceDoc/21952b.pdf
// uses MCP23S17 library version 0.4

#include "mbed.h"
#include "MCP23S17.h"
// Create SPI bus
SPI spi(p5, p6, p7);
//
// Wiring Connections:
// mbed p5,p6,p7 are tied to MCP23S17 SI, SO, SCK pins
// mbed p20 to MCP23S17 CS
// MCP23S17 reset pin pulled high
// MCP23S17 GPA0 connected to GPB0 for loopback test
// A0, A1, A2 of the MCP23S17  are tied to ground on the breadboard, so the 8-bit address for writes is 0x40
// This is referred to as the opcode in the device datasheet
char Opcode = 0x40;

// Next create a MCP23S17
// mbed p20 is connected to ~chipSelect on the MCP23S17
MCP23S17 chip = MCP23S17(spi, p20, Opcode);

// Optional software reset - mbed p14 to MCP23S17 reset pin
// DigitalOut reset(p14);

DigitalOut led1(LED1); // mbed LED1 is used for test status display

int main() {
//  The MCP23S17 reset pin can just be pulled high, since it has a power on reset circuit.
//  The reset pin can be used for a software forced reset by pulling it low with an mbed GPIO pin.
//  But just leave it pulled high for this simple demo code.
//  After a power on reset, both IO ports default to input mode
//
//  Here is the optional code for a software reset
//  reset = 0;
//  wait_us(1);
//  reset = 1;
//
//  Set all 8 Port A bits to output direction
    chip.direction(PORT_A, 0x00);
//  Set all 8 Port B bits to input direction
    chip.direction(PORT_B, 0xFF);
    led1=0;
//  Start Loopback test sending out and reading back values
//  loopback test uses A0 and B0 pins - so use a wire to jumper those two pins on MCP23S17 together
    while (1) {
        led1 = chip.read(PORT_B) & 0x01;
        wait(0.1);
        if (led1) chip.write(PORT_A, 0x55);
        else chip.write(PORT_A, 0x00);
    }
}
// End Part 7 -----------------------------------------------------------------
*/

/*
// Assembly XC ---------------------------------------------------------------------

// This program will blink LED1 and LED4
// using assembly language for LED1 and
// API functions for LED4
// declare external assembly language function (in a *.s file)
extern "C" int my_asm(int value);
// declare LED outputs – let C set them up as output bits
DigitalOut myled1(LED1);
DigitalOut myled4(LED4);

DigitalIn pb(p16);
DigitalOut LED(p21);

int main() {
    int value = 0;
    // loop forever
    while(1) {
      myled4 = pb;
      value = myled4;
      //call assembly language function to control LED1
      my_asm(value);
      // flip value and wait
     // value = pb;
//      wait(0.2);
    }
}
// End Assembly XC -----------------------------------------------------------------
*/

/*
// WatchDog XC ---------------------------------------------------------------------
// LEDs used to indicate code activity and reset source
DigitalOut myled1(LED1); //in main loop part 1
DigitalIn pb(p16);
DigitalOut LED(p21);

// Simon's Watchdog code from
// http://mbed.org/forum/mbed/topic/508/
class Watchdog {
public:
// Load timeout value in watchdog timer and enable
    void kick(float s) {
        LPC_WDT->WDCLKSEL = 0x1;                // Set CLK src to PCLK
        uint32_t clk = SystemCoreClock / 16;    // WD has a fixed /4 prescaler, PCLK default is /4
        LPC_WDT->WDTC = s * (float)clk;
        LPC_WDT->WDMOD = 0x3;                   // Enabled and Reset
        kick();
    }
// "kick" or "feed" the dog - reset the watchdog timer
// by writing this required bit pattern
    void kick() {
        LPC_WDT->WDFEED = 0xAA;
        LPC_WDT->WDFEED = 0x55;
    }
};

// Setup the watchdog timer
Watchdog wdt;

int main() {
    int count = 0;

// setup a 10 second timeout on watchdog timer hardware
// needs to be longer than worst case main loop exection time
    wdt.kick(10.0);

// Main program loop - resets watchdog once each loop iteration
// Would typically have a lot of code in loop with many calls
    while (1) {
        if (pb == 0) LED = 1;
        else LED = 0;

        wait(0.05);
// Simulate a fault lock up with an infinite while loop, but only after 25 loop iterations
        if (count == 50) while (1) {myled1 = 1;};
// LED 2 will stay on during the fault
        myled1 = 0;
        count ++;
// End of main loop so "kick" to reset watchdog timer and avoid a reset
        wdt.kick();
    }
}
// End WatchDog XC -----------------------------------------------------------------
*/
