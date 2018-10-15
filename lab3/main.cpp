// Lidar theremin
/*
#include "mbed.h"
#include "XNucleo53L0A1.h"
#include <stdio.h>
#include "Speaker.h"
Serial pc(USBTX,USBRX);
DigitalOut shdn(p26);
// This VL53L0X board test application performs a range measurement in polling mode
// Use 3.3(Vout) for Vin, p28 for SDA, p27 for SCL, P26 for shdn on mbed LPC1768

//I2C sensor pins
#define VL53L0_I2C_SDA   p28
#define VL53L0_I2C_SCL   p27

static XNucleo53L0A1 *board=NULL;
Speaker mySpeaker(p21);
                                
int main()
{
    int status;
    uint32_t distance;
    DevI2C *device_i2c = new DevI2C(VL53L0_I2C_SDA, VL53L0_I2C_SCL);
    // creates the 53L0A1 expansion board singleton obj
    board = XNucleo53L0A1::instance(device_i2c, A2, D8, D2);
    shdn = 0; //must reset sensor for an mbed reset to work
    wait(0.1);
    shdn = 1;
    wait(0.1);
    // init the 53L0A1 board with default values
    status = board->init_board();
    while (status) {
        pc.printf("Failed to init board! \r\n");
        status = board->init_board();
    }
    //loop taking and printing distance
    while (1) {
        status = board->sensor_centre->get_distance(&distance);
        if (status == VL53L0X_ERROR_NONE) {
            pc.printf("D=%ld mm\r\n", distance);
            mySpeaker.PlayNote(2*distance, 0.05, 0.5);
        }
        
    }
}
*/

// ticker demo
/* 
#include "mbed.h"

Ticker flipper1;
Ticker flipper2;
Ticker flipper3;
Ticker flipper4;

DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);

void flip1()
{
    led1 = !led1;
}

void flip2()
{
    led2 = !led2;
}

void flip3()
{
    led3 = !led3;
}

void flip4()
{
    led4 = !led4;
}

int main()
{
    led1 = 0;
    led2 = 0;
    led3 = 0;
    led4 = 0;
    
    flipper1.attach(&flip1, 1.0);
    flipper2.attach(&flip2, 2.0);
    flipper3.attach(&flip3, 4.0);
    flipper4.attach(&flip4, 8.0);

    while(1)
    {}
}
*/

// bluetooth demo
/*
#include "mbed.h"

RawSerial  pc(USBTX, USBRX);
RawSerial  dev(p28,p27); // Adafruit BLE
DigitalOut led1(LED1);
DigitalOut led4(LED4);

void dev_recv()
{
    led1 = !led1;
    while(dev.readable()) {
        pc.putc(dev.getc());
    }
}

void pc_recv()
{
    led4 = !led4;
    while(pc.readable()) {
        dev.putc(pc.getc());
    }
}

int main()
{
    pc.baud(9600);
    dev.baud(9600);

    pc.attach(&pc_recv, Serial::RxIrq);
    dev.attach(&dev_recv, Serial::RxIrq);

    while(1) {
        sleep();
    }
}
*/
