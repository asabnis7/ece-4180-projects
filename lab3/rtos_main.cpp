#include "mbed.h"
#include "rtos.h"
#include "uLCD_4DGL.h"
#include "SDFileSystem.h"
#include "wave_player.h"
#include <string>

RawSerial  pc(USBTX, USBRX); // computer
RawSerial  dev(p13,p14); // tx, rx - Adafruit BLE

// set up objects
uLCD_4DGL lcd(p28, p27, p30); // tx, rx, rst
SDFileSystem sd(p5, p6, p7, p8, "sd"); //SD card
AnalogOut DACout(p18);
wave_player wav(&DACout); // sound
DigitalOut led1(LED1); // is working?
DigitalOut led2(LED2); // is working?
char buffer[4] = {0,0,0,0};

class RGBLed
{
private:
    PwmOut _red, _green, _blue;
public:
    RGBLed(PinName red, PinName green, PinName blue):
        _red(red),
        _green(green),
        _blue(blue) {
        _red.period(0.001);
    }
    void write(float r, float g, float b) {
        _red = r;
        _green = g;
        _blue = b;
    }
};

RGBLed rgb(p23, p22, p21); // r, g, b
int red, green, blue = 0;
int idx = 0;

Thread th1;
Thread th2;
Thread th3;
Thread th4;
Mutex lcd_mutex; // mutex for lcd
Mutex print_mutex; // mutex for serial


// function threads

void dev_read()
{
    while(dev.readable()) {
        print_mutex.lock();
        buffer[idx] = dev.getc();
        Thread::wait(100);
        print_mutex.unlock();
    }
    idx++;
    if (idx > 4) {
        idx = 0;
        //strcpy(buffer, "0000");
    }
}

void rgb_thread()
{
    while(1) {
        if (buffer[0] == 'l') {
            red = atoi(&buffer[1]);
            green = atoi(&buffer[2]);
            blue = atoi(&buffer[3]);
        }
        rgb.write(red, green, blue);
        Thread::wait(500);
        rgb.write(0.0, 0.0, 0.0);
        Thread::wait(500);
    }
}

void wav_thread()
{
    FILE *wave_file;
    while(1) {
        if (buffer[0] == 's') {
            wave_file = fopen("/sd/mydir/galaga1.wav","r");
            if(wave_file == NULL) {
                pc.printf("Could not open file for read\n\r");
            } else {
                pc.printf("Found .wav file\n\r");
                wav.play(wave_file);
            }
            fclose(wave_file);
        }
    }
}

void lcd_thread1()
{
    int rad = 5;
    while(1) {
        if (buffer[0] == 'm') rad = 10;
        if (buffer[0] == 'n') rad = 15;
        lcd_mutex.lock();
        lcd.cls();
        lcd.filled_circle(64, 64, rad, BLUE);
        lcd_mutex.unlock();
        Thread::wait(1000);
    }
}

void lcd_thread2()
{
    while(1) {
        lcd_mutex.lock();
        Thread::wait(500);
        lcd.circle(20,20,5, GREEN);
        lcd.circle(104,104,5, RED);
        lcd.circle(20,104,5, GREEN);
        lcd.circle(104,20,5, RED);
        lcd_mutex.unlock();
        Thread::wait(500);
    }
}

int main()
{
    pc.baud(9600);
    dev.baud(9600);
    dev.attach(&dev_read, Serial::RxIrq);

    th1.start(rgb_thread);
    th2.start(wav_thread);
    th3.start(lcd_thread1);
    th4.start(lcd_thread2);

    while (true) {
        led1 = !led1;
        Thread::wait(500);
        print_mutex.lock();
        pc.printf(buffer);
        pc.printf("\n\r");
        print_mutex.unlock();
    }
}
