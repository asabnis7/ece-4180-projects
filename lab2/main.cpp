#include <mbed.h>
Serial pc(USBTX, USBRX);
/*
// Part 3 --------------------------------------------------------
#include <mbed.h>
#include <iostream>
#include "LSM9DS1.h"
#include "uLCD_4DGL.h"
#define PI 3.14159
#define DECLINATION -4.94 // Declination (degrees) in Atlanta,GA.

// LCD Display
uLCD_4DGL uLCD(p28, p27, p30); // serial tx, serial rx, reset pin;

// LSM9DS1
DigitalOut myled(LED1);
Serial pc(USBTX, USBRX);

int main() {

    // set baudrate
    //uLCD.baudrate(3000000);
    //wait(0.2);

    uLCD.cls();

    // main loop
    LSM9DS1 IMU(p9, p10, 0xD6, 0x3C);
    IMU.begin();
    if (!IMU.begin()) {
        pc.printf("Failed to communicate with LSM9DS1.\n");
    }
    IMU.calibrate(1);

    float c_x = 64;
    float c_y = 64;
    float x = 0;
    float y = 0;
    float dt = 0.1;
    while (1) {
        uLCD.circle(c_x, c_y, 50, 0xFF00FF);
        uLCD.filled_circle(c_x + x, c_y - y, 10, 0x000000);

        while(!IMU.accelAvailable());
        IMU.readAccel();

        x = (x + IMU.calcAccel(IMU.ax) * 5);
        y = (y - IMU.calcAccel(IMU.ay) * 5);

        if (x >= 40) x = 40;
        else if (x <= -40) x = -40;

        if (y >= 40) y = 40;
        else if (y <= -40) y = -40;
        //uLCD.printf("%f, %f | %f, %f", IMU.calcAccel(IMU.ax), IMU.calcAccel(IMU.ay), x, y);

        uLCD.filled_circle(c_x + x, c_y - y, 10, 0xFF00FF);
        wait(dt);
    }
}
// End Part 3 --------------------------------------------------------
*/

/*
// Part 3 Extra Credit -----------------------------------------------------------
// Formula to convert compass bearing to LCD coordinates
void xyCalc(float heading, float &x1, float &y1)
{
    heading *= (2.0*PI)/360.0;
    float xC = 30.0*sin(heading);
    float yC = -30.0*cos(heading);
    heading *= 360.0/(2.0*PI);
    if (heading > 0.0 && heading <= 90.0) {x1 = 64.0 + xC; y1 = 64.0 + yC;}
    else if (heading > 90.0 && heading <= 180.0) {x1 = 64.0 + xC; y1 = 64.0 + yC;}
    else if (heading > 180.0 && heading <= 270.0) {x1 = 64.0 + xC; y1 = 64.0 + yC;}
    else {x1 = 64.0 + xC; y1 = 64.0 + yC;}
}    
    // add to main after getting heading
    xyCalc(heading, x1, y1);
    lcd.printf("Heading: %5f degrees", heading);
    lcd.circle(64,64,5, 'BLUE');
    lcd.line(x1, y1, x2, y2, 'RED');

// End Part 3 Extra Credit ----------------------------------------------------
*/

/*
// Test Ethernet Connection--------------------------------------------------------
Ethernet eth;

int main() {
    char buf[0x600];

    while(1) {
        int size = eth.receive();
        if(size > 0) {
            eth.read(buf, size);
            printf("Destination:  %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                    buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
            printf("Source: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                    buf[6], buf[7], buf[8], buf[9], buf[10], buf[11]);
        }

        wait(1);
    }
}
// End Test Ethernet Connection--------------------------------------------------------
*/

/*
// Part 4 --------------------------------------------------------
#include "mbed.h"
#include "EthernetInterface.h"
#include "HTTPClient.h"

Serial pc(USBTX, USBRX);
HTTPClient http;
char str[2048];

int main()
{
    EthernetInterface eth;
    printf("HTTP GET REQUEST\r\n");
    printf("Init: %d\r\n", eth.init()); //Use DHCP

    printf("Connect: %d\r\n", eth.connect());

    //GET data
    printf("\nTrying to fetch page...\r\n");
    int ret = http.get("http://www.omdbapi.com/?t=rashomon&apikey=f06cbcb4", str, 1028);
    if (!ret)
    {
      printf("Page fetched successfully - read %d characters\r\n", strlen(str));
      printf("Result: %s\r\n", str);
    }
    else
    {
      printf("Error - ret = %d - HTTP return code = %d\r\n", ret, http.getHTTPResponseCode());
    }

    eth.disconnect();

    while(1) {
    }
}
// End Part 4 --------------------------------------------------------
*/

// Microphone demo --------------------------------------------------------
/*
#include "mbed.h"
Serial pc(p28, p27);
//Adafruit MEMs SPW2430 microphone demo with audio output - the "mPA"
BusOut myleds(LED1,LED2,LED3,LED4);
AnalogOut speaker(p18);
//also setting any unused analog input pins to digital outputs reduces A/D noise a bit
//see http://mbed.org/users/chris/notebook/Getting-best-ADC-performance/
DigitalOut P15(p15);
DigitalOut P16(p16);
DigitalOut P19(p19);
DigitalOut P20(p20);
class microphone
{
public :
    microphone(PinName pin);
    float read();
    operator float ();
private :
    AnalogIn _pin;
};
microphone::microphone (PinName pin):
    _pin(pin)
{
}
float microphone::read()
{
    return _pin.read();
}
inline microphone::operator float ()
{
    return _pin.read();
}

microphone mymicrophone(p16);

int main()
{
    float sample;
    float average = 0.67/3.3;//initial DC bias level
    while(1) {
//read in sample value
        sample = mymicrophone;
//subtract 0.67V DC bias - but it varies quite a bit after loud or long sounds
        average = 0.9999*average + 0.0001*sample;//try to slowly auto adjust the DC bias level
        speaker = 0.5 +((sample - average)*33.0);//scale up to 0.0 to 1.0 for speaker output
        myleds = int(abs((sample - average)*300.0)); //scale to around 15 for LEDs
//No faster than a 16kHz audio sample rate;
        wait(1.0/16000.0);
    }
}
 */ 
// End microphone demo --------------------------------------------------------

// Lidar demo --------------------------------------------------------
/*
 #include "mbed.h"
#include "XNucleo53L0A1.h"
#include <stdio.h>
Serial pc(USBTX,USBRX);
DigitalOut shdn(p26);
// This VL53L0X board test application performs a range measurement in polling mode
// Use 3.3(Vout) for Vin, p28 for SDA, p27 for SCL, P26 for shdn on mbed LPC1768

//I2C sensor pins
#define VL53L0_I2C_SDA   p28
#define VL53L0_I2C_SCL   p27

static XNucleo53L0A1 *board=NULL;

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
        }
    }
}
*/
// End lidar demo --------------------------------------------------------

// SD system demo --------------------------------------------------------
/*
#include "mbed.h"
#include "SDFileSystem.h"

SDFileSystem sd(p5, p6, p7, p8, "sd"); // the pinout on the mbed Cool Components workshop board
Serial pc(USBTX,USBRX);

char buffer[20];

int main()
{
    pc.printf("Hello World!\n\r");

    mkdir("/sd/dir", 0777);

    pc.printf("Directory made\n\r");

    FILE *fp = fopen("/sd/dir/sdtest.txt", "w");
    if(fp == NULL) {
        pc.printf("Could not open file for write\n\r");
    } else {
        pc.printf("Connected for write\n\r");
        fprintf(fp, "Hello SD file world");
    }
    fclose(fp);

    fp = fopen("/sd/dir/sdtest.txt", "r");
    if(fp == NULL) {
        pc.printf("Could not open file for read\n\r");
    } else {
        pc.printf("Connected for read\n\r");
        while (!feof(fp)) {
            fgets(buffer, 64, fp);
            pc.printf("%s",buffer);
        }
    }

    fclose(fp);
}
*/
// End SD system demo --------------------------------------------------------


// wave player demo --------------------------------------------------------
#include "mbed.h"
#include "SDFileSystem.h"
#include "wave_player.h"


SDFileSystem sd(p5, p6, p7, p8, "sd"); //SD card
Serial pc(USBTX, USBRX);
AnalogOut DACout(p18);

wave_player waver(&DACout);

int main()
{
    FILE *wave_file;
    pc.printf("\n\n\nHello, wave world!\n");
    wave_file = fopen("/sd/mydir/galaga1.wav","r");
    if(wave_file == NULL) {
        pc.printf("Could not open file for read\n\r");
    } else {
        pc.printf("Found .wav file\n\r");
        waver.play(wave_file);
    }
    fclose(wave_file);
}
// End wave player demo --------------------------------------------------------

// solenoid extra credit --------------------------------------------------------
/*
#include "mbed.h"
Serial pc(USBTX, USBRX);
 
DigitalOut myled(LED1);
DigitalOut Ctrl(p21); //Solenoid output control bit
 
int main() {
    while(1) {
        Ctrl = 1; //ON
        myled = 1;
        wait(0.5); 
        Ctrl = 0; //OFF
        myled = 0;
        wait(2);
    }
}
*/
// end solenoid extra credit --------------------------------------------------------


// RTC extra credit --------------------------------------------------------
/*
#include <mbed.h>
#include <uLCD_4DGL.h>
Serial pc(USBTX, USBRX);

uLCD_4DGL lcd(p28, p27, p30);
char timestamp[32];

int main()
{
    set_time(1537545730); 
    lcd.baudrate(3000000);
    
    while (1)
    {
        time_t seconds = time(NULL);
        lcd.cls();
        lcd.text_width(2);
        lcd.text_height(2);
        strftime(timestamp, 32, "%I:%M:%S %p\n", localtime(&seconds));
        lcd.printf(timestamp);
        wait(0.2);
    }
}
*/
// RTC extra credit --------------------------------------------------------
