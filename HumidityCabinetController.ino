#include <Wire.h>

#include <TM1637Display.h>
#include "Adafruit_Si7021.h"
//#include <avr/wdt.h>
#include "PID_v1.h"

#define PIN_OUTPUT 10


// Define the display pins
// DISPLAY 1
#define Disp1DIO 2
#define Disp1Clk 3
// DISPLAY 2
#define Disp2DIO 5
#define Disp2Clk 6

// Declare variables
float Humidity;
float Temperature;

// Initialise Sensor
Adafruit_Si7021 sensor = Adafruit_Si7021();

// Initialise Displays
TM1637Display display1(Disp1Clk, Disp1DIO);
TM1637Display display2(Disp2Clk, Disp2DIO);

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=100, Ki=3, Kd=0;
//double Kp=20, Ki=4, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
double start;

void setup() {
  //wdt_enable(WDTO_4S);
  // Turn the displays on
  display1.setBrightness(2);
  display2.setBrightness(2);

  // Start sensor
  sensor.begin();
  start = millis();
    //initialize the variables we're linked to
  Input = sensor.readTemperature();
  Setpoint = 22.0;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

void loop() {
  //wdt_reset();
  
  Humidity = sensor.readHumidity();
  Temperature = sensor.readTemperature();


  // LED indicator on
  //digitalWrite(13, HIGH);

  
  Input = (double)Temperature;
  
  myPID.Compute();
  analogWrite(PIN_OUTPUT, Output);
  //analogWrite(PIN_OUTPUT, 128);

  if(millis() >= start + 1500)
  {
      start = millis();
    //display1.showNumberDec(Temperature);
    display1.showNumberDecEx(round(Temperature*100), (0x80 >> 1), true);
    display2.showNumberDecEx(round(Humidity*100), (0x80 >> 1), true);
      
  }
  
  //wdt_reset();  
  //delay(100);

  // LED indicator off
  //digitalWrite(13, LOW);

  //delay(1400);

}
