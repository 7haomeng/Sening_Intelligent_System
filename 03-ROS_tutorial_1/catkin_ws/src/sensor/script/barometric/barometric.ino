/* 
   Use BMP085 to measure pressure and temperature
   Output format:
   pressure temperature
   Edited by Sean
   Last edited: 2018/09/20
   Changelog:
    2018/05/04: First version
    2018/08/09: Add output temperature
    2018/09/20: Change baudrate to 115200
                Revise delay to 0.1s
*/
//   Refer to: 
// https://github.com/adafruit/Adafruit_BMP085_Unified/blob/master/examples/sensorapi/sensorapi.pde
#include <Wire.h>
// Download Adafruit_Sensor.h here
// https://github.com/adafruit/Adafruit_Sensor
#include <Adafruit_Sensor.h>
// https://github.com/adafruit/Adafruit_BMP085_Unified
/*********************************************************
 * Don't forget to include libraries to your Arduino IDE * 
 *********************************************************/
#include <Adafruit_BMP085_U.h>

// Sensor id: 10085
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

/**********************
 Connect it up
 VDD <-> 3.3V (Not 5V!)
 GND <-> GND
 SCL <-> A5
 SDA <-> A4
 ***********************/
 
void setup() {
  Serial.begin(115200);
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
}

void loop() {
  sensors_event_t event;
  bmp.getEvent(&event);
  if(event.pressure)
  {
    Serial.print(event.pressure); // Unit: hPa
    Serial.print(" ");
    float temperature;
    bmp.getTemperature(&temperature);
    Serial.println(temperature); // Unit: Celcius
  }
  else
  {
    Serial.println("Sensor error!");
  }
  delay(100);
}
