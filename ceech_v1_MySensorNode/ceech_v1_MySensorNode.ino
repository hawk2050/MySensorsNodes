/*
* devduino-temp-hum-sensor.ino - Firmware for DevDuino v2.0 based temperature and humidity sensor Node with nRF24L01+ module
*
* Copyright 2014 Tomas Hozza <thozza@gmail.com>
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, see <http://www.gnu.org/licenses/>.
*
* Authors:
* Tomas Hozza <thozza@gmail.com>
*
* MySensors library - http://www.mysensors.org/
* DevDuino v2.0 - http://www.seeedstudio.com/wiki/DevDuino_Sensor_Node_V2.0_(ATmega_328)
* nRF24L01+ spec - https://www.sparkfun.com/datasheets/Wireless/Nordic/nRF24L01P_Product_Specification_1_0.pdf
*
Hardware Connections (Breakoutboard to Arduino):
 -VCC = 3.3V
 -GND = GND
 -SDA = A4 (use inline 10k resistor if your board is 5V)
 -SCL = A5 (use inline 10k resistor if your board is 5V)

Ceech Board v1 Compatible with Arduino PRO 3.3V@8MHz

 */
#include <MyMessage.h>
#include <MySensor.h>
#include <SPI.h>
#include <stdint.h>
#include <math.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>  

#include <Wire.h>
#include "HTU21D.h"

// FORCE_TRANSMIT_INTERVAL, this number of times of wakeup, the sensor is forced to report all values to 
// the controller
#define FORCE_TRANSMIT_INTERVAL 3 
#define SLEEP_TIME 5000

#define NODE_ID 7

#define DEBUG 1

#define LIGHT_LEVEL_ENABLE  0
#define DALLAS_ENABLE       1
#define HTU21D_ENABLE       0
#define DHT_ENABLE          1

#define CHILD_ID_HUMIDITY 5
#define CHILD_ID_TEMPA 4
#define CHILD_ID_TEMPB 6
#define CHILD_ID_VOLTAGE 3

#define CHILD_ID_LIGHT 0

/***********************************/
/********* PIN DEFINITIONS *********/
/***********************************/
#define LED_pin 9
#define LIGHT_SENSOR_ANALOG_PIN A0
// Data wire is plugged into port 3 on the Arduino
#define HUMIDITY_SENSOR_DIGITAL_PIN 2

#define ONE_WIRE_BUS 5
#define RF24_CE_pin 7
#define RF24_CS_pin 8

/*****************************/
/********* FUNCTIONS *********/
/*****************************/
#if DHT_ENABLE
DHT dht;
MyMessage msgHum(CHILD_ID_HUMIDITY, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMPA, V_TEMP);
#endif

#if HTU21D_ENABLE
//Create an instance of the object
HTU21D myHumidity;
MyMessage msgHum(CHILD_ID_HUMIDITY, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMPA, V_TEMP);
#endif

#if LIGHT_LEVEL_ENABLE
void readLDRLightLevel(bool force);
MyMessage msgLightLevel(CHILD_ID_LIGHT, V_LIGHT_LEVEL);
#endif


uint16_t measureBattery(bool force);
MyMessage msgVolt(CHILD_ID_VOLTAGE, V_VOLTAGE); 

uint8_t getBatteryPercent();
uint16_t readVcc();
void switchClock(unsigned char clk);
bool highfreq = true;

float lastTemperature;
boolean receivedConfig = false;
boolean metric = true; 
uint8_t loopCount = 0;
/************************************/
/********* GLOBAL VARIABLES *********/
/************************************/
MySensor node(RF24_CE_pin, RF24_CS_pin);


#if DALLAS_ENABLE
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature dallas_sensor(&oneWire);

// arrays to hold device address
DeviceAddress garageThermometer, freezerThermometer;

void readDS18B20(DeviceAddress deviceAddress,bool force);
MyMessage msgDallasTempGarage(CHILD_ID_TEMPA, V_TEMP);
MyMessage msgDallasTempFreezer(CHILD_ID_TEMPB, V_TEMP);

#endif
/**********************************/
/********* IMPLEMENTATION *********/
/**********************************/
void setup()
{
  // start serial port
  Serial.begin(9600);
  /*
  ** Auto Node numbering
  node.begin();
  */
  #ifdef NODE_ID
  node.begin(NULL,NODE_ID,false);
  #else
  node.begin(NULL,AUTO,false);
  #endif
  
  analogReference(INTERNAL);
  node.sendSketchInfo("ceechv1-temp-hum", "0.4");
  
  node.present(CHILD_ID_VOLTAGE, S_CUSTOM);
  // Register all sensors to gateway (they will be created as child devices)

#if DALLAS_ENABLE
  dallas_sensor.begin();
  node.present(CHILD_ID_TEMPA, S_TEMP);
  node.present(CHILD_ID_TEMPB, S_TEMP);
  
  // locate devices on the bus
  
  Serial.print("Locating devices...");
  Serial.print("Found ");
  Serial.print(dallas_sensor.getDeviceCount(), DEC);
  Serial.println(" devices.");
  
  // search for devices on the bus and assign based on an index.  ideally,
  // you would do this to initially discover addresses on the bus and then 
  // use those addresses and manually assign them (see above) once you know 
  // the devices on your bus (and assuming they don't change).
  // 
  // method 1: by index
  if (!sensors.getAddress(garageThermometer, 0))
 {
   Serial.println("Unable to find address for Device 0"); 
 }
 
  if (!sensors.getAddress(freezerThermometer, 1))
 {
   Serial.println("Unable to find address for Device 1"); 
 }

  // set the resolution to 9 bit
  dallas_sensor.setResolution(garageThermometer, 12);
  dallas_sensor.setResolution(freezerThermometer, 9);
#endif

#if LIGHT_LEVEL_ENABLE
  node.present(CHILD_ID_LIGHT, S_LIGHT_LEVEL);
#endif

#if HTU21D_ENABLE
  myHumidity.begin();
  node.present(CHILD_ID_HUMIDITY, S_HUM);
  node.present(CHILD_ID_TEMPA, S_TEMP);
#endif

#if DHT_ENABLE
  dht.setup(HUMIDITY_SENSOR_DIGITAL_PIN); 
  node.present(CHILD_ID_HUMIDITY, S_HUM);
  node.present(CHILD_ID_TEMPA, S_TEMP);
#endif
  
}
void loop() 
{
  loopCount ++;
  bool forceTransmit = false;
  
  // When we wake up the 5th time after power on, switch to 1Mhz clock
  // This allows us to print debug messages on startup (as serial port is dependend on oscilator settings).
  //if ( (loopCount == 5) && highfreq)
  //{
  //  switchClock(1<<CLKPS2); // Switch to 1Mhz for the reminder of the sketch, save power.
  //}
  
  if (loopCount > FORCE_TRANSMIT_INTERVAL)
  { // force a transmission
    forceTransmit = true; 
    loopCount = 0;
  }
  // Process incoming messages (like config from server)
  node.process();
  measureBattery(forceTransmit);
  
  #if LIGHT_LEVEL_ENABLE
  readLDRLightLevel(forceTransmit);
  #endif
  
   #if DALLAS_ENABLE
  readDS18B20(forceTransmit);
  #endif
  
  #if HTU21D_ENABLE
  readHTU21DTemperature(forceTransmit);
  readHTU21DHumidity(forceTransmit);  
  #endif
  
  node.sleep(SLEEP_TIME);
   
  
}

#if DHT_ENABLE
void readDHTHumidityAndTemperature(bool force)
{
  static float lastTemp = 0;
  static float lastHumidity = 0;
  
  if (force)
  {
    lastTemp = -100.0;
    lastHumidity = -100.0;
  }
  
  float humidity = dht.getHumidity();
  
  if(!isnan(humidity))
  {
    if(lastHumidity != humidity)
    {
      node.send(msgHum.set(humidity,1));
      lastHumidity = humidity;
    }
  }
  float temperature = dht.getTemperature();
}
#endif

#if HTU21D_ENABLE
void readHTU21DTemperature(bool force)
{
  static float lastTemp = 0;
  
  if (force)
  {
   lastTemperature = -100;
  }
  float temp = myHumidity.readTemperature();
  
  if(lastTemp != temp)
  {
    node.send(msgTemp.set(temp,1));
    lastTemp = temp;
  }
}

void readHTU21DHumidity(bool force)
{
  static float lastHumidity = 0;
  
  if (force) 
  {
    lastHumidity = -100;
  }
  float humd = myHumidity.readHumidity();
  
  if(lastHumidity != humd)
  {
    node.send(msgHum.set(humd,1));
    lastHumidity = humd;
  }
}
#endif

#if DALLAS_ENABLE
void readDS18B20(DeviceAddress deviceAddress, bool force)
{
  static float lastTemperatureG = -200.1;
  static float lastTemperatureF = -200.1;
  if (force) 
  {
    lastTemperatureG = -100;
    lastTemperatureF = -100;
  }
  
  float tempC = dallas_sensor.getTempC(deviceAddress);
  Serial.print("Temp C: ");
  Serial.print(tempC);
  
  // Only send data if temperature has changed and no error
  if (lastTemperature != tempC && tempC != -127.00 )
  {
    // Send in the new temperature
    if (deviceAddress == garageThermometer)
    {
      node.send(msgDallasTemp.set(tempC,1));
      lastTemperature = tempC;
  }
}
#endif



#if LIGHT_LEVEL_ENABLE
void readLDRLightLevel(bool force)
{

  static int lastLightLevel = 0;
  
  if (force) 
  {
    lastLightLevel = -100;
  }
  
  int lightLevel = (1023-analogRead(LIGHT_SENSOR_ANALOG_PIN))/10.23;
  if (lightLevel != lastLightLevel)
  {
      node.send(msgLightLevel.set(lightLevel));
      lastLightLevel = lightLevel;
  }
 
}
#endif


/**
* Get the percentage of power in the battery
*/
uint8_t getBatteryPercent()
{
  static const float full_battery_v = 3169.0;
  float level = readVcc() / full_battery_v;
  uint8_t percent = level * 100;
  #if DEBUG
  Serial.print("Battery state = ");
  Serial.println(percent);
  Serial.println('\r');
  #endif
  node.sendBatteryLevel(percent);
  return percent;
}

uint16_t measureBattery(bool force)
{
  static uint16_t lastVcc = 0;
  
  if (force)
  {
    lastVcc = 0;
  }
  
  uint16_t thisVcc = readVcc();
  if(thisVcc != lastVcc)
  {
    node.send(msgVolt.set(readVcc(), 1));
    lastVcc = thisVcc;
  }
  return thisVcc;
  
}
/**
* Measure remaining voltage in battery in millivolts
*
* From http://www.seeedstudio.com/wiki/DevDuino_Sensor_Node_V2.0_(ATmega_328)#Measurement_voltage_power
*/

uint16_t readVcc()
{
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif
  delay(75); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
  uint8_t low = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both
  long result = (high<<8) | low;
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  #if DEBUG
  Serial.print("Read Vcc = ");
  Serial.println(result);
  Serial.println('\r');
  #endif
  return (uint16_t)result; // Vcc in millivolts
  
}

void switchClock(unsigned char clk)
{
  cli();
  
  CLKPR = 1<<CLKPCE; // Set CLKPCE to enable clk switching
  CLKPR = clk;  
  sei();
  highfreq = false;
}

