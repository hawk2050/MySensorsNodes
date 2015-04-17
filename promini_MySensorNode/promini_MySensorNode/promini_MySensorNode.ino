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
*/
#include <MyMessage.h>
#include <MySensor.h>
#include <SPI.h>
#include <stdint.h>
#include <math.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define SLEEP_TIME 300000

#define NODE_ID 7

#define DEBUG 0

#define LIGHT_LEVEL_ENABLE 0
#define MCP9700_ENABLE 0
#define DALLAS_ENABLE 1

#define CHILD_ID_VOLTAGE 3
#define CHILD_ID_MCP9700_TEMP 2
#define CHILD_ID_DALLAS_TEMP 1
#define CHILD_ID_LIGHT 0

/***********************************/
/********* PIN DEFINITIONS *********/
/***********************************/
#define LED_pin 9
#define LIGHT_SENSOR_ANALOG_PIN A0
// Data wire is plugged into port 3 on the Arduino
#define RF_POWER_REG 2
#define ONE_WIRE_BUS 3
#define RF24_CE_pin 8
#define RF24_CS_pin 7
#define MCP9700_pin A3
/*****************************/
/********* FUNCTIONS *********/
/*****************************/
#if MCP9700_ENABLE
float readMCP9700Temp();
MyMessage msgMCP9700Temp(CHILD_ID_MCP9700_TEMP, V_TEMP);
#endif

#if LIGHT_LEVEL_ENABLE
void readLDRLightLevel();
MyMessage msgLightLevel(CHILD_ID_LIGHT, V_LIGHT_LEVEL);
#endif

#if DALLAS_ENABLE
void readDS18B20();
MyMessage msgDallasTemp(CHILD_ID_DALLAS_TEMP, V_TEMP);
#endif

uint16_t measureBattery();
MyMessage msgVolt(CHILD_ID_VOLTAGE, V_VOLTAGE); 

uint8_t getBatteryPercent();


uint16_t readVcc();


float lastTemperature;
boolean receivedConfig = false;
boolean metric = true; 

uint8_t loopCount = 0;
/************************************/
/********* GLOBAL VARIABLES *********/
/************************************/
MySensor node(RF24_CE_pin, RF24_CS_pin);



// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature dallas_sensor(&oneWire);

// arrays to hold device address
DeviceAddress insideThermometer;
/**********************************/
/********* IMPLEMENTATION *********/
/**********************************/
void setup()
{
  
  /*
  ** Auto Node numbering
  node.begin();
  */
  
  pinMode(RF_POWER_REG, OUTPUT);      // sets the digital pin as output
  digitalWrite(RF_POWER_REG, HIGH);   // Apply power to the nRF24L01 adapter board
  
  node.begin(NULL,NODE_ID);
  analogReference(INTERNAL);
  node.sendSketchInfo("promini-temp-sensor", "0.1");
  
  node.present(CHILD_ID_VOLTAGE, S_CUSTOM);
  // Register all sensors to gateway (they will be created as child devices)
#if MCP9700_ENABLE
  node.present(CHILD_ID_MCP9700_TEMP, S_TEMP);
#endif

#if DALLAS_ENABLE
  node.present(CHILD_ID_DALLAS_TEMP, S_TEMP);
#endif

#if LIGHT_LEVEL_ENABLE
  node.present(CHILD_ID_LIGHT, S_LIGHT_LEVEL);
#endif
  
}
void loop() 
{
  
  // Process incoming messages (like config from server)
  node.process();
  measureBattery();
  
  #if LIGHT_LEVEL_ENABLE
  readLDRLightLevel();
  #endif
  
  #if MCP9700_ENABLE
  readMCP9700Temp();
  #endif
  
  #if DALLAS_ENABLE
  readDS18B20();
  #endif
  
  digitalWrite(RF_POWER_REG, LOW);
  node.sleep(SLEEP_TIME);
  digitalWrite(RF_POWER_REG, HIGH);
  
  loopCount = loopCount++ & 0x3;
}

void readDS18B20()
{
  static float lastTemperature = -200.1;
  // Fetch temperatures from Dallas sensors
  dallas_sensor.requestTemperatures(); 
  // Fetch and round temperature to one decimal
  float temperature = static_cast<float>(static_cast<int>((node.getConfig().isMetric?dallas_sensor.getTempCByIndex(0):dallas_sensor.getTempFByIndex(0)) * 10.)) / 10.;
  // Only send data if temperature has changed and no error
  if (lastTemperature != temperature && temperature != -127.00 || loopCount == 0)
  {
    // Send in the new temperature
    node.send(msgDallasTemp.set(temperature,1));
    lastTemperature = temperature;
  }
}


#if LIGHT_LEVEL_ENABLE
void readLDRLightLevel()
{

  static int lastLightLevel = 0;
  int lightLevel = (1023-analogRead(LIGHT_SENSOR_ANALOG_PIN))/10.23;
  if (lightLevel != lastLightLevel || loopCount == 0)
  {
      node.send(msgLightLevel.set(lightLevel));
      lastLightLevel = lightLevel;
  }
 
}
#endif

/**
* Read the temperature from MCP9700
*/

#if MCP9700_ENABLE
float readMCP9700Temp() 
{

static float lastTemp = -200.0;
  float temp = analogRead(MCP9700_pin)*3.3/1024.0;
  temp = temp - 0.5;
  temp = temp / 0.01;
  #if DEBUG
  Serial.print("Read Temp from MCP9700 = ");
  Serial.println(temp);
  Serial.println('\r');
  #endif
  if (temp != lastTemp || loopCount == 0)
  {
    node.send(msgMCP9700Temp.set(temp, 1));
    lastTemp = temp;
  }
  return temp;
  
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

uint16_t measureBattery()
{
  static uint16_t lastVcc = 0;
  
  uint16_t thisVcc = readVcc();
  if(thisVcc != lastVcc || loopCount == 0)
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

