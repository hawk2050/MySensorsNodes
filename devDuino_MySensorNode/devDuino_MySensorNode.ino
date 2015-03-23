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

#define DEBUG 0

#define LIGHT_LEVEL_ENABLE 0
#define MCP9700_ENABLE 0

#define SLEEP_TIME 10000


#define CHILD_ID_MCP9700_TEMP 2
#define CHILD_ID_DALLAS_TEMP 1
#define CHILD_ID_LIGHT 0

/***********************************/
/********* PIN DEFINITIONS *********/
/***********************************/
#define LED_pin 9
#define LIGHT_SENSOR_ANALOG_PIN A0
// Data wire is plugged into port 3 on the Arduino
#define ONE_WIRE_BUS 3
#define RF24_CE_pin 8
#define RF24_CS_pin 7
#define MCP9700_pin A3
/*****************************/
/********* FUNCTIONS *********/
/*****************************/
#if MCP9700_ENABLE
float readMCP9700Temp();
#endif
uint16_t readVcc();
uint8_t getVccLevel();

#if LIGHT_LEVEL_ENABLE
int lastLightLevel;
#endif
float lastTemperature;
boolean receivedConfig = false;
boolean metric = true; 
/************************************/
/********* GLOBAL VARIABLES *********/
/************************************/
MySensor node(RF24_CE_pin, RF24_CS_pin);
#if MCP9700_ENABLE
MyMessage msgMCP9700Temp(CHILD_ID_MCP9700_TEMP, V_TEMP);
#endif
MyMessage msgDallasTemp(CHILD_ID_DALLAS_TEMP, V_TEMP);
#if LIGHT_LEVEL_ENABLE
MyMessage msgLightLevel(CHILD_ID_LIGHT, V_LIGHT_LEVEL);
#endif

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
  node.begin();
  node.sendSketchInfo("devduino-temp-sensor", "0.2");
  // Register all sensors to gateway (they will be created as child devices)
  #if MCP9700_ENABLE
  node.present(CHILD_ID_MCP9700_TEMP, S_TEMP);
  #endif
  node.present(CHILD_ID_DALLAS_TEMP, S_TEMP);
#if LIGHT_LEVEL_ENABLE
  node.present(CHILD_ID_LIGHT, S_LIGHT_LEVEL);
#endif
  
}
void loop() 
{
  // Process incoming messages (like config from server)
  node.process();
  node.sendBatteryLevel(getVccLevel());
  
  #if MCP9700_ENABLE
  node.send(msgMCP9700Temp.set(readMCP9700Temp(), 1));
  #endif
  
  // Fetch temperatures from Dallas sensors
  dallas_sensor.requestTemperatures(); 
  // Fetch and round temperature to one decimal
  float temperature = static_cast<float>(static_cast<int>((node.getConfig().isMetric?dallas_sensor.getTempCByIndex(0):dallas_sensor.getTempFByIndex(0)) * 10.)) / 10.;
  // Only send data if temperature has changed and no error
  //if (lastTemperature != temperature && temperature != -127.00)
  //{
    // Send in the new temperature
    node.send(msgDallasTemp.set(temperature,1));
    //lastTemperature = temperature;
  //}
  
#if LIGHT_LEVEL_ENABLE
  int lightLevel = (1023-analogRead(LIGHT_SENSOR_ANALOG_PIN))/10.23;
  if (lightLevel != lastLightLevel)
  {
      node.send(msgLightLevel.set(lightLevel));
      lastLightLevel = lightLevel;
  }
#endif
  node.sleep(SLEEP_TIME);
}



#if LIGHT_LEVEL_ENABLE
int readLightLevel()
{
  int lightLevel = (1023-analogRead(LIGHT_SENSOR_ANALOG_PIN))/10.23;
  return lightLevel;
  
}
#endif

/**
* Read the temperature from MCP9700
*/

#if MCP9700_ENABLE
float readMCP9700Temp() 
{
  float temp = analogRead(MCP9700_pin)*3.3/1024.0;
  temp = temp - 0.5;
  temp = temp / 0.01;
  #if DEBUG
  Serial.print("Read Temp from MCP9700 = ");
  Serial.println(temp);
  Serial.println('\r');
  #endif
  return temp;
}
#endif
/**
* Get the percentage of power in the battery
*/
uint8_t getVccLevel()
{
  static const float full_battery_v = 3169.0;
  float level = readVcc() / full_battery_v;
  uint8_t percent = level * 100;
  #if DEBUG
  Serial.print("Battery state = ");
  Serial.println(percent);
  Serial.println('\r');
  #endif
  return percent;
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
