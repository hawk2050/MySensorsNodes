/*
* MYS_v11_MySensorNode.ino - Firmware for Mys v1.1 board temperature and humidity sensor Node with nRF24L01+ module
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
* Mys v1.1 -http://www.sa2avr.se/mys-1-1/
* nRF24L01+ spec - https://www.sparkfun.com/datasheets/Wireless/Nordic/nRF24L01P_Product_Specification_1_0.pdf
*
Hardware Connections (Breakoutboard to Arduino):
 -VCC = 3.3V
 -GND = GND
 -SDA = A4 (use inline 10k resistor if your board is 5V)
 -SCL = A5 (use inline 10k resistor if your board is 5V)

Mys_v1.1 board compatible with Arduino PRO Mini 3.3V@8MHz

System Clock  = 8MHz

 */

#include <MySensor.h>
#include <SPI.h>
#include <stdint.h>
#include <math.h>


#define API_v15
#define DEBUG_RCC 0


// FORCE_TRANSMIT_INTERVAL, this number of times of wakeup, the sensor is forced to report all values to 
// the controller
#define FORCE_TRANSMIT_INTERVAL 3 
#define SLEEP_TIME 300000
#define MAX_ATTACHED_DS18B20 2

#define NODE_ID 7



#define LIGHT_LEVEL_ENABLE  0
#define DALLAS_ENABLE       0
#define HTU21D_ENABLE       1
#define DHT_ENABLE          0

enum sensor_id
{
  CHILD_ID_LIGHT = 0,
  CHILD_ID_HTU21D_HUMIDITY,
  CHILD_ID_HTU21D_TEMP,
  CHILD_ID_DHT22_HUMIDITY,
  CHILD_ID_DHT22_TEMP,
  CHILD_ID_DALLAS_TEMP_BASE,
  CHILD_ID_VOLTAGE = CHILD_ID_DALLAS_TEMP_BASE + MAX_ATTACHED_DS18B20
  
};


/***********************************/
/********* PIN DEFINITIONS *********/
/***********************************/
#define LED_pin 9
#define LIGHT_SENSOR_ANALOG_PIN A0
// Data wire is plugged into port 3 on the Arduino
#define HUMIDITY_SENSOR_DIGITAL_PIN 2

#define ONE_WIRE_BUS 5

/*These are actually the default pins expected by the MySensors framework.
 * This means we can use the default constructor without arguments when 
 * creating an instance of the Mysensors class. Other defaults will include
 * transmitting on channel 76 with a data rate of 250kbps.
 */
#define RF24_CE_PIN 9
#define RF24_CS_PIN 10

/*****************************/
/********* FUNCTIONS *********/
/*****************************/
#if DHT_ENABLE
#include "DHT.h"
DHT dht;
MyMessage msgHum(CHILD_ID_DHT22_HUMIDITY, V_HUM);
MyMessage msgTemp(CHILD_ID_DHT22_TEMP, V_TEMP);
#endif

#if HTU21D_ENABLE
#include <Wire.h>
#include "HTU21D.h"
//Create an instance of the object
HTU21D myHumidity;
MyMessage msgHum(CHILD_ID_HTU21D_HUMIDITY, V_HUM);
MyMessage msgTemp(CHILD_ID_HTU21D_TEMP, V_TEMP);
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

boolean receivedConfig = false;
boolean metric = true; 
uint8_t loopCount = 0;
uint8_t clockSwitchCount = 0;
/************************************/
/********* GLOBAL VARIABLES *********/
/************************************/
#ifdef API_v15
MyTransportNRF24 transport(RF24_CE_PIN, RF24_CS_PIN, RF24_PA_LEVEL);
/*We're also tried to make the MySensors class hardware independent by introducing hardware profiles. 
 * They handle platform dependent things like sleeping, storage (EEPROM), watchdog, serial in- and output. 
 * Currently there is only one implementation for the ATMega328p (which also works fine for AtMega 2560)
 *Construct the class like this:
  */
MyHwATMega328 hw;
MySensor node(transport,hw);
#else
MySensor node(RF24_CE_PIN, RF24_CS_PIN);;
#endif


#if DALLAS_ENABLE
#include <OneWire.h>
#include <DallasTemperature.h>
float lastTemperature[MAX_ATTACHED_DS18B20];
int numSensors=0;
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature dallas_sensor(&oneWire);


void readDS18B20(bool force);
// Initialize temperature message
MyMessage msgDallas(CHILD_ID_DALLAS_TEMP_BASE, V_TEMP);

#endif
/**********************************/
/********* IMPLEMENTATION *********/
/**********************************/
void setup()
{
  // start serial port
  //Serial.begin();
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
  node.sendSketchInfo("mys_v11-temp-hum", "0.4");
  
  node.present(CHILD_ID_VOLTAGE, S_CUSTOM);
  // Register all sensors to gateway (they will be created as child devices)

#if DALLAS_ENABLE
  dallas_sensor.begin();
  // Fetch the number of attached temperature sensors  
  // locate devices on the bus
  
  Serial.print("Locating devices...");
  Serial.print("Found ");
  numSensors = dallas_sensor.getDeviceCount();
  Serial.print(numSensors, DEC);
  Serial.println(" devices.");

  // Present all sensors to controller
  for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++)
  {   
     node.present(CHILD_ID_DALLAS_TEMP_BASE+i, S_TEMP);
  }
  
#endif

#if LIGHT_LEVEL_ENABLE
  node.present(CHILD_ID_LIGHT, S_LIGHT_LEVEL);
#endif

#if HTU21D_ENABLE
  myHumidity.begin();
  node.present(CHILD_ID_HTU21D_HUMIDITY, S_HUM);
  node.present(CHILD_ID_HTU21D_TEMP, S_TEMP);
#endif

#if DHT_ENABLE
  dht.setup(HUMIDITY_SENSOR_DIGITAL_PIN); 
  node.present(CHILD_ID_DHT22_HUMIDITY, S_HUM);
  node.present(CHILD_ID_DHT22_TEMP, S_TEMP);
#endif
  
}
void loop() 
{
  
  bool forceTransmit = false;
  
  loopCount++;
  clockSwitchCount++;
  
  // When we wake up the 5th time after power on, switch to 4Mhz clock
  // This allows us to print debug messages on startup (as serial port is dependend on oscilator settings).
  // Switch to 1Mhz for the reminder of the sketch, save power and allow operation down to 1.8V
  //BUG: loopCount nevers gets to 5 since it's reset to 0 after it gets above FORCE_TRANSMIT_INTERVAL
  if ( (clockSwitchCount == 5) && highfreq)
  {
    switchClock(1<<CLKPS0); //should divide by 2 giving 4MHz operation
  }
  
  if (loopCount > FORCE_TRANSMIT_INTERVAL)
  { // force a transmission
    forceTransmit = true; 
    loopCount = 0;
  }
  // Process incoming messages (like config from server)
  node.process();
  measureBattery(forceTransmit);
  //measureBattery(true);
  
  #if LIGHT_LEVEL_ENABLE
  readLDRLightLevel(forceTransmit);
  #endif
  
   #if DALLAS_ENABLE
  readDS18B20(forceTransmit);
  #endif
  
  #if HTU21D_ENABLE
  readHTU21DTemperature(forceTransmit);
  readHTU21DHumidity(forceTransmit);  
  //readHTU21DTemperature(true);
  //readHTU21DHumidity(true);  
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
   lastTemp = -100;
  }
  float temp = myHumidity.readTemperature();
  
  if(lastTemp != temp)
  {
    node.send(msgTemp.set(temp,1));
    lastTemp = temp;
    #ifdef DEBUG_RCC
    Serial.print(" Temperature:");
    Serial.print(temp, 1);
    Serial.print("C");
    Serial.println();
    #endif
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
    #ifdef DEBUG_RCC
    Serial.print(" Humidity:");
    Serial.print(humd, 1);
    Serial.print("%");
    Serial.println();
    #endif
  }
}
#endif

#if DALLAS_ENABLE
void readDS18B20(bool force)
{
  if (force) 
  {
    for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++)
    {
      lastTemperature[i] = -100.0;
    }
  }
  // Fetch temperatures from Dallas sensors
  dallas_sensor.requestTemperatures(); 
  
  // Read temperatures and send them to controller 
  for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++)
  {
 
    // Fetch and round temperature to one decimal
    float temperature = static_cast<float>(static_cast<int>((node.getConfig().isMetric ? dallas_sensor.getTempCByIndex(i) : dallas_sensor.getTempFByIndex(i)) * 10.)) / 10.;
 
    // Only send data if temperature has changed and no error
    if (lastTemperature[i] != temperature && temperature != -127.00)
    {
 
      // Send in the new temperature
      node.send(msgDallas.setSensor(i).set(temperature,1));
      lastTemperature[i]=temperature;
    }
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
  #if DEBUG_RCC
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
    #if DEBUG_RCC
    Serial.print("Battery voltage = ");
    Serial.println(thisVcc);
    Serial.println('\r');
    #endif
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
  #if DEBUG_RCC
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

