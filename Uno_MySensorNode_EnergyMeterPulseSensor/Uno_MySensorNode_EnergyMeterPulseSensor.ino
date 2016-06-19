/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * REVISION HISTORY
 * Version 1.0 - Henrik EKblad
 * 
 * DESCRIPTION
 * This sketch provides an example how to implement a distance sensor using HC-SR04 
 * Use this sensor to measure KWH and Watt of your house meeter
 * You need to set the correct pulsefactor of your meeter (blinks per KWH).
 * The sensor starts by fetching current KWH value from gateway.
 * Reports both KWH and Watt back to gateway.
 *
 * Unfortunately millis() won't increment when the Arduino is in 
 * sleepmode. So we cannot make this sensor sleep if we also want 
 * to calculate/report watt-number.
 * http://www.mysensors.org/build/pulse_power
 */

#include <SPI.h>
#include <MySensor.h>  

// display
#include <Wire.h>                                           // I2C
#include <LiquidCrystal_I2C.h>                              // LCD display with I2C interface


// helpers
#define LOCAL_DEBUG

#ifdef LOCAL_DEBUG
#define Sprint(a) (Serial.print(a))                     // macro as substitute for print, enable if no print wanted
#define Sprintln(a) (Serial.println(a))                 // macro as substitute for println
#else
#define Sprint(a)                                       // enable if no print wanted -or- 
#define Sprintln(a)                                     // enable if no print wanted
#endif

#define NODE_ID 20


#define RF24_CE_PIN 9
#define RF24_CS_PIN 10
MyTransportNRF24 transport(RF24_CE_PIN, RF24_CS_PIN, RF24_PA_LEVEL);

#define DIGITAL_INPUT_SENSOR 3  // The digital input you attached your light sensor.  (Only 2 and 3 generates interrupt!)
#define PULSE_FACTOR 1000       // Nummber of blinks per KWH of your meeter
#define SLEEP_MODE false        // Watt-value can only be reported when sleep mode is false.
#define MAX_WATT 10000          // Max watt value to report. This filetrs outliers.
#define INTERRUPT DIGITAL_INPUT_SENSOR-2 // Usually the interrupt = pin -2 (on uno/nano anyway)
#define CHILD_ID 1              // Id of the sensor child
unsigned long SEND_FREQUENCY = 5000; // Minimum time between send (in milliseconds). We don't wnat to spam the gateway.


MyHwATMega328 hw;
MySensor gw(transport,hw);

// ***** LCD
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

double ppwh = ((double)PULSE_FACTOR)/1000; // Pulses per watt hour
boolean pcReceived = false;
volatile unsigned long pulseCount = 0;   
volatile unsigned long lastBlink = 0;
volatile unsigned long watt = 0;
unsigned long oldPulseCount = 0;   
unsigned long oldWatt = 0;
double oldKwh;
double kwh;
unsigned long lastSend;
MyMessage wattMsg(CHILD_ID,V_WATT);
MyMessage kwhMsg(CHILD_ID,V_KWH);
MyMessage pcMsg(CHILD_ID,V_VAR1);


void setup()  
{  
  Wire.begin();  // I2C
  // ** LCD display **
  // LCD 2 lines * 16 char.
  lcd.begin(16, 2);
  lcd.setBacklight(HIGH);
  lcd.setCursor(0, 0);
  lcd.print("Energy Pulse Meter");   
   
  #ifdef NODE_ID
  gw.begin(NULL,NODE_ID,false);
  #else
  gw.begin(NULL,AUTO,false);
  #endif
  //gw.begin(incomingMessage);

  // Send the sketch version information to the gateway and Controller
  gw.sendSketchInfo("Energy Meter", "1.0");

  // Register this device as power sensor
  gw.present(CHILD_ID, S_POWER);

  // Fetch last known pulse count value from gw
  //gw.request(CHILD_ID, V_VAR1);
  
  attachInterrupt(INTERRUPT, onPulse, RISING);
  lastSend=millis();

  pcReceived = true;
}


void loop()     
{ 
  LCD_local_display();
  gw.process();
  unsigned long now = millis();
  // Only send values at a maximum frequency or woken up from sleep
  bool sendTime = now - lastSend > SEND_FREQUENCY;
  if (pcReceived && (SLEEP_MODE || sendTime))
  {
    // New watt value has been calculated  
    if (!SLEEP_MODE && watt != oldWatt)
    {
      // Check that we dont get unreasonable large watt value. 
      // could hapen when long wraps or false interrupt triggered
      if (watt<((unsigned long)MAX_WATT))
      {
        gw.send(wattMsg.set(watt));  // Send watt value to gw 
      }  
      Serial.print("Watt:");
      Serial.println(watt);
      oldWatt = watt;
    }
  
    // Pulse cout has changed
    if (pulseCount != oldPulseCount)
    {
      gw.send(pcMsg.set(pulseCount));  // Send pulse count value to gw 
      kwh = ((double)pulseCount/((double)PULSE_FACTOR));     
      oldPulseCount = pulseCount;
      if (kwh != oldKwh)
      {
        gw.send(kwhMsg.set(kwh, 4));  // Send kwh value to gw 
        oldKwh = kwh;
      }
    }    
    lastSend = now;
  }
  else if (sendTime && !pcReceived)
  {
    // No count received. Try requesting it again
    gw.request(CHILD_ID, V_VAR1);
    lastSend=now;
  }
  
  if (SLEEP_MODE)
  {
    gw.sleep(SEND_FREQUENCY);
  }
}

/*
void incomingMessage(const MyMessage &message) {
  if (message.type==V_VAR1) {  
    pulseCount = oldPulseCount = message.getLong();
    Serial.print("Received last pulse count from gw:");
    Serial.println(pulseCount);
    pcReceived = true;
  }
}
*/
void onPulse()     
{ 
  if (!SLEEP_MODE) {
    unsigned long newBlink = micros();  
    unsigned long interval = newBlink-lastBlink;
    if (interval<10000L) { // Sometimes we get interrupt on RISING
      return;
    }
    watt = (3600000000.0 /interval) / ppwh;
    lastBlink = newBlink;
  } 
  pulseCount++;
}


void LCD_local_display(void)
{
/* prints all available variables on LCD display with units
*/
    
    char buf[17];                                           // buffer for max 16 char display
    lcd.setCursor(0, 0);
    snprintf(buf, sizeof buf, "KWh:%5d ", kwh);
    lcd.print(buf);
    lcd.setCursor(0, 1);
    snprintf(buf, sizeof buf, "Load:%4d watts", watt);
    lcd.print(buf);
}
