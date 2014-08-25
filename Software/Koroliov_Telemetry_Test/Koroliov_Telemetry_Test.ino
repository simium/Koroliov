/*

 Based on the demo code to drive NTX2B via PWM by Anthony Stirk M0UPU.
 RTTY code from Rob Harrison Icarus Project.
 
 Special thanks to http://ukhas.org.uk
 
 MPL3115A2 code by Nathan Seidle from Sparkfun.
 
 */

/* Useful Constants */
#define SECS_PER_MIN  (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY  (SECS_PER_HOUR * 24L)
#define TELEMETRY_INTERVAL (15000)

/* Useful Macros for getting elapsed time */
#define numberOfSeconds(_time_) (_time_ % SECS_PER_MIN)  
#define numberOfMinutes(_time_) ((_time_ / SECS_PER_MIN) % SECS_PER_MIN) 
#define numberOfHours(_time_) (( _time_% SECS_PER_DAY) / SECS_PER_HOUR)
#define elapsedDays(_time_) ( _time_ / SECS_PER_DAY)  

/* Pinout definitions */
#define RADIOPIN 9
#define TMP36PIN 0

/* Includes */
#include <string.h>
#include<stdlib.h>
#include <util/crc16.h>
#include <Wire.h>
#include "MPL3115A2.h"

char datastring[80];
char time_str[9+1];
char temp_str[15+1];
char temp_str_aux[7+1];
char press_str[15+1];
char press_str_aux[7+1];
char alt_str[15+1];
char alt_str_aux[7+1];
char checksum_str[5+1];

unsigned long previousMillis = 0;

//Create an instance of the object
MPL3115A2 myPressure;

void setup() {
  Wire.begin();        // Join i2c bus
  myPressure.begin(); // Get sensor online

  pinMode(RADIOPIN,OUTPUT);
  setKoroliovPwmFrequency();
  analogReference(INTERNAL);
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis > TELEMETRY_INTERVAL) {
    previousMillis = currentMillis;

    /* CALLSIGN */
    snprintf(datastring,80,"$$KRLV"); // Puts the text in the datastring

    /* Time */
    unsigned long time = currentMillis/1000;
    int hours = numberOfHours(time);
    int minutes = numberOfMinutes(time);
    int seconds = numberOfSeconds(time);
    sprintf(time_str, "|%02d:%02d:%02d", hours, minutes, seconds);
    strcat(datastring,time_str);

    /* Telemtry Data */
    float temperature = 0.0;
    float voltage = 0.0;
    int temperatureReading = 0;
    float pressure = 0.0;
    float altitude = 0.0;

    /* TMP36 Temperature */
    temperatureReading = analogRead(TMP36PIN);
    voltage = temperatureReading * 2.56;
    voltage = voltage/1024.0;
    temperature = (voltage-0.5)*100;
    dtostrf(temperature, 5, 2, temp_str_aux);
    sprintf(temp_str, "|%sC", temp_str_aux);
    strcat(datastring,temp_str);

    /* MPL3115A2 Pressure, Temperature and Altitude */
    myPressure.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa
    myPressure.setOversampleRate(7); // Set Oversample to the recommended 128
    myPressure.enableEventFlags(); // Enable all three pressure and temp event flags 
    pressure = myPressure.readPressure();
    temperature = myPressure.readTemp();

    dtostrf(pressure, 9, 2, press_str_aux);
    sprintf(press_str, "|%sPa", press_str_aux);
    strcat(datastring,press_str);

    dtostrf(temperature, 5, 2, temp_str_aux);
    sprintf(temp_str, "|%sC", temp_str_aux);
    strcat(datastring,temp_str);

    myPressure.setModeAltimeter(); // Measure altitude above sea level in meters
    myPressure.setOversampleRate(7); // Set Oversample to the recommended 128
    myPressure.enableEventFlags(); // Enable all three pressure and temp event flags 
    
    altitude = myPressure.readAltitude();
    
    dtostrf(altitude, 5, 2, alt_str_aux);
    sprintf(alt_str, "|%sM", alt_str_aux);
    strcat(datastring,alt_str);

    /* CRC16 checksum */
    unsigned int CHECKSUM = gps_CRC16_checksum(datastring); // Calculates the checksum for this datastring
    sprintf(checksum_str, "*%04X\n", CHECKSUM);
    strcat(datastring,checksum_str);
    rtty_txstring (datastring);
  }
}

void rtty_txstring (char * string)
{

  /* Simple function to sent a char at a time to
   ** rtty_txbyte function.
   ** NB Each char is one byte (8 Bits)
   */

  char c;

  c = *string++;

  while ( c != '\0')
  {
    rtty_txbyte (c);
    c = *string++;
  }
}
void rtty_txbyte (char c)
{
  /* Simple function to sent each bit of a char to
   ** rtty_txbit function.
   ** NB The bits are sent Least Significant Bit first
   **
   ** All chars should be preceded with a 0 and
   ** proceed with a 1. 0 = Start bit; 1 = Stop bit
   **
   */

  int i;

  rtty_txbit (0); // Start bit

  // Send bits for for char LSB first

  for (i=0;i<7;i++) // Change this here 7 or 8 for ASCII-7 / ASCII-8
  {
    if (c & 1) rtty_txbit(1);

    else rtty_txbit(0);

    c = c >> 1;

  }
  rtty_txbit (1); // Stop bit
  rtty_txbit (1); // Stop bit
}

void rtty_txbit (int bit)
{
  if (bit)
  {
    // high
    analogWrite(RADIOPIN,110);
  }
  else
  {
    // low
    analogWrite(RADIOPIN,100);

  }

  // delayMicroseconds(3370); // 300 baud
  delayMicroseconds(10000); // For 50 Baud uncomment this and the line below.
  delayMicroseconds(10150); // You can't do 20150 it just doesn't work as the
  // largest value that will produce an accurate delay is 16383
  // See : http://arduino.cc/en/Reference/DelayMicroseconds

}

uint16_t gps_CRC16_checksum (char *string) {
  size_t i;
  uint16_t crc;
  uint8_t c;

  crc = 0xFFFF;

  // Calculate checksum ignoring the first two $s
  for (i = 2; i < strlen(string); i++)
  {
    c = string[i];
    crc = _crc_xmodem_update (crc, c);
  }

  return crc;
}

void setKoroliovPwmFrequency() {
  /* Radio pin is always 9 in Koroliov board */
  TCCR1B = TCCR1B & 0b11111000 | 0x01;
}

/*
void setPwmFrequency(int pin, int divisor) {
 
 byte mode;
 if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
 switch(divisor) {
 case 1:
 mode = 0x01;
 break;
 case 8:
 mode = 0x02;
 break;
 case 64:
 mode = 0x03;
 break;
 case 256:
 mode = 0x04;
 break;
 case 1024:
 mode = 0x05;
 break;
 default:
 return;
 }
 if(pin == 5 || pin == 6) {
 TCCR0B = TCCR0B & 0b11111000 | mode;
 }
 else {
 TCCR1B = TCCR1B & 0b11111000 | mode;
 }
 }
 
 TCCR1B = TCCR1B & 0b11111000 | mode;
 }
 */
















