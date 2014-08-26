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
#define SSDV_INTERVAL      (600000)

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
#include <Adafruit_VC0706.h>
#include <SoftwareSerial.h>
#include "ssdv.h"
#include "config.h"
#include "rtty.h"

char datastring[80];
char time_str[9+1];
char temp_str[15+1];
char temp_str_aux[7+1];
char press_str[15+1];
char press_str_aux[15+1];
char alt_str[15+1];
char alt_str_aux[7+1];
char checksum_str[5+1];

char debug_str[31+1];

unsigned long previousTelemetryMillis = 0;
unsigned long previousSSDVMillis = 0;

int telemetrySent = 0;
int errorstatus=0;
unsigned char cameracode = 0x00;

//Create an instance of the object
MPL3115A2 myPressure;

SoftwareSerial cameraconnection = SoftwareSerial(8, 5);
Adafruit_VC0706 cam = Adafruit_VC0706(&cameraconnection);

void setup() {
  pinMode(RADIOPIN,OUTPUT);

  //rtx_init();

  Serial.begin(9600);
  Serial.println("VC0706 Camera snapshot test");

  Wire.begin();        // Join i2c bus
  myPressure.begin(); // Get sensor online

  setKoroliovPwmFrequency();
  analogReference(INTERNAL);

  pinMode(4, INPUT);
  pinMode(6,OUTPUT);
  // Try to locate the camera
  if (cam.begin()) {
    Serial.println("Camera Found:");
  } 
  else {
    Serial.println("No camera found?");
    return;
  }
  // Print out the camera version information (optional)
  char *reply = cam.getVersion();
  if (reply == 0) {
    Serial.print("Failed to get version");
  } 
  else {
    Serial.println("-----------------");
    Serial.print(reply);
    Serial.println("-----------------");
  }
}

void loop() {
  unsigned long currentMillis = millis();

  if ((currentMillis - previousTelemetryMillis) > TELEMETRY_INTERVAL ) {
    previousTelemetryMillis = currentMillis;

    /* CALLSIGN */
    snprintf(datastring,80,"$$KRLV"); // Puts the text in the datastring

    /* Time */
    unsigned long time = currentMillis/1000;
    int hours = numberOfHours(time);
    int minutes = numberOfMinutes(time);
    int seconds = numberOfSeconds(time);
    sprintf(time_str, ",%02d:%02d:%02d", hours, minutes, seconds);
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
    sprintf(temp_str, ",%sC", temp_str_aux);
    strcat(datastring,temp_str);

    /* MPL3115A2 Pressure, Temperature and Altitude */
    myPressure.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa
    myPressure.setOversampleRate(7); // Set Oversample to the recommended 128
    myPressure.enableEventFlags(); // Enable all three pressure and temp event flags 
    pressure = myPressure.readPressure();
    temperature = myPressure.readTemp();

    //dtostrf(pressure, 9, 2, press_str_aux);
    //sprintf(press_str, ",%sPa", press_str_aux);
    //strcat(datastring,press_str);

    dtostrf(temperature, 5, 2, temp_str_aux);
    sprintf(temp_str, ",%sC", temp_str_aux);
    strcat(datastring,temp_str);

    myPressure.setModeAltimeter(); // Measure altitude above sea level in meters
    myPressure.setOversampleRate(7); // Set Oversample to the recommended 128
    myPressure.enableEventFlags(); // Enable all three pressure and temp event flags 

      altitude = myPressure.readAltitude();

    dtostrf(altitude, 5, 2, alt_str_aux);
    sprintf(alt_str, ",%sM", alt_str_aux);
    strcat(datastring,alt_str);

    /* CRC16 checksum */
    unsigned int CHECKSUM = gps_CRC16_checksum(datastring); // Calculates the checksum for this datastring
    sprintf(checksum_str, "*%04X\n", CHECKSUM);
    strcat(datastring,checksum_str);
    rtx_string (datastring);

    telemetrySent++;
  }  

  if (telemetrySent >= 1) {
    telemetrySent = 0;
    rtty_tximage();
  }
}

void rtty_tximage(void)
{
  static char setup = 0;
  static uint8_t img_id = 0;
  static ssdv_t ssdv;
  static uint8_t pkt[SSDV_PKT_SIZE];
  static uint8_t img[32];
  int r;

  static uint8_t imgsize; 
  static uint16_t jpglen;

  if(!setup)
  {
    setup = -1;
    cam.setImageSize(VC0706_320x240);

    imgsize = cam.getImageSize();

    if (cam.takePicture()) {
      jpglen = cam.frameLength();

      sprintf(debug_str, "$$KRLV,Image size: %d\n", imgsize);
      rtx_string (debug_str);

      sprintf(debug_str, "$$KRLV,Frame length: %d\n", jpglen);
      rtx_string (debug_str);
    }
    else {
      sprintf(debug_str, "$$KRLV,takePicture() failed: %d\n");
      rtx_string (debug_str);
    }

    errorstatus = sizeof(ssdv_t);

    ssdv_enc_init(&ssdv, RTTY_CALLSIGN, img_id++);
    ssdv_enc_set_buffer(&ssdv, pkt);                  
  }

  while((r = ssdv_enc_get_packet(&ssdv)) == SSDV_FEED_ME)
  {
    uint8_t *buffer; 
    uint8_t bytesToRead = min(32, jpglen); // change 32 to 64 for a speedup but may not work with all setups!
    buffer = cam.readPicture(bytesToRead);

    //size_t r = c3_read(img, 64);
    //if(r == 0) break;

    ssdv_enc_feed(&ssdv, buffer, bytesToRead);
    jpglen -= bytesToRead;

    //rtx_string("$$" RTTY_CALLSIGN ":ssdv_enc_get_packet() SSDV_FEED_ME failed again\n");
  }

  if(r != SSDV_OK)
  {
    sprintf(debug_str, "$$KRLV,r != SSDV_OK, r=%d, s->width=%d, s->height=%d\n", r, ssdv.width, ssdv.height);
    rtx_string (debug_str);
    // Something went wrong! //
    //c3_close();
    //setup = 0;
    //rtx_string_P(PSTR("$$" RTTY_CALLSIGN ":ssdv_enc_get_packet() failed again\n"));
    //rtx_string("$$" RTTY_CALLSIGN ":ssdv_enc_get_packet() failed again\n");
    //snprintf_P((char *) img, 64, PSTR("$$" RTTY_CALLSIGN ":Camera error %d\n"), r);
    //rtx_string((char *) img);
    //return(setup);
    setup = 0;
    cam.resumeVideo();
  }

  if(!(jpglen > 0))
  {
    // The end of the image has been reached //
    //c3_close();
    setup = 0;
    cam.resumeVideo();
  }

  cameracode = r;
  // Got the packet! Transmit it //
  rtx_data(pkt, SSDV_PKT_SIZE);
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


