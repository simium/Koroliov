/*

 RTTY code by Philip Heron <phil@sanslogic.co.uk> https://github.com/fsphil/
 Arduino project structure based on the CUBEX project by https://github.com/arkorobotics/
 
 Special thanks to http://ukhas.org.uk and the people in the IRC.
 
 GPS and Camera code by Adafruit.
 
 */

/* Includes */
#include <string.h>
#include<stdlib.h>
#include <util/crc16.h>
#include <Adafruit_VC0706.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include "ssdv.h"
#include "config.h"
#include "rtty.h"

char datastring[80];
char temp_str[16+1];
char temp_str_aux[16+1];
char checksum_str[5+1];

unsigned long previousTelemetryMillis = 0;
unsigned long previousSSDVMillis = 0;

int errorstatus=0;
unsigned char cameracode = 0x00;
int telemetry_sent = 0;
unsigned long currentMillis;

SoftwareSerial camera_connection(8, 5);
Adafruit_VC0706 cam = Adafruit_VC0706(&camera_connection);

Adafruit_GPS GPS(&Serial1);
HardwareSerial mySerial = Serial1;

boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

_payload Koroliov;

int first_boot = 1;

void setup() {
  pinMode(RADIO_TX_PIN,OUTPUT);

  setKoroliovPwmFrequency();
  analogReference(INTERNAL);

  /* Fix for CAM RX bug */
  pinMode(4, INPUT);

  // Turn on the camera
  cam.begin();

  // Turn on the GPS
  GPS.begin(9600);
  //GPS.begin(4800);

  delay(100);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  delay(50);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  delay(50);

  useInterrupt(true);

  while (Koroliov.sentence_id < 10) {
    if (update_payload_status(&Koroliov) != 0) {
      rtty_txtelemetry(&Koroliov);
      delay(2000);
    }
  }

}

void loop() {
  currentMillis = millis();

  if ((currentMillis - previousTelemetryMillis) > TELEMETRY_INTERVAL ) {
    previousTelemetryMillis = millis();
    
    if (update_payload_status(&Koroliov) != 0) {
      rtty_txtelemetry(&Koroliov);
    }
  }  

  if ((currentMillis - previousSSDVMillis) > SSDV_INTERVAL) {
    previousSSDVMillis = millis();
    rtty_tximage();
  }
}

int update_payload_status(struct _payload* payload) {
  int status = 0;
  
  if (GPS.newNMEAreceived()) {
    if (GPS.parse(GPS.lastNMEA())) {
      status = 1;

      payload->sentence_id++;
      payload->running_millis = millis();

      payload->gps_hour = GPS.hour;
      payload->gps_minute = GPS.minute;
      payload->gps_seconds = GPS.seconds;
      payload->gps_latitude = GPS.latitude;
      payload->gps_longitude = GPS.longitude;
      payload->gps_altitude = (uint16_t) GPS.altitude;

      /* TMP36 - Internal temperature */
      int temperatureReading = analogRead(TMP36PIN);
      float voltage = temperatureReading * 2.56;
      voltage = voltage/1024.0;

      payload->internal_temperature = (voltage-0.5)*100;
    }
  }



  return status;
}

void rtty_txtelemetry(struct _payload* payload) {

  /* CALLSIGN */
  snprintf(datastring,80,"$$KRLV"); // Puts the text in the datastring

  /* Sentence ID */
  sprintf(temp_str, ",%03d", payload->sentence_id);
  strcat(datastring,temp_str);

  /* Time */
  sprintf(temp_str, ",%02d:%02d:%02d", payload->gps_hour, payload->gps_minute, payload->gps_seconds);
  strcat(datastring,temp_str);

  /* Latitude */
  dtostrf(payload->gps_latitude, 0, 6, temp_str_aux);
  sprintf(temp_str, ",%s", temp_str_aux);
  strcat(datastring,temp_str);

  /* Longitude */
  dtostrf(payload->gps_longitude, 0, 6, temp_str_aux);
  sprintf(temp_str, ",%s", temp_str_aux);
  strcat(datastring,temp_str);

  /* Altitude */
  sprintf(temp_str, ",%d", payload->gps_altitude);
  strcat(datastring,temp_str);

  /* Internal Temperature */
  dtostrf(payload->internal_temperature, 0, 2, temp_str_aux);
  sprintf(temp_str, ",%s", temp_str_aux);
  strcat(datastring,temp_str);

  /* CRC16 checksum */
  unsigned int CHECKSUM = gps_CRC16_checksum(datastring); // Calculates the checksum for this datastring
  sprintf(checksum_str, "*%04X\n", CHECKSUM);
  strcat(datastring,checksum_str);

  rtx_string (datastring);
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
    cam.setCompression(255);

    imgsize = cam.getImageSize();

    if (cam.takePicture()) {
      jpglen = cam.frameLength();
    }
    else {
      sprintf(temp_str, "$$KRLV,CAMERROR\n");
      rtx_string (temp_str);
      jpglen = 0;
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

    ssdv_enc_feed(&ssdv, buffer, bytesToRead);
    jpglen -= bytesToRead;
  }

  if(r != SSDV_OK)
  {
    setup = 0;
    cam.resumeVideo();
  }

  if(!(jpglen > 0))
  {
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

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0x57;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}




