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

#include <avr/wdt.h>

char datastring[80];
char temp_str[16+1];
char temp_str_aux[16+1];
char checksum_str[5+1];

int errorstatus=SSDV_ERROR;
unsigned char cameracode = 0x00;

SoftwareSerial camera_connection(CAM_RX_PIN, CAM_TX_PIN);
Adafruit_VC0706 cam = Adafruit_VC0706(&camera_connection);

Adafruit_GPS GPS(&Serial1);
HardwareSerial mySerial = Serial1;

_payload Koroliov;

static unsigned long previousTelemetrySeconds;
static unsigned long previousSSDVSeconds;

void setup() {
  pinMode(RADIO_TX_PIN,OUTPUT);
  pinMode(CUTDOWN_PIN, OUTPUT);

  setKoroliovPwmFrequency();
  analogReference(INTERNAL);

  /* Fix for CAM RX bug */
  pinMode(OLD_CAM_RX_PIN, INPUT);

  // Turn on the camera
  cam.begin();

  // Turn on the GPS
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate

  Koroliov.descending = false;

  int i = 0;

  while (i<5) {
    if (update_payload_status(&Koroliov) != 0) {
      rtty_txtelemetry(&Koroliov);
      delay(1000);
      i++;
    }
  }

  wdt_enable(WDTO_8S);
}

void loop() {
  unsigned long currentSeconds = millis()/1000;

  if ((currentSeconds - previousTelemetrySeconds) >= TELEMETRY_INTERVAL) {
    previousTelemetrySeconds = millis()/1000;
    while(!update_payload_status(&Koroliov));
    rtty_txtelemetry(&Koroliov);
  }

  if ((currentSeconds - previousSSDVSeconds) >= SSDV_INTERVAL && (int)Koroliov.gps_altitude >= 500) {
    previousSSDVSeconds = millis()/1000;
    rtty_tximage();
  }

  /* Cutdown <- altitude >= 30000m and more than 7200 seconds passed since boot (almost 2 hours) */
  if ((int)Koroliov.gps_altitude >= 30000 && currentSeconds > 6000 && Koroliov.descending == false) {
    wdt_disable();
    digitalWrite(CUTDOWN_PIN, HIGH);
    delay(10*1000);
    digitalWrite(CUTDOWN_PIN, LOW);
    wdt_enable(WDTO_8S);

    Koroliov.descending = true;
  }

  wdt_reset();
}

int update_payload_status(struct _payload* payload) {
  int status = 0;

  char c = GPS.read();
  while (c!=0) {
    c = GPS.read();
  }

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
      payload->gps_previous_altitude = payload->gps_altitude;
      payload->gps_altitude = GPS.altitude;

      /* TMP36 - Internal temperature */
      int temperatureReading = analogRead(TMP36_PIN);
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
  sprintf(temp_str, ",%lu", payload->running_millis/1000);
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
  dtostrf(payload->gps_altitude, 0, 1, temp_str_aux);
  sprintf(temp_str, ",%s", temp_str_aux);
  strcat(datastring,temp_str);

  /* Internal Temperature */
  dtostrf(payload->internal_temperature, 0, 2, temp_str_aux);
  sprintf(temp_str, ",%s", temp_str_aux);
  strcat(datastring,temp_str);

  /* CRC16 checksum */
  unsigned int CHECKSUM = gps_CRC16_checksum(datastring); // Calculates the checksum for this datastring
  sprintf(checksum_str, "*%04X\n\0", CHECKSUM);
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

    wdt_reset();

    cam.reset();
    cam.setImageSize(VC0706_320x240);
    cam.setCompression(255);

    imgsize = cam.getImageSize();

    if (cam.takePicture()) {
      jpglen = cam.frameLength();
    }
    else {
      jpglen = 0;
    }

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

    wdt_reset();
  }

  if(r != SSDV_OK || !(jpglen > 0))
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






