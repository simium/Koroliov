#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <stdint.h>
#include <Arduino.h>

#define RTTY_CALLSIGN "KRLV"

/* CONFIGURABLE BITS */
#define ASCII 8          // ASCII 7 or 8
#define STOPBITS 2       // Either 1 or 2
#define TXDELAY 0        // Delay between sentence TX's
#define RTTY_BAUD 300     // Baud rate for use with RFM22B Max = 600

#define TX_DELAY      300   // default was 300

#define RADIO_FREQUENCY      (434650000UL)

/* Useful Constants */
#define SECS_PER_MIN  (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY  (SECS_PER_HOUR * 24L)
#define TELEMETRY_INTERVAL (30)
#define SSDV_INTERVAL      (1)

/* Useful Macros for getting elapsed time */
#define numberOfSeconds(_time_) (_time_ % SECS_PER_MIN)  
#define numberOfMinutes(_time_) ((_time_ / SECS_PER_MIN) % SECS_PER_MIN) 
#define numberOfHours(_time_) (( _time_% SECS_PER_DAY) / SECS_PER_HOUR)
#define elapsedDays(_time_) ( _time_ / SECS_PER_DAY)  

/* Digital pins */
#define OLD_CAM_RX_PIN         (4)
#define CAM_RX_PIN         (8)
#define CAM_TX_PIN         (5)
#define RADIO_TX_PIN         (9)
#define CUTDOWN_PIN         (13)
//#define STATUS_RADIO_LED     (8)
//#define STATUS_ALTIMETER_LED (7)
//#define STATUS_CAMERA_LED    (6)
//#define CAM_RX_PIN           (5)
//#define CAM_TX_PIN           (4)
//#define I2C_SCL_PIN          (3)
//#define I2C_SDA_PIN          (2)

/* Analog pins */
#define TMP36_PIN            (A0)

#define IMAGE_SIZE_LARGE     (3)
#define IMAGE_SIZE_MEDIUM    (2)
#define IMAGE_SIZE_SMALL     (1)

struct _payload {
  uint8_t sentence_id;
  unsigned long running_millis;
  uint8_t   gps_hour;
  uint8_t   gps_minute;
  uint8_t   gps_seconds;
  uint16_t  gps_milliseconds;
  uint8_t   gps_year;
  uint8_t   gps_month;
  uint8_t   gps_day;
  float gps_latitude;
  float gps_longitude;
  float gps_altitude;
  float gps_previous_altitude;
  float gps_geoidheight;
  float internal_temperature;
  boolean descending;
};

#endif

