#ifndef __CONFIG_H__
#define __CONFIG_H__

#define RTTY_CALLSIGN "KRLV"

/* CONFIGURABLE BITS */
#define ASCII 8          // ASCII 7 or 8
#define STOPBITS 2       // Either 1 or 2
#define TXDELAY 0        // Delay between sentence TX's
#define RTTY_BAUD 300     // Baud rate for use with RFM22B Max = 600

#define TX_DELAY      300   // default was 300

#define RADIO_FREQUENCY      (434650000UL)

/* Digital pins */
#define RADIO_TX_PIN         (9)
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

#endif

