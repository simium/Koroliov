
/* Project Swift - High altitude balloon flight software                 */
/*=======================================================================*/
/* Copyright 2010-2012 Philip Heron <phil@sanslogic.co.uk>               */
/*                                                                       */
/* This program is free software: you can redistribute it and/or modify  */
/* it under the terms of the GNU General Public License as published by  */
/* the Free Software Foundation, either version 3 of the License, or     */
/* (at your option) any later version.                                   */
/*                                                                       */
/* This program is distributed in the hope that it will be useful,       */
/* but WITHOUT ANY WARRANTY; without even the implied warranty of        */
/* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         */
/* GNU General Public License for more details.                          */
/*                                                                       */
/* You should have received a copy of the GNU General Public License     */
/* along with this program.  If not, see <http://www.gnu.org/licenses/>. */

#include "config.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <string.h>
#include "rtty.h"
#include "timeout.h"
#include "Arduino.h"
#include <avr/wdt.h>

volatile static uint8_t  txpgm = 0;
volatile static uint8_t *txbuf = 0;
volatile static uint16_t txlen = 0;

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

  for (i=0;i<8;i++) // Change this here 7 or 8 for ASCII-7 / ASCII-8
  {
    if (c & 1) rtty_txbit(1);

    else rtty_txbit(0);

    c = c >> 1;

  }
  rtty_txbit (1); // Stop bit
  rtty_txbit (1); // Stop bit
  
  wdt_reset();
}

void rtty_txbit (int bit)
{
  if (bit)
  {
    // high
    analogWrite(RADIO_TX_PIN,110);
  }
  else
  {
    // low
    analogWrite(RADIO_TX_PIN,100);

  }

  //delayMicroseconds(6800); // 150 baud
  if (RTTY_BAUD == 50) {
    delayMicroseconds(10000);
    delayMicroseconds(10150);
  }
  else if (RTTY_BAUD == 100) {
    delayMicroseconds(10000/2);
    delayMicroseconds(10150/2);
  }
  else if (RTTY_BAUD == 300) {
        delayMicroseconds(3380);
  }
  else { // Assume 50 baud
    delayMicroseconds(10000);
    delayMicroseconds(10150);
  }
  // largest value that will produce an accurate delay is 16383
  // See : http://arduino.cc/en/Reference/DelayMicroseconds

}

void rtx_data(uint8_t *data, size_t length)
{

  /* Simple function to sent a char at a time to
   ** rtty_txbyte function.
   ** NB Each char is one byte (8 Bits)
   */

  uint8_t c;
  size_t i;

  c = *data++;

  for (i=0;i<length;i++)
  {
    rtty_txbyte ((char)c);
    c = *data++;
  }
}

void rtx_string(char *s)
{

  /* Simple function to sent a char at a time to
   ** rtty_txbyte function.
   ** NB Each char is one byte (8 Bits)
   */

  char c;

  c = *s++;

  while ( c != '\0')
  {
    rtty_txbyte (c);
    c = *s++;
  }
}

