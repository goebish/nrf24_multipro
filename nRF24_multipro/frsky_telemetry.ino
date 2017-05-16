/*
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License.
 If not, see <http://www.gnu.org/licenses/>.
 */

/* 
 FrSky Telemetry for OpenTx
 The UART TX-Pin is connected to Pin5 of the JR-Connector. FrSky uses inverted serial levels 
 and thus a inverter is needed in between. Example circuit can be found here:
 https://github.com/openLRSng/openLRSng/wiki/Telemetry-guide
 */

#include "frsky_telemetry.h"

#define SMARTPORT_INTERVAL 12000
#define SMARTPORT_BAUDRATE 57600

uint32_t frskyLast = 0;
uint8_t frskySchedule = 0;

void frskyInit()
{
  frskyLast = micros();
  Serial.begin(SMARTPORT_BAUDRATE);
}

void smartportSend(uint8_t *p)
{
  uint16_t crc = 0;
  Serial.write(0x7e);
  for (int i = 0; i < 9; i++) {
    if (i == 8) {
      p[i] = 0xff - crc;
    }
    if ((p[i] == 0x7e) || (p[i] == 0x7d)) {
      Serial.write(0x7d);
      Serial.write(0x20 ^ p[i]);
    } 
    else {
      Serial.write(p[i]);
    }
    if (i>0) {
      crc += p[i]; //0-1FF
      crc += crc >> 8; //0-100
      crc &= 0x00ff;
      crc += crc >> 8; //0-0FF
      crc &= 0x00ff;
    }
  }
}

void smartportIdle()
{
  Serial.write(0x7e);
}

void smartportSendFrame()
{
  uint8_t buf[9];
  
  uint8_t * bytes;
  uint32_t temp;
  
  frskySchedule = (frskySchedule + 1) % 3;
  buf[0] = 0x98;
  buf[1] = 0x10;
  switch (frskySchedule) {
  case 0: // RSSI
    temp = constrain(telemetry_data.rssi,0,100);
    bytes = (uint8_t *) &temp;
    buf[2] = RSSI_ID & 0xff;
    buf[3] = RSSI_ID >> 8;
    buf[4] = bytes[0];
    buf[5] = bytes[1];
    break;
  case 1: // VBAT
    temp = telemetry_data.volt1;
    bytes = (uint8_t *) &temp;
    buf[2] = VFAS_FIRST_ID & 0xff;
    buf[3] = VFAS_FIRST_ID >> 8;
    buf[4] = bytes[0];
    buf[5] = bytes[1];
    buf[6] = bytes[2];
    buf[7] = bytes[3];
    break;
  default:
    smartportIdle();
    return;
  }
  smartportSend(buf);
}

void frskyUpdate()
{
  uint32_t now = micros();
  if ((now - frskyLast) > SMARTPORT_INTERVAL) {
    smartportSendFrame();
    frskyLast = now;
  }
}


