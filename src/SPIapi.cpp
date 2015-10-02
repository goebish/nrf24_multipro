/*
 * SPIapi.cpp
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License.
 * If not, see <http://www.gnu.org/licenses/>.
 */
#include <Arduino.h>

#include "nrf24_multipro.h"

#ifndef SOFTSPI
#include <SPI.h>
#endif

#include "SPIapi.h"


/**
 * PIN setup
 */

// store the pin number
int pinCE, pinCS;

#if defined (__AVR__) || defined(TEENSYDUINO)
volatile uint8_t *pinCEport, *pinCSport;
uint8_t pinCEmask, pinCSmask;


#define CE_on       *pinCEport |= pinCEmask
#define CE_off      *pinCEport &= ~pinCEmask

#define CS_on       *pinCSport |= pinCSmask
#define CS_off      *pinCSport &= ~pinCSmask


#elif defined (ESP8266)
uint32_t pinCEmask, pinCSmask;

#define CE_on       GPOS = pinCEmask
#define CE_off      GPOC = pinCEmask
#define CS_on       GPOS = pinCSmask
#define CS_off      GPOC = pinCSmask

#else

#error not implemented plattform

#endif


#ifdef SOFTSPI

// store the pin number
int pinMOSI, pinSCK, pinMISO;

#if defined (__AVR__) || defined(TEENSYDUINO)
volatile uint8_t *pinMOSIport, *pinSCKport, *pinMISOport;
uint8_t pinMOSImask, pinSCKmask, pinMISOmask;

// SPI outputs
#define MOSI_on     *pinMOSIport |= pinMOSImask
#define MOSI_off    *pinMOSIport &= ~pinMOSImask

#define SCK_on      *pinSCKport |= pinSCKmask
#define SCK_off     *pinSCKport &= ~pinSCKmask

// SPI input
#define  MISO_on    ((*pinMISOport & pinMISOmask) != 0)

#elif defined (ESP8266)

uint32_t pinMOSImask, pinSCKmask, pinMISOmask;

// SPI outputs
#define MOSI_on     GPOS = pinMOSImask
#define MOSI_off    GPOC = pinMOSImask
#define SCK_on      GPOS = pinSCKmask
#define SCK_off     GPOC = pinSCKmask

// SPI input
#define  MISO_on    ((GPI & pinMISOmask) != 0)

#else
#error not implemented plattform
#endif
#endif

#ifdef SOFTSPI
void spi_config(int _pinMOSI, int _pinSCK, int _pinMISO, int _pinCE, int _pinCS) {

    pinMOSI = _pinMOSI;
    pinSCK = _pinSCK;
    pinMISO = _pinMISO;

#else
void spi_config(int _pinCE, int _pinCS) {
#endif

    pinCE = _pinCE;
    pinCS = _pinCS;

#if defined (__AVR__) || defined(TEENSYDUINO)

#ifdef SOFTSPI
    pinMOSIport = portOutputRegister(digitalPinToPort(pinMOSI));
    pinMOSImask = digitalPinToBitMask(pinMOSI);

    pinSCKport = portOutputRegister(digitalPinToPort(pinSCK));
    pinSCKmask = digitalPinToBitMask(pinSCK);

    pinMISOport = portInputRegister(digitalPinToPort(pinMISO));
    pinMISOmask = digitalPinToBitMask(pinMISO);
#endif

    pinCEport = portOutputRegister(digitalPinToPort(pinCE));
    pinCEmask = digitalPinToBitMask(pinCE);

    pinCSport = portOutputRegister(digitalPinToPort(pinCS));
    pinCSmask = digitalPinToBitMask(pinCS);

#elif defined (ESP8266)

#ifdef SOFTSPI
   pinMOSImask = digitalPinToBitMask(pinMOSI);
   pinSCKmask = digitalPinToBitMask(pinSCK);
   pinMISOmask = digitalPinToBitMask(pinMISO);
#endif

   pinCEmask = digitalPinToBitMask(pinCE);
   pinCSmask = digitalPinToBitMask(pinCS);

#endif

}

void spi_begin(void) {
    pinMode(pinCS, OUTPUT);
    pinMode(pinCE, OUTPUT);

#ifdef SOFTSPI
    pinMode(pinMOSI, OUTPUT);
    pinMode(pinSCK, OUTPUT);
    pinMode(pinMISO, INPUT);
#else
    SPI.begin();
#endif
}

uint8_t spi_read(void) {
#ifdef SOFTSPI
    uint8_t result = 0;
    uint8_t i;
    MOSI_off;
    _NOP();
    for(i = 0; i < 8; i++) {
        if(MISO_on) { // if MISO is HIGH
            result = (result << 1) | 0x01;
        } else {
            result = result << 1;
        }
        SCK_on;
        _NOP();
        SCK_off;
        _NOP();
    }
    return result;
#else
    return SPI.transfer(0x00);
#endif
}

uint8_t spi_write(uint8_t command) {
#ifdef SOFTSPI
    uint8_t result = 0;
    uint8_t n = 8;
    SCK_off;
    MOSI_off;
    while(n--) {
        if(command & 0x80) {
            MOSI_on;
        } else {
            MOSI_off;
        }
        if(MISO_on) {
            result |= 0x01;
        }
        SCK_on;
        _NOP();
        SCK_off;
        command = command << 1;
        result = result << 1;
    }
    MOSI_on;
    return result;
#else
    return SPI.transfer(command);
#endif
}

void spi_read_bytes(uint8_t *data, uint8_t length) {
    uint8_t i;
    CS_off;
    spi_write(0x61); // Read RX payload
    for(i = 0; i < length; i++) {
        data[i] = spi_read();
    }
    CS_on;

}

void spi_write_address(uint8_t address, uint8_t data) {
    CS_off;
    spi_write(address);
    _NOP();
    spi_write(data);
    CS_on;
}

uint8_t spi_read_address(uint8_t address) {
    uint8_t result;
    CS_off;
    spi_write(address);
    result = spi_read();
    CS_on;
    return (result);
}


uint8_t spi_write_byte(uint8_t byte) {
    uint8_t result;
    CS_off;
    result = spi_write(byte);
    CS_on;
    return result;
}

void spi_write_address_bytes(uint8_t address, uint8_t * data, uint8_t len) {
    CS_off;
    spi_write(address);
    for(uint8_t i = 0; i < len; i++) {
        spi_write(data[i]);
    }
    CS_on;
}

void spi_read_address_bytes(uint8_t address, uint8_t * data, uint8_t len) {
    CS_off;
    spi_write(address);
    for(uint8_t i = 0; i < len; i++) {
        data[i] = spi_read();
    }
    CS_on;
}

void spi_CE_on(void) {
    CE_on;
}

void spi_CE_off(void) {
    CE_off;
}
