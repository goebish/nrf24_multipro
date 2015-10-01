/*
 * multiRemote.cpp
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
#include <EEPROM.h>
#include "nrf24_multipro.h"
#include "SPIapi.h"

/**
 * global transmitterID
 * @todo use function
 */
uint8_t transmitterID[4];


/**
 * LED
 * @todo use function
 */
int pinLED;

/**
 * constractor
 */
#ifdef SOFTSPI
nrf24_multipro::nrf24_multipro(int _pinMOSI, int _pinSCK, int _pinMISO, int _pinCE, int _pinCS, int _pinLED) {
#else
nrf24_multipro::nrf24_multipro(int _pinCE, int _pinCS, int _pinLED) {
#endif
    pinLED = _pinLED;

    current_protocol = PROTO_V2X2;
    channel_num = CH_MAX_CONTROL;

#ifdef SOFTSPI
    spi_config(_pinMOSI, _pinSCK, _pinMISO, _pinCE, _pinCS);
#else
    spi_config(_pinCE, _pinCS);
#endif

}

/**
 * called on startup
 */
void nrf24_multipro::begin(t_protocols protocol) {

    randomSeed((analogRead(A4) & 0x1F) | (analogRead(A5) << 5));

    pinMode(pinLED, OUTPUT);
    digitalWrite(pinLED, LOW); //start LED off

    spi_begin();

    DEBUG_MULTI("[MR] set_txid...\n");

    set_txid(false);

    current_protocol = protocol;

    DEBUG_MULTI("[MR] current_protocol: ");
    DEBUG_MULTI(current_protocol);
    DEBUG_MULTI("\n");

    reset();
    DEBUG_MULTI("[MR] begin done.\n");

}

/**
 *
 */
void nrf24_multipro::begin(void) {
    begin((t_protocols) constrain(EEPROM.read(ee_PROTOCOL_ID), 0, PROTO_END - 1));
}

/**
 * reset the Protocol + RF
 */
void nrf24_multipro::reset(bool bind) {

    /**
     * reset ppm values
     */
    for(uint8_t ch = 0; ch < CH_MAX_CONTROL; ch++) {
        ppm[ch] = PPM_MID;
    }

    ppm[CH_THROTTLE] = PPM_MIN;

    initRF();

    if(bind) {
        initProt();
    }
}

/**
 * called from arduino loop
 */
void nrf24_multipro::loop(void) {
    static unsigned long next;
    if(micros() >= next) {
        protAPI * protocol = protocols[current_protocol];
        next = protocol->loop();
    }
}

/**
 *
 * @return t_protocols
 */
t_protocols nrf24_multipro::getProtocol(void) {
    return current_protocol;
}

/**
 * switch protocol
 * @param protocol t_protocols
 */
void nrf24_multipro::setProtocol(t_protocols protocol) {

    if(current_protocol == protocol) {
        return;
    }

    DEBUG_MULTI("[MR] setProtocol to: ");
    DEBUG_MULTI(current_protocol);
    DEBUG_MULTI("\n");
    current_protocol = protocol;

    // update eeprom
    EEPROM.update(ee_PROTOCOL_ID, current_protocol);

    reset();
}

/**
 * get channel num
 * @return number of channels of Remote
 */
uint8_t nrf24_multipro::getChannelNum(void) {
    return channel_num;
}

/**
 * sets the  channels from the Remote
 * @param num uint8_t
 */
void nrf24_multipro::setChannelNum(uint8_t num) {
    channel_num = num;

    if(channel_num >= CH_MAX_CONTROL) {
        channel_num = (CH_MAX_CONTROL - 1);
    }
}

/**
 * get channel value
 * @param ch t_channelOrder
 * @return channel value
 */
uint16_t nrf24_multipro::getChannel(t_channelOrder ch) {
    if(ch < CH_MAX_CONTROL) {
        return ppm[ch];
    }
    return 0;
}

/**
 *
 * @param ch t_channelOrder
 * @param out_min
 * @param out_max
 * @return channel value after map
 */
long nrf24_multipro::getChannel(t_channelOrder ch, long out_min, long out_max) {
    if(ch < CH_MAX_CONTROL) {
        return map(ppm[ch], PPM_MIN, PPM_MAX, out_min, out_max);
    }
    return out_min;
}

/**
 * 2way switch decode
 * @param ch t_channelOrder
 * @return true if channel value is > PPM_MAX_COMMAND
 */
bool nrf24_multipro::getChannelIsCMD(t_channelOrder ch) {
    if(ch < CH_MAX_CONTROL) {
        return (ppm[ch] > PPM_MAX_COMMAND);
    }
    return false;
}

/**
 * 3way switch decode
 * @param ch
 * @return -1 < PPM_MIN_COMMAND   1 > PPM_MAX_COMMAND  or  0
 */
int8_t nrf24_multipro::getChannel3way(t_channelOrder ch) {
    if(ch < CH_MAX_CONTROL) {
        if(ppm[ch] < PPM_MIN_COMMAND) {
            return -1;
        } else if(ppm[ch] > PPM_MAX_COMMAND) {
            return 1;
        }
    }
    return 0;
}

/**
 *
 * @param ch t_channelOrder
 * @param value uint16_t
 */
void nrf24_multipro::setChannel(t_channelOrder ch, uint16_t value) {
    if(ch < CH_MAX_CONTROL) {
        ppm[ch] = value;
    }
}

/**
 * init NRF24 (RF)
 */
void nrf24_multipro::initRF(void) {

    DEBUG_MULTI("[MR] initRF...\n");
    NRF24L01_Reset();
    NRF24L01_Initialize();
    DEBUG_MULTI("[MR] initRF... Done.\n");
}

/**
 * init Protocol
 */
void nrf24_multipro::initProt(void) {

    DEBUG_MULTI("[MR] initProt...\n");
    protAPI * protocol = protocols[current_protocol];
    protocol->init();
    protocol->bind();
    DEBUG_MULTI("[MR] initProt... Done.\n");

}

/**
 *
 * @param renew bool
 */
void nrf24_multipro::set_txid(bool renew) {
    uint8_t i;
    for(i = 0; i < 4; i++) {
        transmitterID[i] = EEPROM.read(ee_TXID0 + i);
    }
    if(renew || (transmitterID[0] == 0xFF && transmitterID[1] == 0x0FF)) {
        for(i = 0; i < 4; i++) {
            transmitterID[i] = random() & 0xFF;
            EEPROM.update(ee_TXID0 + i, transmitterID[i]);
        }
    }
}

