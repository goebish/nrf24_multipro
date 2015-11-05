/*
 * multiRemote.h
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


#ifndef MULTIREMOTE_H_
#define MULTIREMOTE_H_

#include "protocol.h"

#define SOFTSPI

#define DEBUG_MULTI(text) Serial.print(text);

#define RF_POWER 2 // 0-3, it was found that using maximum power can cause some issues, so let's use 2...

// PPM stream settings
typedef enum {
    CH_THROTTLE,    //!< (CH1)
    CH_AILERON,     //!< (CH2)
    CH_ELEVATOR,    //!< (CH3)
    CH_RUDDER,      //!< (CH4)
    CH_3WAY,        //!< (CH5)  led light, or 3 pos. rate on CX-10, H7, or inverted flight on H101 (Bayang), RTH (Bayang)
    CH_FLIP,        //!< (CH6)  flip control
    CH_PIC,         //!< (CH7)  sill camera
    CH_CAM,         //!< (CH8)  video camera
    CH_HEADLESS,    //!< (CH9)  headless
    CH_AUX6,        //!< (CH10) calibrate Y (V2x2), pitch trim (H7)
    CH_AUX7,        //!< (CH11) calibrate X (V2x2), roll trim (H7)
    CH_MAX_CONTROL  //!< 11 channel for all controles used
} t_channelOrder;

// EEPROM locations
typedef enum {
    ee_PROTOCOL_ID = 0,
    ee_TXID0,
    ee_TXID1,
    ee_TXID2,
    ee_TXID3
} t_eepromAddrs;

#define PPM_MIN 1000
#define PPM_MAX 2000
#define PPM_MID ((( PPM_MAX - PPM_MIN ) / 2 ) + PPM_MIN)
#define PPM_SAFE_THROTTLE 1050
#define PPM_MIN_COMMAND 1300
#define PPM_MAX_COMMAND 1700

class nrf24_multipro {
    public:
#ifdef SOFTSPI
        nrf24_multipro(int _pinMOSI = 3, int _pinSCK = 4, int _pinMISO = A0, int _pinCE = 5, int _pinCS = A1, int _pinLED = 13);
#else
        nrf24_multipro(int _pinCE = A0, int _pinCS = A1, int _pinLED = A2);
#endif

        void begin(void);
        void begin(t_protocols protocol);
        void loop(void);

        void reset(bool bind = true);

        t_protocols getProtocol(void);
        void setProtocol(t_protocols protocol);

        uint8_t getChannelNum(void);
        void setChannelNum(uint8_t num);

        uint16_t getChannel(t_channelOrder ch);
        long getChannel(t_channelOrder ch, long out_min, long out_max);

        bool getChannelIsCMD(t_channelOrder ch);
        int8_t getChannel3way(t_channelOrder ch);

        void setChannel(t_channelOrder ch, uint16_t value);

        void set_txid(bool renew);

    private:

        void initRF(void);
        void initProt(void);

        t_protocols current_protocol;
        uint16_t ppm[CH_MAX_CONTROL];
        uint8_t channel_num;

};

extern uint8_t transmitterID[4];
extern nrf24_multipro multipro;
extern int pinLED;

#endif /* MULTIREMOTE_H_ */
