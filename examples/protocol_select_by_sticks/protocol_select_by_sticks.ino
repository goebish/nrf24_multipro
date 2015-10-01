/*
 ##########################################
 #####   MultiProtocol nRF24L01 Tx   ######
 ##########################################
 #        by goebish on rcgroups          #
 #                                        #
 #   Parts of this project are derived    #
 #     from existing work, thanks to:     #
 #                                        #
 #   - PhracturedBlue for DeviationTX     #
 #   - victzh for XN297 emulation layer   #
 #   - Hasi for Arduino PPM decoder       #
 #   - hexfet, midelic, closedsink ...    #
 ##########################################


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

#include <util/atomic.h>
#include <Arduino.h>
#include <EEPROM.h>

#include <nrf24_multipro.h>

#ifndef SOFTSPI
#include <SPI.h>
#endif


#define PPM_pin         2
#define PPM_CHANNELS    12

// PPM output values of remote control
#define PPM_RC_MIN      1000
#define PPM_RC_MAX      2000

#if F_CPU == 16000000
#define PPM_SCALE 1L
#elif F_CPU == 8000000
#define PPM_SCALE 0L
#else
#error // 8 or 16MHz only !
#endif

#define CH_BIND (CH_MAX_CONTROL)
#define CH_MAX  (CH_MAX_CONTROL+1)




/*
// use this style to setup own pin mapping

#define led_Pin   13  // LED  - D13

//SPI Comm.pins with nRF24L01
#define MOSI_pin  5   // MOSI - D5
#define SCK_pin   4   // SCK  - D4
#define MISO_pin  6   // MISO - D6

#define CE_pin    A3  // CE   - A3
#define CS_pin    8   // CS   - D8

nrf24_multipro multipro = nrf24_multipro(MOSI_pin, SCK_pin, MISO_pin, CE_pin, CS_pin, led_Pin);
*/

nrf24_multipro multipro;

// CH 1-11   control
// CH 12     bind / reset
// CH 13     set Mode
static uint16_t ppm[CH_MAX] = { 0 };
static volatile uint16_t ppm_irq[CH_MAX] = { 0 };
static volatile bool ppm_ok = false;
static unsigned long last_ppm = 0;

static bool reset = true;


void ppm_update(void);
void ISR_ppm(void);
void selectProtocol(void);

void setup(void) {
    Serial.begin(115200);

    DEBUG_MULTI(F("\n\n\n"));

    // pullup to allow open drain remotes
    pinMode(PPM_pin, INPUT_PULLUP);

    // PPM ISR setup
    attachInterrupt(PPM_pin - 2, ISR_ppm, CHANGE);
    TCCR1A = 0;  //reset timer1
    TCCR1B = 0;
    TCCR1B |= (1 << CS11);  //set timer1 to increment every 1 us @ 8MHz, 0.5 us @16MHz

    DEBUG_MULTI(F("wait for remote...\n"));
    while(!ppm_ok) {}
    ppm_update();

    DEBUG_MULTI(F("wait for throttle low...\n"));
    while(ppm[CH_THROTTLE] > PPM_SAFE_THROTTLE) {
        ppm_update();
    }


    multipro.setChannelNum(PPM_CHANNELS);
    multipro.begin();

}

void loop(void) {
    if(ppm_ok) {
        ppm_update();

        if(reset || ppm[CH_BIND] > PPM_MAX_COMMAND) {
            reset = false;
            DEBUG_MULTI(F("wait for bind / reset low...\n"));
            while(ppm[CH_BIND] > PPM_MAX_COMMAND) {
                ppm_update();
            }

            selectProtocol();

            multipro.reset();
        }


        for(uint8_t ch = 0; ((ch < CH_MAX_CONTROL) && (ch < PPM_CHANNELS)); ch++) {
#if ((PPM_RC_MIN != PPM_MIN ) || (PPM_RC_MAX != PPM_MAX))
            multipro.setChannel((t_channelOrder) ch, map(ppm[ch], PPM_RC_MIN, PPM_RC_MAX, PPM_MIN, PPM_MAX));
#else
            multipro.setChannel((t_channelOrder) ch, ppm[ch]);
#endif
        }

        ppm_update();
    }

    /**
     * lost connection to remote control
     * kill the quad connection for safety
     */
    if(last_ppm && millis() - last_ppm > 10000) {
        DEBUG_MULTI(F("remote control connection lost!\n"));
        multipro.reset();

        DEBUG_MULTI(F("wait for remote...\n"));
        while((millis() - last_ppm > 10000)) {
            ppm_update();
        }

        DEBUG_MULTI(F("remote control connection found! reconnect...\n"));
        multipro.reset();
        ppm_update();
    }

    multipro.loop();
}

void ppm_update(void) {
    if(ppm_ok) {
        last_ppm = millis();
        for(uint8_t ch = 0; ch < PPM_CHANNELS; ch++) {
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
            {
                ppm[ch] = ppm_irq[ch];
            }
        }
        ppm_ok = false;
    }
}

void ISR_ppm(void) {
    static unsigned int pulse;
    static unsigned long counterPPM;
    static byte chan;
    counterPPM = TCNT1;
    TCNT1 = 0;
    ppm_ok = false;
    if(counterPPM < 510 << PPM_SCALE) {  //must be a pulse if less than 510us
        pulse = counterPPM;
    } else if(counterPPM > 1910 << PPM_SCALE) {  //sync pulses over 1910us
        chan = 0;
    } else {  //servo values between 510us and 2420us will end up here
        if(chan < PPM_CHANNELS) {
            ppm_irq[chan] = constrain((counterPPM + pulse) >> PPM_SCALE, PPM_RC_MIN, PPM_RC_MAX);
            if(chan == PPM_CHANNELS - 1) {
                ppm_ok = true; // complete ppm frame
            }
        }
        chan++;
    }
}

void selectProtocol(void)
{

    t_protocols current_protocol;

    // wait for multiple complete ppm frames
    ppm_ok = false;
    uint8_t count = 10;
    while(count) {
        while(!ppm_ok) {} // wait
        ppm_update();
        if(ppm[CH_BIND] < PPM_MAX_COMMAND) // reset chan released
            count--;
        ppm_ok = false;
    }
    
    // startup stick commands
    
    if(ppm[CH_RUDDER] < PPM_MIN_COMMAND)        // Rudder left
        multipro.set_txid(true);                      // Renew Transmitter ID
    
    // protocol selection
    
    // Elevator down + Aileron right
    if(ppm[CH_ELEVATOR] < PPM_MIN_COMMAND && ppm[CH_AILERON] > PPM_MAX_COMMAND)
        current_protocol = PROTO_YD829; // YD-829, YD-829C, YD-822 ...
    
    // Elevator down + Aileron left
    else if(ppm[CH_ELEVATOR] < PPM_MIN_COMMAND && ppm[CH_AILERON] < PPM_MIN_COMMAND)
        current_protocol = PROTO_SYMAX5C1; // Syma X5C-1, X11, X11C, X12
    
    // Elevator up + Aileron right
    else if(ppm[CH_ELEVATOR] > PPM_MAX_COMMAND && ppm[CH_AILERON] > PPM_MAX_COMMAND)
        current_protocol = PROTO_BAYANG;    // EAchine H8 mini, BayangToys X6/X7/X9, JJRC JJ850 ...
    
    // Elevator up + Aileron left
    else if(ppm[CH_ELEVATOR] > PPM_MAX_COMMAND && ppm[CH_AILERON] < PPM_MIN_COMMAND)
        current_protocol = PROTO_H7;        // EAchine H7, MT99xx
    
    // Elevator up  
    else if(ppm[CH_ELEVATOR] > PPM_MAX_COMMAND)
        current_protocol = PROTO_V2X2;       // WLToys V202/252/272, JXD 385/388, JJRC H6C ...
        
    // Elevator down
    else if(ppm[CH_ELEVATOR] < PPM_MIN_COMMAND)
        current_protocol = PROTO_CG023;      // EAchine CG023/CG031/3D X4, (todo :ATTOP YD-836/YD-836C) ...
    
    // Aileron right
    else if(ppm[CH_AILERON] > PPM_MAX_COMMAND)
        current_protocol = PROTO_CX10_BLUE;  // Cheerson CX10(blue pcb, newer red pcb)/CX10-A/CX11/CX12 ... 
    
    // Aileron left
    else if(ppm[CH_AILERON] < PPM_MIN_COMMAND)
        current_protocol = PROTO_CX10_GREEN;  // Cheerson CX10(green pcb)... 
    
    // read last used protocol from eeprom
    else 
        current_protocol = (t_protocols) constrain(EEPROM.read(ee_PROTOCOL_ID),0,PROTO_END-1);
    // update eeprom 

    multipro.setProtocol(current_protocol);

    // wait for safe throttle
    while(ppm[CH_THROTTLE] > PPM_SAFE_THROTTLE) {
        delay(100);
        ppm_update();
    }
}
