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
#include <EEPROM.h>
#include "iface_nrf24l01.h"


// ############ Wiring ################
#define PPM_pin   2  // PPM in
//SPI Comm.pins with nRF24L01
#define MOSI_pin  3  // MOSI - D3
#define SCK_pin   4  // SCK  - D4
#define CE_pin    5  // CE   - D5
#define MISO_pin  A0 // MISO - A0
#define CS_pin    A1 // CS   - A1

#define ledPin    13 // LED  - D13

// SPI outputs
#define MOSI_on PORTD |= _BV(3)  // PD3
#define MOSI_off PORTD &= ~_BV(3)// PD3
#define SCK_on PORTD |= _BV(4)   // PD4
#define SCK_off PORTD &= ~_BV(4) // PD4
#define CE_on PORTD |= _BV(5)    // PD5
#define CE_off PORTD &= ~_BV(5)  // PD5
#define CS_on PORTC |= _BV(1)    // PC1
#define CS_off PORTC &= ~_BV(1)  // PC1
// SPI input
#define  MISO_on (PINC & _BV(0)) // PC0

// PPM stream settings
#define CHANNELS 12 // number of channels in ppm stream, 12 ideally
enum chan_order{
    THROTTLE,
    AILERON,
    ELEVATOR,
    RUDDER,
    AUX1,  // (CH5)  led light (3 pos. rate on Cheersons)
    AUX2,  // (CH6)  flip control
    AUX3,  // (CH7)  sill camera
    AUX4,  // (CH8)  video camera
    AUX5,  // (CH9)  headless
    AUX6,  // (CH10) calibrate Y (V2x2)
    AUX7,  // (CH11) calibrate X (V2x2)
    AUX8,  // (CH12) Reset / Rebind
};

#define PPM_MIN 1000
#define PPM_SAFE_THROTTLE 1050 
#define PPM_MID 1500
#define PPM_MAX 2000
#define PPM_MIN_COMMAND 1300
#define PPM_MAX_COMMAND 1700

// supported protocols
enum {
    PROTO_V2X2 = 0,
    PROTO_CG023,
    PROTO_CX10_BLUE,
    PROTO_CX10_GREEN,
    PROTO_XN297RX,
    PROTO_END
};

// EEPROM locations
enum{
    ee_PROTOCOL_ID = 0,
    ee_TXID0,
    ee_TXID1,
    ee_TXID2,
    ee_TXID3
};

uint8_t transmitterID[4];
uint8_t current_protocol;
static volatile bool ppm_ok = false;
uint8_t packet[32];
static bool reset=true;
volatile uint16_t Servo_data[12];
static uint16_t ppm[12] = {PPM_MIN,PPM_MIN,PPM_MIN,PPM_MIN,PPM_MIN,PPM_MIN,
                           PPM_MIN,PPM_MIN,PPM_MIN,PPM_MIN,PPM_MIN,PPM_MIN,};

void setup()
{
    randomSeed((analogRead(A4) & 0x1F) | (analogRead(A5) << 5));
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW); //start LED off
    pinMode(PPM_pin, INPUT);
    pinMode(MOSI_pin, OUTPUT);
    pinMode(SCK_pin, OUTPUT);
    pinMode(CS_pin, OUTPUT);
    pinMode(CE_pin, OUTPUT);
    pinMode(MISO_pin, INPUT);
     
    // PPM ISR setup
    attachInterrupt(PPM_pin - 2, ISR_ppm, CHANGE);
    TCCR1A = 0;  //reset timer1
    TCCR1B = 0;
    TCCR1B |= (1 << CS11);  //set timer1 to increment every 1 us @ 8MHz, 0.5 us @16MHz

    set_txid(false);
}

void loop()
{
    uint32_t timeout;
    // reset / rebind
    if(reset || ppm[AUX8] > PPM_MAX_COMMAND) {
        reset = false;
        selectProtocol();
        NRF24L01_Reset();
        NRF24L01_Initialize();
        init_protocol();
    }
    // process protocol
    switch(current_protocol) {
        case PROTO_CG023:
        timeout = process_CG023();
        break;
        case PROTO_V2X2:
        timeout = process_V2x2();
        break;
        case PROTO_CX10_GREEN:
        case PROTO_CX10_BLUE:
        timeout = process_CX10();
        break;
    }
    // updates ppm values out of ISR
    update_ppm();
    // wait before sending next packet
    while(micros() < timeout)
    {   };
}

void set_txid(bool renew)
{
    uint8_t i;
    for(i=0; i<4; i++)
        transmitterID[i] = EEPROM.read(ee_TXID0+i);
    if(renew || (transmitterID[0]==0xFF && transmitterID[1]==0x0FF)) {
        for(i=0; i<4; i++) {
            transmitterID[i] = random() & 0xFF;
            EEPROM.update(ee_TXID0+i, transmitterID[i]); 
        }            
    }
}

void selectProtocol()
{
    // wait for multiple complete ppm frames
    ppm_ok = false;
    uint8_t count = 10;
    while(count) {
        while(!ppm_ok) {} // wait
        update_ppm();
        if(ppm[AUX8] < PPM_MAX_COMMAND) // reset chan released
            count--;
        ppm_ok = false;
    }
    
    // startup stick commands
    
    if(ppm[RUDDER] < PPM_MIN_COMMAND)        // Rudder left
        set_txid(true);                      // Renew Transmitter ID
    
    // protocol selection
    if(ppm[ELEVATOR] > PPM_MAX_COMMAND)      // Elevator up  
        current_protocol = PROTO_V2X2;       // WLToys V202/252/272, JXD 385/388 ...
        
    else if(ppm[ELEVATOR] < PPM_MIN_COMMAND) // Elevator down
        current_protocol = PROTO_CG023;      // EAchine CG023/CG031/3D X4, (todo :ATTOP YD-836/YD-836C) ...
        
    else if(ppm[AILERON] > PPM_MAX_COMMAND)  // Aileron right
        current_protocol = PROTO_CX10_BLUE;  // Cheerson CX10(blue pcb)/CX10-A/CX11/CX12 ... 
    
    else if(ppm[AILERON] < PPM_MIN_COMMAND)  // Aileron left
        current_protocol = PROTO_CX10_GREEN;  // Cheerson CX10(green pcb)... 
    
    // read last used protocol from eeprom
    else 
        current_protocol = constrain(EEPROM.read(ee_PROTOCOL_ID),0,PROTO_END-1);      
    // update eeprom 
    EEPROM.update(ee_PROTOCOL_ID, current_protocol);
    // wait for safe throttle
    while(ppm[THROTTLE] > PPM_SAFE_THROTTLE) {
        delay(100);
        update_ppm();
    }
}

void init_protocol()
{
    switch(current_protocol) {
        case PROTO_CG023:
            CG023_init();
            CG023_bind();
            break;
        case PROTO_V2X2:
            V2x2_init();
            V2x2_bind();
            break;
        case PROTO_CX10_GREEN:
        case PROTO_CX10_BLUE:
            CX10_init();
            CX10_bind();
            break;
    }
}

// update ppm values out of ISR    
void update_ppm()
{
    for(uint8_t ch=0; ch<CHANNELS; ch++) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            ppm[ch] = Servo_data[ch];
        }
    }    
}

void ISR_ppm()
{
    #if F_CPU == 16000000
        #define PPM_SCALE 1L
    #elif F_CPU == 8000000
        #define PPM_SCALE 0L
    #else
        #error // 8 or 16MHz only !
    #endif
    static unsigned int pulse;
    static unsigned long counterPPM;
    static byte chan;
    counterPPM = TCNT1;
    TCNT1 = 0;
    ppm_ok=false;
    if(counterPPM < 510 << PPM_SCALE) {  //must be a pulse if less than 510us
        pulse = counterPPM;
    }
    else if(counterPPM > 1910 << PPM_SCALE) {  //sync pulses over 1910us
        chan = 0;
    }
    else{  //servo values between 510us and 2420us will end up here
        if(chan < CHANNELS) {
            Servo_data[chan]= constrain((counterPPM + pulse) >> PPM_SCALE, PPM_MIN, PPM_MAX);
            if(chan==CHANNELS-1)
                ppm_ok = true; // complete ppm frame
        }
        chan++;
    }
}
