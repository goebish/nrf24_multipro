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

#define RF_POWER TX_POWER_80mW 

// tune ppm input for "special" transmitters
// #define SPEKTRUM // TAER, 1100-1900, AIL & RUD reversed

// PPM stream settings
#define CHANNELS 12 // number of channels in ppm stream, 12 ideally
enum chan_order{
    THROTTLE,
    AILERON,
    ELEVATOR,
    RUDDER,
    AUX1,  // (CH5)  led light, or 3 pos. rate on CX-10, H7, or inverted flight on H101
    AUX2,  // (CH6)  flip control
    AUX3,  // (CH7)  still camera (snapshot)
    AUX4,  // (CH8)  video camera
    AUX5,  // (CH9)  headless
    AUX6,  // (CH10) calibrate Y (V2x2), pitch trim (H7), RTH (Bayang, H20), 360deg flip mode (H8-3D, H22)
    AUX7,  // (CH11) calibrate X (V2x2), roll trim (H7)
    AUX8,  // (CH12) Reset / Rebind
};

#define PPM_MIN 1000
#define PPM_SAFE_THROTTLE 1050 
#define PPM_MID 1500
#define PPM_MAX 2000
#define PPM_MIN_COMMAND 1300
#define PPM_MAX_COMMAND 1700
#define GET_FLAG(ch, mask) (ppm[ch] > PPM_MAX_COMMAND ? mask : 0)
#define GET_FLAG_INV(ch, mask) (ppm[ch] < PPM_MIN_COMMAND ? mask : 0)

// supported protocols
enum {
    PROTO_V2X2 = 0,     // WLToys V2x2, JXD JD38x, JD39x, JJRC H6C, Yizhan Tarantula X6 ...
    PROTO_CG023,        // EAchine CG023, CG032, 3D X4
    PROTO_CX10_BLUE,    // Cheerson CX-10 blue board, newer red board, CX-10A, CX-10C, Floureon FX-10, CX-Stars (todo: add DM007 variant)
    PROTO_CX10_GREEN,   // Cheerson CX-10 green board
    PROTO_H7,           // EAchine H7, MoonTop M99xx
    PROTO_BAYANG,       // EAchine H8(C) mini, H10, BayangToys X6, X7, X9, JJRC JJ850, Floureon H101
    PROTO_SYMAX5C1,     // Syma X5C-1 (not older X5C), X11, X11C, X12
    PROTO_YD829,        // YD-829, YD-829C, YD-822 ...
    PROTO_H8_3D,        // EAchine H8 mini 3D, JJRC H20, H22
    PROTO_MJX,          // MJX X600 (can be changed to Weilihua WLH08, X800 or H26D)
    PROTO_SYMAXOLD,     // Syma X5C, X2
    PROTO_HISKY,        // HiSky RXs, HFP80, HCP80/100, FBL70/80/90/100, FF120, HMX120, WLToys v933/944/955 ...
    PROTO_KN,           // KN (WLToys variant) V930/931/939/966/977/988
    PROTO_YD717,        // Cheerson CX-10 red (older version)/CX11/CX205/CX30, JXD389/390/391/393, SH6057/6043/6044/6046/6047, FY326Q7, WLToys v252 Pro/v343, XinXun X28/X30/X33/X39/X40
    PROTO_FQ777124,     // FQ777-124 pocket drone
    PROTO_E010,         // EAchine E010, NiHui NH-010, JJRC H36 mini
    PROTO_BAYANG_SILVERWARE, // Bayang for Silverware with frsky telemetry
    PROTO_END
};

// EEPROM locationss
enum{
    ee_PROTOCOL_ID = 0,
    ee_TXID0,
    ee_TXID1,
    ee_TXID2,
    ee_TXID3
};

struct {
    uint16_t volt1;
    uint16_t rssi;
    uint8_t updated;
    uint32_t lastUpdate;
} telemetry_data;

uint8_t transmitterID[4];
uint8_t current_protocol;
static volatile bool ppm_ok = false;
uint8_t packet[32];
static bool reset=true;
volatile uint16_t Servo_data[12];
static uint16_t ppm[12] = {PPM_MIN,PPM_MIN,PPM_MIN,PPM_MIN,PPM_MID,PPM_MID,
                           PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID,};

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
    frskyInit();
    
    // PPM ISR setup
    attachInterrupt(digitalPinToInterrupt(PPM_pin), ISR_ppm, CHANGE);
    TCCR1A = 0;  //reset timer1
    TCCR1B = 0;
    TCCR1B |= (1 << CS11);  //set timer1 to increment every 1 us @ 8MHz, 0.5 us @16MHz

    set_txid(false);
}

void loop()
{
    uint32_t timeout=0;
    // reset / rebind
    if(reset || ppm[AUX8] > PPM_MAX_COMMAND) {
        reset = false;
        selectProtocol();
        NRF24L01_Reset();
        NRF24L01_Initialize();
        init_protocol();
    }
    telemetry_data.updated = 0;
    // process protocol
    switch(current_protocol) {
        case PROTO_CG023:
        case PROTO_YD829:
            timeout = process_CG023();
            break;
        case PROTO_V2X2:
            timeout = process_V2x2();
            break;
        case PROTO_CX10_GREEN:
        case PROTO_CX10_BLUE:
            timeout = process_CX10();
            break;
        case PROTO_H7:
            timeout = process_H7();
            break;
        case PROTO_BAYANG:
        case PROTO_BAYANG_SILVERWARE:
            timeout = process_Bayang();
            break;
        case PROTO_SYMAX5C1:
        case PROTO_SYMAXOLD:
            timeout = process_SymaX();
            break;
        case PROTO_H8_3D:
            timeout = process_H8_3D();
            break;
        case PROTO_MJX:
        case PROTO_E010:
            timeout = process_MJX();
            break;
        case PROTO_HISKY:
            timeout = process_HiSky();
            break;
        case PROTO_KN:
            timeout = process_KN();
            break;
        case PROTO_YD717:
            timeout = process_YD717();
            break;
        case PROTO_FQ777124:
            timeout = process_FQ777124();
            break;
    }
    // updates ppm values out of ISR
    update_ppm();
    
    while(micros() < timeout) {
        if(telemetry_data.updated) {
            frskyUpdate();
        }            
    }
    telemetry_data.updated = 0;
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
    
    // startup stick commands (protocol selection / renew transmitter ID)
    
    if(ppm[RUDDER] < PPM_MIN_COMMAND && ppm[AILERON] < PPM_MIN_COMMAND) // rudder left + aileron left
        current_protocol = PROTO_BAYANG_SILVERWARE; // Bayang protocol for Silverware with frsky telemetry
        
    else if(ppm[RUDDER] < PPM_MIN_COMMAND)   // Rudder left
        set_txid(true);                      // Renew Transmitter ID
    
    // Rudder right + Aileron right + Elevator down
    else if(ppm[RUDDER] > PPM_MAX_COMMAND && ppm[AILERON] > PPM_MAX_COMMAND && ppm[ELEVATOR] < PPM_MIN_COMMAND)
        current_protocol = PROTO_E010; // EAchine E010, NiHui NH-010, JJRC H36 mini
    
    // Rudder right + Aileron right + Elevator up
    else if(ppm[RUDDER] > PPM_MAX_COMMAND && ppm[AILERON] > PPM_MAX_COMMAND && ppm[ELEVATOR] > PPM_MAX_COMMAND)
        current_protocol = PROTO_FQ777124; // FQ-777-124

    // Rudder right + Aileron left + Elevator up
    else if(ppm[RUDDER] > PPM_MAX_COMMAND && ppm[AILERON] < PPM_MIN_COMMAND && ppm[ELEVATOR] > PPM_MAX_COMMAND)
        current_protocol = PROTO_YD717; // Cheerson CX-10 red (older version)/CX11/CX205/CX30, JXD389/390/391/393, SH6057/6043/6044/6046/6047, FY326Q7, WLToys v252 Pro/v343, XinXun X28/X30/X33/X39/X40
    
    // Rudder right + Aileron left + Elevator down
    else if(ppm[RUDDER] > PPM_MAX_COMMAND && ppm[AILERON] < PPM_MIN_COMMAND && ppm[ELEVATOR] < PPM_MIN_COMMAND)
        current_protocol = PROTO_KN; // KN (WLToys variant) V930/931/939/966/977/988
    
    // Rudder right + Elevator down
    else if(ppm[RUDDER] > PPM_MAX_COMMAND && ppm[ELEVATOR] < PPM_MIN_COMMAND)
        current_protocol = PROTO_HISKY; // HiSky RXs, HFP80, HCP80/100, FBL70/80/90/100, FF120, HMX120, WLToys v933/944/955 ...
    
    // Rudder right + Elevator up
    else if(ppm[RUDDER] > PPM_MAX_COMMAND && ppm[ELEVATOR] > PPM_MAX_COMMAND)
        current_protocol = PROTO_SYMAXOLD; // Syma X5C, X2 ...
    
    // Rudder right + Aileron right
    else if(ppm[RUDDER] > PPM_MAX_COMMAND && ppm[AILERON] > PPM_MAX_COMMAND)
        current_protocol = PROTO_MJX; // MJX X600, other sub protocols can be set in code
    
    // Rudder right + Aileron left
    else if(ppm[RUDDER] > PPM_MAX_COMMAND && ppm[AILERON] < PPM_MIN_COMMAND)
        current_protocol = PROTO_H8_3D; // H8 mini 3D, H20 ...
    
    // Elevator down + Aileron right
    else if(ppm[ELEVATOR] < PPM_MIN_COMMAND && ppm[AILERON] > PPM_MAX_COMMAND)
        current_protocol = PROTO_YD829; // YD-829, YD-829C, YD-822 ...
    
    // Elevator down + Aileron left
    else if(ppm[ELEVATOR] < PPM_MIN_COMMAND && ppm[AILERON] < PPM_MIN_COMMAND)
        current_protocol = PROTO_SYMAX5C1; // Syma X5C-1, X11, X11C, X12
    
    // Elevator up + Aileron right
    else if(ppm[ELEVATOR] > PPM_MAX_COMMAND && ppm[AILERON] > PPM_MAX_COMMAND)
        current_protocol = PROTO_BAYANG;    // EAchine H8(C) mini, BayangToys X6/X7/X9, JJRC JJ850 ...
    
    // Elevator up + Aileron left
    else if(ppm[ELEVATOR] > PPM_MAX_COMMAND && ppm[AILERON] < PPM_MIN_COMMAND) 
        current_protocol = PROTO_H7;        // EAchine H7, MT99xx
    
    // Elevator up  
    else if(ppm[ELEVATOR] > PPM_MAX_COMMAND)
        current_protocol = PROTO_V2X2;       // WLToys V202/252/272, JXD 385/388, JJRC H6C ...
        
    // Elevator down
    else if(ppm[ELEVATOR] < PPM_MIN_COMMAND) 
        current_protocol = PROTO_CG023;      // EAchine CG023/CG031/3D X4, (todo :ATTOP YD-836/YD-836C) ...
    
    // Aileron right
    else if(ppm[AILERON] > PPM_MAX_COMMAND)  
        current_protocol = PROTO_CX10_BLUE;  // Cheerson CX10(blue pcb, newer red pcb)/CX10-A/CX11/CX12 ... 
    
    // Aileron left
    else if(ppm[AILERON] < PPM_MIN_COMMAND)  
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
        case PROTO_YD829:
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
        case PROTO_H7:
            H7_init();
            H7_bind();
            break;
        case PROTO_BAYANG:
        case PROTO_BAYANG_SILVERWARE:
            Bayang_init();
            Bayang_bind();
            break;
        case PROTO_SYMAX5C1:
        case PROTO_SYMAXOLD:
            Symax_init();
            break;
        case PROTO_H8_3D:
            H8_3D_init();
            H8_3D_bind();
            break;
        case PROTO_MJX:
        case PROTO_E010:
            MJX_init();
            MJX_bind();
            break;
        case PROTO_HISKY:
            HiSky_init();
            break;
        case PROTO_KN:
            kn_start_tx(true); // autobind
            break;
        case PROTO_YD717:
            YD717_init();
            break;
        case PROTO_FQ777124:
            FQ777124_init();
            FQ777124_bind();
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
#ifdef SPEKTRUM
    for(uint8_t ch=0; ch<CHANNELS; ch++) {
        if(ch == AILERON || ch == RUDDER) {
            ppm[ch] = 3000-ppm[ch];
        }
        ppm[ch] = constrain(map(ppm[ch],1120,1880,PPM_MIN,PPM_MAX),PPM_MIN,PPM_MAX);
    }
#endif
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
            if(chan==3)
                ppm_ok = true; // 4 first channels Ok
        }
        chan++;
    }
}
