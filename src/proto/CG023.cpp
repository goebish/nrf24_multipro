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

#include <Arduino.h>
#include "nrf24_multipro.h"
#include "protocol.h"
#include "CG023.h"

enum CG023_FLAGS {
    // flags going to Ppacket[13]
    CG023_FLIP = 0x01, // right shoulder (3D flip switch), resets after aileron or elevator has moved and came back to neutral
    CG023_EASY = 0x02, // left shoulder (headless mode, for CG023)
    CG023_VIDEO = 0x04, // video camera (for YD-829C)
    CG023_STILL = 0x08, // still camera (for YD-829C)
    CG023_LED_OFF = 0x10,
    CG023_RATE_60 = 0x20,
    CG023_RATE_100 = 0x40,
};

enum {
    // flags going to packet[13] (YD-829)
    YD_FLAG_FLIP = 0x01,
    YD_MASK_RATE = 0x0C,
    YD_FLAG_RATE_MID = 0x04,
    YD_FLAG_RATE_HIGH = 0x08,
    YD_FLAG_HEADLESS = 0x20,
    YD_FLAG_VIDEO = 0x40,
    YD_FLAG_STILL = 0x80,
};

static uint16_t CG023_txid[2];
static uint8_t CG023_rf_data_channel;

uint32_t protCG023::loop() {
    uint32_t nextPacket = micros() + cg_packet_period;
    XN297_Configure(_BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP));
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);     // Clear data ready, data sent, and retransmit
    NRF24L01_FlushTx();
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, CG023_rf_data_channel); // Set radio frequency
    CG023_WritePacket(0x55);
    return nextPacket;
}

void protCG023::init() {
    switch(version) {
        case PROTO_YD829:
            cg_packet_period = YD829_PACKET_PERIOD;
            break;
        case PROTO_CG023:
            cg_packet_period = CG023_PACKET_PERIOD;
            break;
    }
    const uint8_t tx_addr[] = { 0x26, 0xA8, 0x67, 0x35, 0xCC };
    CG023_txid[0] = (transmitterID[0] % 40) | 0x80; // [0x80;0xBF]
    CG023_txid[1] = transmitterID[1];
    if(CG023_txid[0] == 0xAA) // avoid using same freq for bind and data channel
        CG023_txid[0]++;
    CG023_rf_data_channel = CG023_txid[0] - 0x7D; // [0x03;0x42]
    NRF24L01_Initialize();
    NRF24L01_Reset();
    delay(10);
    XN297_SetTXAddr(tx_addr, 5);
    NRF24L01_FlushTx();
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);     // Clear data ready, data sent, and retransmit
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknowledgment on all data pipes
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);  // Enable data pipe 0 only
    NRF24L01_SetBitrate(NRF24L01_BR_1M);             // 1Mbps
    NRF24L01_SetPower(RF_POWER);
    NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x00);      // Disable dynamic payload length on all pipes
    NRF24L01_WriteReg(NRF24L01_1D_FEATURE, 0x00);    // Set feature bits

    delay(400);
}

void protCG023::bind() {
    byte counter = 255;
    while(counter--) {
        XN297_Configure(_BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP));
        NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70); // Clear data ready, data sent, and retransmit
        NRF24L01_FlushTx();
        NRF24L01_WriteReg(NRF24L01_05_RF_CH, CG023_RF_BIND_CHANNEL); // Set radio frequency
        CG023_WritePacket(0xAA); // send bind Ppacket
        digitalWrite(pinLED, bitRead(counter, 3)); //check for 0bxxxx1xxx to flash LED
        delayMicroseconds(cg_packet_period);
    }
    digitalWrite(pinLED, HIGH); // LED on at end of bind
}

void protCG023::CG023_WritePacket(uint8_t init) {
    Ppacket[0] = init;
    Ppacket[1] = CG023_txid[0];
    Ppacket[2] = CG023_txid[1];
    Ppacket[3] = 0;
    Ppacket[4] = 0;
    Ppacket[5] = multipro.getChannel(CH_THROTTLE, 0, 255); // throttle stick
    if(multipro.getChannel(CH_RUDDER) < PPM_MID)
        Ppacket[6] = multipro.getChannel(CH_RUDDER, 0x80, 0xBC);
    else if(multipro.getChannel(CH_RUDDER) > PPM_MID)
        Ppacket[6] = multipro.getChannel(CH_RUDDER, 0x00, 0x3C);
    else
        Ppacket[6] = 0x00;
    Ppacket[7] = multipro.getChannel(CH_ELEVATOR, 0xBB, 0x43); // elevator stick 0xBB - 0x7F - 0x43
    Ppacket[8] = multipro.getChannel(CH_AILERON, 0xBB, 0x43); // aileron stick 0xBB - 0x7F - 0x43
    Ppacket[9] = 0x20; // throttle trim, neutral = 0x20
    Ppacket[10] = 0x20; // rudder trim, neutral = 0x20
    Ppacket[11] = 0x40; // elevator trim, neutral = 0x40
    Ppacket[12] = 0x40; // aileron trim, neutral = 0x40
    switch(version) {
        case PROTO_CG023:
            Ppacket[13] = CG023_RATE_100; // 100% rate
            if(multipro.getChannelIsCMD(CH_3WAY))
                Ppacket[13] |= CG023_LED_OFF;
            if(multipro.getChannelIsCMD(CH_FLIP))
                Ppacket[13] |= CG023_FLIP;
            if(multipro.getChannelIsCMD(CH_PIC))
                Ppacket[13] |= CG023_STILL;
            if(multipro.getChannelIsCMD(CH_CAM))
                Ppacket[13] |= CG023_VIDEO;
            if(multipro.getChannelIsCMD(CH_HEADLESS))
                Ppacket[13] |= CG023_EASY;
            break;
        case PROTO_YD829:
            Ppacket[13] = YD_FLAG_RATE_HIGH;
            // reverse aileron direction
            Ppacket[8] = 0xFE - Ppacket[8];
            if(multipro.getChannelIsCMD(CH_FLIP))
                Ppacket[13] |= YD_FLAG_FLIP;
            if(multipro.getChannelIsCMD(CH_PIC))
                Ppacket[13] |= YD_FLAG_STILL;
            if(multipro.getChannelIsCMD(CH_CAM))
                Ppacket[13] |= YD_FLAG_VIDEO;
            if(multipro.getChannelIsCMD(CH_HEADLESS))
                Ppacket[13] |= YD_FLAG_HEADLESS;
            break;
    }
    Ppacket[14] = 0x00;
    XN297_WritePayload(Ppacket, CG023_PAYLOAD_SIZE);
}
