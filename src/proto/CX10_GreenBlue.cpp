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
#include "CX10_GreenBlue.h"

static const uint8_t CX10_tx_rx_id[] = { 0xCC, 0xCC, 0xCC, 0xCC, 0xCC };

uint32_t protCX10_GREENBLUE::loop() {
    uint32_t nextPacket = micros() + CX10_packet_period;
    XN297_Configure(_BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP));
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);     // Clear data ready, data sent, and retransmit
    NRF24L01_FlushTx();
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, CX10_freq[CX10_current_chan++]);
    CX10_current_chan %= CX10_NUM_RF_CHANNELS;
    CX10_Write_Packet(0x55);
    return nextPacket;
}

void protCX10_GREENBLUE::init() {
    uint8_t i;
    switch(version) {
        case PROTO_CX10_BLUE:
            for(i = 0; i < 4; i++) {
                Ppacket[5 + i] = 0xFF; // clear aircraft ID
            }
            CX10_packet_length = CX10_BLUE_PACKET_LENGTH;
            CX10_packet_period = CX10_BLUE_PACKET_PERIOD;
            break;
        case PROTO_CX10_GREEN:
            CX10_packet_length = CX10_GREEN_PACKET_LENGTH;
            CX10_packet_period = CX10_GREEN_PACKET_PERIOD;
            break;
    }

    for(i = 0; i < 4; i++) {
        CX10_txid[i] = transmitterID[i];
    }
    CX10_txid[1] &= 0x2F;
    CX10_freq[0] = (CX10_txid[0] & 0x0F) + 0x03;
    CX10_freq[1] = (CX10_txid[0] >> 4) + 0x16;
    CX10_freq[2] = (CX10_txid[1] & 0x0F) + 0x2D;
    CX10_freq[3] = (CX10_txid[1] >> 4) + 0x40;

    CX10_current_chan = 0;
    NRF24L01_Reset();
    NRF24L01_Initialize();
    NRF24L01_SetTxRxMode(TX_EN);
    delay(10);
    XN297_SetTXAddr(CX10_tx_rx_id, 5);
    XN297_SetRXAddr(CX10_tx_rx_id, 5);
    NRF24L01_FlushTx();
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);     // Clear data ready, data sent, and retransmit
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknowledgment on all data pipes
    NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, CX10_packet_length); // rx pipe 0
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);  // Enable data pipe 0 only
    NRF24L01_SetBitrate(NRF24L01_BR_1M);             // 1Mbps
    NRF24L01_SetPower(RF_POWER);
    NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x00);       // Disable dynamic payload length on all pipes
    NRF24L01_WriteReg(NRF24L01_1D_FEATURE, 0x00);     // Set feature bits
    delay(150);
}

void protCX10_GREENBLUE::bind() {
    uint16_t counter = CX10_GREEN_BIND_COUNT;
    bool bound = false;
    uint32_t timeout;
    while(!bound) {
        NRF24L01_SetTxRxMode(TX_EN);
        XN297_Configure(_BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP));
        NRF24L01_WriteReg(NRF24L01_05_RF_CH, CX10_RF_BIND_CHANNEL);
        NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
        NRF24L01_FlushTx();
        CX10_Write_Packet(0xAA); // send bind Ppacket
        switch(version) {
            case PROTO_CX10_GREEN:
                delayMicroseconds(CX10_packet_period);
                if(counter == 0)
                    bound = true;
                break;
            case PROTO_CX10_BLUE:
                delay(1);
                NRF24L01_SetTxRxMode(RX_EN);
                NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
                NRF24L01_FlushRx();
                XN297_Configure(_BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP) | _BV(NRF24L01_00_PRIM_RX));
                timeout = millis() + 5;
                while(millis() < timeout) {
                    if(NRF24L01_ReadReg(NRF24L01_07_STATUS) & _BV(NRF24L01_07_RX_DR)) { // data received from aircraft
                        XN297_ReadPayload(Ppacket, CX10_packet_length);
                        if(Ppacket[9] == 0x01)
                            bound = true;
                        break;
                    }
                }
                break;
        }
        digitalWrite(ledPin, counter-- & 0x10);
    }
    digitalWrite(ledPin, HIGH);
}

void protCX10_GREENBLUE::CX10_Write_Packet(uint8_t init) {
    uint8_t offset = 0;
    if(version == PROTO_CX10_BLUE)
        offset = 4;
    Ppacket[0] = init;
    Ppacket[1] = CX10_txid[0];
    Ppacket[2] = CX10_txid[1];
    Ppacket[3] = CX10_txid[2];
    Ppacket[4] = CX10_txid[3];
    // Ppacket[5] to [8] (aircraft id) is filled during bind for blue board CX10
    Ppacket[5 + offset] = lowByte(3000 - multipro.getChannel(CH_AILERON));
    Ppacket[6 + offset] = highByte(3000 - multipro.getChannel(CH_AILERON));
    Ppacket[7 + offset] = lowByte(3000 - multipro.getChannel(CH_ELEVATOR));
    Ppacket[8 + offset] = highByte(3000 - multipro.getChannel(CH_ELEVATOR));
    Ppacket[9 + offset] = lowByte(multipro.getChannel(CH_THROTTLE));
    Ppacket[10 + offset] = highByte(multipro.getChannel(CH_THROTTLE));
    Ppacket[11 + offset] = lowByte(multipro.getChannel(CH_RUDDER));
    Ppacket[12 + offset] = highByte(multipro.getChannel(CH_RUDDER));
    if(multipro.getChannel(CH_FLIP) > PPM_MID)
        Ppacket[12 + offset] |= 0x10; // flip flag
    // rate / mode (use headless channel)
    if(multipro.getChannel(CH_AUX1) > PPM_MAX_COMMAND) // mode 3 / headless
        Ppacket[13 + offset] = 0x02;
    else if(multipro.getChannel(CH_AUX1) < PPM_MIN_COMMAND) // mode 1
        Ppacket[13 + offset] = 0x00;
    else
        // mode 2
        Ppacket[13 + offset] = 0x01;
    Ppacket[14 + offset] = 0x00;
    XN297_WritePayload(Ppacket, CX10_packet_length);
}
