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
#include "SymaX.h"

static uint32_t SYMAX_packet_counter;
static uint8_t SYMAX_rx_tx_addr[5];
static uint8_t SYMAX_current_chan;
static uint8_t SYMAX_chans[SYMAX_RF_CHANNELS];

static void SYMAX_initialize_rx_tx_addr();
static void SYMAX_send_packet(uint8_t bind);
static void SYMAX_build_packet(u8 bind);
// channels determined by last byte of tx address
static void SYMAX_set_channels(u8 address);
static uint8_t SymaX_checksum(uint8_t *data);

uint32_t protSYMAX::loop() {
    uint32_t nextPacket = micros() + SYMAX_PACKET_PERIOD;
    SYMAX_send_packet(false);
    return nextPacket;
}

void protSYMAX::init() {
    const uint8_t bind_rx_tx_addr[] = { 0xab, 0xac, 0xad, 0xae, 0xaf };
    uint8_t first_packet[] = { 0xf9, 0x96, 0x82, 0x1b, 0x20, 0x08, 0x08, 0xf2, 0x7d, 0xef, 0xff, 0x00, 0x00, 0x00, 0x00 };
    uint8_t chans_bind[] = { 0x4b, 0x30, 0x40, 0x20 };
    SYMAX_packet_counter = 0;

    NRF24L01_Reset();
    NRF24L01_Initialize();
    NRF24L01_SetTxRxMode(TX_EN);
    NRF24L01_ReadReg(NRF24L01_07_STATUS);
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, _BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO));
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknowledgment
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x3F);  // Enable all data pipes (even though not used?)
    NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x03);   // 5-byte RX/TX address
    NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0xff); // 4mS retransmit t/o, 15 tries (retries w/o AA?)
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, 0x08);
    NRF24L01_SetBitrate(NRF24L01_BR_250K);
    NRF24L01_SetPower(RF_POWER);
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);     // Clear data ready, data sent, and retransmit
    NRF24L01_WriteReg(NRF24L01_08_OBSERVE_TX, 0x00);
    NRF24L01_WriteReg(NRF24L01_09_CD, 0x00);
    NRF24L01_WriteReg(NRF24L01_0C_RX_ADDR_P2, 0xC3); // LSB byte of pipe 2 receive address
    NRF24L01_WriteReg(NRF24L01_0D_RX_ADDR_P3, 0xC4);
    NRF24L01_WriteReg(NRF24L01_0E_RX_ADDR_P4, 0xC5);
    NRF24L01_WriteReg(NRF24L01_0F_RX_ADDR_P5, 0xC6);
    NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, 10);   // bytes of data payload for pipe 1
    NRF24L01_WriteReg(NRF24L01_12_RX_PW_P1, 10);
    NRF24L01_WriteReg(NRF24L01_13_RX_PW_P2, 10);
    NRF24L01_WriteReg(NRF24L01_14_RX_PW_P3, 10);
    NRF24L01_WriteReg(NRF24L01_15_RX_PW_P4, 10);
    NRF24L01_WriteReg(NRF24L01_16_RX_PW_P5, 10);
    NRF24L01_WriteReg(NRF24L01_17_FIFO_STATUS, 0x00); // Just in case, no real bits to write here
    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, bind_rx_tx_addr, 5);
    NRF24L01_ReadReg(NRF24L01_07_STATUS);
    NRF24L01_FlushTx();
    NRF24L01_ReadReg(NRF24L01_07_STATUS);
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x0e);
    NRF24L01_ReadReg(NRF24L01_00_CONFIG);
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, 0x0c);
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, 0x0e);  // power on    
    NRF24L01_FlushTx();
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, 0x08);
    NRF24L01_WritePayload(first_packet, 15);
    SYMAX_initialize_rx_tx_addr();   // make info available for bind packets
    memcpy(SYMAX_chans, chans_bind, SYMAX_RF_CHANNELS);
    SYMAX_current_chan = 0;
    SYMAX_packet_counter = 0;
    delay(12);
}

void protSYMAX::bind() {
    uint16_t bind_counter = SYMAX_BIND_COUNT;
    while(bind_counter) {
        SYMAX_send_packet(true);
        delayMicroseconds(SYMAX_PACKET_PERIOD);
        digitalWrite(ledPin, bind_counter-- & 0x10);
    }
    SYMAX_set_channels(SYMAX_rx_tx_addr[0]);
    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, SYMAX_rx_tx_addr, 5);
    SYMAX_current_chan = 0;
    SYMAX_packet_counter = 0;
    digitalWrite(ledPin, HIGH);
}

static uint8_t SymaX_checksum(uint8_t *data) {
    uint8_t sum = data[0];
    for(uint8_t i = 1; i < SYMAX_PACKET_SIZE - 1; i++)
        sum ^= data[i];
    return sum + 0x55;
}

// channels determined by last byte of tx address
static void SYMAX_set_channels(u8 address) {
    static const uint8_t start_chans_1[] = { 0x0a, 0x1a, 0x2a, 0x3a };
    static const uint8_t start_chans_2[] = { 0x2a, 0x0a, 0x42, 0x22 };
    static const uint8_t start_chans_3[] = { 0x1a, 0x3a, 0x12, 0x32 };
    uint8_t laddress = address & 0x1f;
    uint8_t i;
    uint32_t *pchans = (u32 *) SYMAX_chans;
    if(laddress < 0x10) {
        if(laddress == 6)
            laddress = 7;
        for(i = 0; i < SYMAX_RF_CHANNELS; i++) {
            SYMAX_chans[i] = start_chans_1[i] + laddress;
        }
    } else if(laddress < 0x18) {
        for(i = 0; i < SYMAX_RF_CHANNELS; i++) {
            SYMAX_chans[i] = start_chans_2[i] + (laddress & 0x07);
        }
        if(laddress == 0x16) {
            SYMAX_chans[0] += 1;
            SYMAX_chans[1] += 1;
        }
    } else if(laddress < 0x1e) {
        for(i = 0; i < SYMAX_RF_CHANNELS; i++) {
            SYMAX_chans[i] = start_chans_3[i] + (laddress & 0x07);
        }
    } else if(laddress == 0x1e) {
        *pchans = 0x38184121;
    } else {
        *pchans = 0x39194121;
    }
}

static void SYMAX_build_packet(u8 bind) {
    if(bind) {
        Ppacket[0] = SYMAX_rx_tx_addr[4];
        Ppacket[1] = SYMAX_rx_tx_addr[3];
        Ppacket[2] = SYMAX_rx_tx_addr[2];
        Ppacket[3] = SYMAX_rx_tx_addr[1];
        Ppacket[4] = SYMAX_rx_tx_addr[0];
        Ppacket[5] = 0xaa;
        Ppacket[6] = 0xaa;
        Ppacket[7] = 0xaa;
        Ppacket[8] = 0x00;
    } else {
        Ppacket[0] = multipro.getChannel(CH_THROTTLE, 0x00, 0xFF); // throttle;
        if(multipro.getChannel(CH_ELEVATOR) < PPM_MID) {
            Ppacket[1] = map(multipro.getChannel(CH_ELEVATOR), PPM_MID, PPM_MIN, 0x80, 0xFF);
        } else {
            Ppacket[1] = map(multipro.getChannel(CH_ELEVATOR), PPM_MID, PPM_MAX, 0x00, 0x7F);
        }

        if(multipro.getChannel(CH_RUDDER) < PPM_MID) {
            Ppacket[2] = map(multipro.getChannel(CH_RUDDER), PPM_MID, PPM_MIN, 0x00, 0x7F);
        } else {
            Ppacket[2] = map(multipro.getChannel(CH_RUDDER), PPM_MID, PPM_MAX, 0x80, 0xFF);
        }

        if(multipro.getChannel(CH_AILERON) < PPM_MID) {
            Ppacket[3] = map(multipro.getChannel(CH_AILERON), PPM_MID, PPM_MIN, 0x00, 0x7F);
        } else {
            Ppacket[3] = map(multipro.getChannel(CH_AILERON), PPM_MID, PPM_MAX, 0x80, 0xFF);
        }

        Ppacket[4] = 0;

        if(multipro.getChannelIsCMD(CH_CAM)) {
            Ppacket[4] |= SYMAX_FLAG_VIDEO;
        }

        if(multipro.getChannelIsCMD(CH_PIC)) {
            Ppacket[4] |= SYMAX_FLAG_PICTURE;
        }

        /// todo CH_3WAY for rate ??
        // use trims to extend controls
        Ppacket[5] = (Ppacket[1] >> 2) | 0xc0;  // always high rates (bit 7 is rate control)
        Ppacket[6] = (Ppacket[2] >> 2);
        Ppacket[7] = (Ppacket[3] >> 2);

        if(multipro.getChannelIsCMD(CH_FLIP)) {
            Ppacket[6] |= SYMAX_FLAG_FLIP;
        }

        if(multipro.getChannelIsCMD(CH_HEADLESS)) {
            Ppacket[7] |= SYMAX_FLAG_HEADLESS;
        }

        Ppacket[8] = 0x00;
    }
    Ppacket[9] = SymaX_checksum(Ppacket);
}

static void SYMAX_send_packet(uint8_t bind) {
    SYMAX_build_packet(bind);
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, 0x2e);
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, SYMAX_chans[SYMAX_current_chan]);
    NRF24L01_FlushTx();
    NRF24L01_WritePayload(Ppacket, SYMAX_PACKET_SIZE);
    if(SYMAX_packet_counter++ % 2) {   // use each channel twice
        SYMAX_current_chan = (SYMAX_current_chan + 1) % SYMAX_RF_CHANNELS;
    }
}

static void SYMAX_initialize_rx_tx_addr() {
    for(uint8_t i = 0; i < 4; i++)
        SYMAX_rx_tx_addr[i] = transmitterID[i];
    SYMAX_rx_tx_addr[4] = 0xa2;
}
