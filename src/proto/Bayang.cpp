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
#include "Bayang.h"

static uint8_t Bayang_rf_chan;
static uint8_t Bayang_rf_channels[BAYANG_RF_NUM_CHANNELS] = {0,};
static uint8_t Bayang_rx_tx_addr[BAYANG_ADDRESS_LENGTH];

enum{
    // flags going to Ppacket[2]
    BAYANG_FLAG_RTH      = 0x01,
    BAYANG_FLAG_HEADLESS = 0x02,
    BAYANG_FLAG_FLIP     = 0x08,
};


static void send_packet(u8 bind);

uint32_t protBAYANG::loop()
{
    uint32_t timeout = micros() + BAYANG_PACKET_PERIOD;
    send_packet(0);
    return timeout;
}

void protBAYANG::init()
{
    uint8_t i;
    const u8 bind_address[] = {0,0,0,0,0};
    for(i=0; i<BAYANG_ADDRESS_LENGTH; i++) {
        Bayang_rx_tx_addr[i] = random() & 0xff;
    }
    Bayang_rf_channels[0] = 0x00;
    for(i=1; i<BAYANG_RF_NUM_CHANNELS; i++) {
        Bayang_rf_channels[i] = random() % 0x42;
    }
    NRF24L01_Initialize();
    NRF24L01_SetTxRxMode(TX_EN);
    XN297_SetTXAddr(bind_address, BAYANG_ADDRESS_LENGTH);
    NRF24L01_FlushTx();
    NRF24L01_FlushRx();
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknowldgement on all data pipes
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);
    NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x03);
    NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0x00); // no retransmits
    NRF24L01_SetBitrate(NRF24L01_BR_1M);             // 1Mbps
    NRF24L01_SetPower(3);
    NRF24L01_Activate(0x73);                         // Activate feature register
    NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x00);      // Disable dynamic payload length on all pipes
    NRF24L01_WriteReg(NRF24L01_1D_FEATURE, 0x01);
    NRF24L01_Activate(0x73);
    delay(150);
}

void protBAYANG::bind()
{
    uint16_t counter = BAYANG_BIND_COUNT;
    while(counter) {
        send_packet(1);
        delayMicroseconds(BAYANG_PACKET_PERIOD);
        digitalWrite(ledPin, counter-- & 0x10);
    }
    XN297_SetTXAddr(Bayang_rx_tx_addr, BAYANG_ADDRESS_LENGTH);
    digitalWrite(ledPin, HIGH);
}

#define DYNTRIM(chval) ((u8)((chval >> 2) & 0xfc))
#define GET_FLAG(ch, mask) (multipro.getChannelIsCMD(ch) ? mask : 0)

static void send_packet(u8 bind)
{
    union {
        u16 value;
        struct {
            u8 lsb;
            u8 msb;
        } bytes;
    } chanval;

    if (bind) {
        Ppacket[0] = 0xa4;
        memcpy(&Ppacket[1], Bayang_rx_tx_addr, 5);
        memcpy(&Ppacket[6], Bayang_rf_channels, 4);
        Ppacket[10] = transmitterID[0];
        Ppacket[11] = transmitterID[1];
    } else {
        Ppacket[0] = 0xa5;
        Ppacket[1] = 0xfa;   // normal mode is 0xf7, expert 0xfa
        Ppacket[2] = GET_FLAG(CH_FLIP, BAYANG_FLAG_FLIP)
                  | GET_FLAG(CH_HEADLESS, BAYANG_FLAG_HEADLESS)
                  | GET_FLAG(CH_AUX6, BAYANG_FLAG_RTH);
        Ppacket[3] = 0x00;
        chanval.value = multipro.getChannel(CH_AILERON, 0, 0x3ff);   // aileron
        Ppacket[4] = chanval.bytes.msb + DYNTRIM(chanval.value);
        Ppacket[5] = chanval.bytes.lsb;
        chanval.value = multipro.getChannel(CH_ELEVATOR, 0, 0x3ff);   // elevator
        Ppacket[6] = chanval.bytes.msb + DYNTRIM(chanval.value);
        Ppacket[7] = chanval.bytes.lsb;
        chanval.value = multipro.getChannel(CH_THROTTLE, 0, 0x3ff);   // throttle
        Ppacket[8] = chanval.bytes.msb + 0x7c;
        Ppacket[9] = chanval.bytes.lsb;
        chanval.value = multipro.getChannel(CH_RUDDER, 0, 0x3ff);   // rudder
        Ppacket[10] = chanval.bytes.msb + DYNTRIM(chanval.value);
        Ppacket[11] = chanval.bytes.lsb;
    }
    Ppacket[12] = transmitterID[2];
    Ppacket[13] = 0x0a;
    Ppacket[14] = 0;
    for(uint8_t i=0; i<BAYANG_PACKET_SIZE-1; i++) {
        Ppacket[14] += Ppacket[i];
    }
    
    XN297_Configure(_BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP));
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, bind ? BAYANG_RF_BIND_CHANNEL : Bayang_rf_channels[Bayang_rf_chan++]);
    Bayang_rf_chan %= sizeof(Bayang_rf_channels);
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
    NRF24L01_FlushTx();
    XN297_WritePayload(Ppacket, BAYANG_PACKET_SIZE);
}
