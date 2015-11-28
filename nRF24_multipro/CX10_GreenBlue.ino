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

#define CX10_GREEN_PACKET_LENGTH 15
#define CX10_BLUE_PACKET_LENGTH 19
#define CX10_BLUE_PACKET_PERIOD 6000
#define CX10_GREEN_PACKET_PERIOD 3000
#define CX10_GREEN_BIND_COUNT 1000
#define CX10_RF_BIND_CHANNEL 0x02
#define CX10_NUM_RF_CHANNELS    4

static uint8_t CX10_txid[4]; // transmitter ID
static uint8_t CX10_freq[4]; // frequency hopping table
static uint8_t CX10_current_chan = 0;
static uint8_t CX10_packet_length;
static uint32_t CX10_packet_period;
static const uint8_t CX10_tx_rx_id[] = {0xCC,0xCC,0xCC,0xCC,0xCC};

uint32_t process_CX10()
{
    uint32_t nextPacket = micros() + CX10_packet_period;
    XN297_Configure(_BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP));
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);     // Clear data ready, data sent, and retransmit
    NRF24L01_FlushTx();
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, CX10_freq[CX10_current_chan++]);
    CX10_current_chan %= CX10_NUM_RF_CHANNELS;
    CX10_Write_Packet(0x55);
    return nextPacket;
}

void CX10_init()
{
    uint8_t i;
    switch(current_protocol) {
        case PROTO_CX10_BLUE:
            for(i=0; i<4; i++) {
                packet[5+i] = 0xFF; // clear aircraft ID
            }
            CX10_packet_length = CX10_BLUE_PACKET_LENGTH;
            CX10_packet_period = CX10_BLUE_PACKET_PERIOD;
            break;
        case PROTO_CX10_GREEN:
            CX10_packet_length = CX10_GREEN_PACKET_LENGTH;
            CX10_packet_period = CX10_GREEN_PACKET_PERIOD;
            break;
    }

    for(i=0; i<4; i++) {
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
    XN297_SetTXAddr(CX10_tx_rx_id,5);
    XN297_SetRXAddr(CX10_tx_rx_id,5);
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

void CX10_bind()
{
    uint16_t counter=CX10_GREEN_BIND_COUNT;
    bool bound=false;
    uint32_t timeout;
    while(!bound) {
        NRF24L01_SetTxRxMode(TX_EN);
        XN297_Configure(_BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP));
        NRF24L01_WriteReg(NRF24L01_05_RF_CH, CX10_RF_BIND_CHANNEL);
        NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
        NRF24L01_FlushTx();
        CX10_Write_Packet(0xAA); // send bind packet
        switch(current_protocol) {
            case PROTO_CX10_GREEN:
                delayMicroseconds(CX10_packet_period);
                if(counter==0)
                    bound = true;
                break;
            case PROTO_CX10_BLUE:
                delay(1);
                NRF24L01_SetTxRxMode(RX_EN);
                NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
                NRF24L01_FlushRx();
                XN297_Configure(_BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP) | _BV(NRF24L01_00_PRIM_RX));
                timeout = millis()+5;
                while(millis()<timeout) {
                    if(NRF24L01_ReadReg(NRF24L01_07_STATUS) & _BV(NRF24L01_07_RX_DR)) { // data received from aircraft
                        XN297_ReadPayload(packet, CX10_packet_length);
                        if( packet[9] == 0x01)
                        bound = true;
                        break;
                    }
                }
                break;
        }
        digitalWrite(ledPin, counter-- & 0x10);
        if(ppm[AUX8] > PPM_MAX_COMMAND) {
            reset = true;
            return;
        }
    }
    digitalWrite(ledPin, HIGH);
}

void CX10_Write_Packet(uint8_t init)
{
    uint8_t offset = 0;
    if(current_protocol == PROTO_CX10_BLUE)
        offset = 4;
    packet[0] = init;
    packet[1] = CX10_txid[0];
    packet[2] = CX10_txid[1];
    packet[3] = CX10_txid[2];
    packet[4] = CX10_txid[3];
    // packet[5] to [8] (aircraft id) is filled during bind for blue board CX10
    packet[5+offset] = lowByte(3000-ppm[AILERON]);
    packet[6+offset]= highByte(3000-ppm[AILERON]);
    packet[7+offset]= lowByte(3000-ppm[ELEVATOR]);
    packet[8+offset]= highByte(3000-ppm[ELEVATOR]);
    packet[9+offset]= lowByte(ppm[THROTTLE]);
    packet[10+offset]= highByte(ppm[THROTTLE]);
    packet[11+offset]= lowByte(ppm[RUDDER]);
    packet[12+offset]= highByte(ppm[RUDDER]);
    if(ppm[AUX2] > PPM_MAX_COMMAND)
        packet[12+offset] |= 0x10; // flip flag
    // rate / mode
    if(ppm[AUX1] > PPM_MAX_COMMAND) // mode 3 / headless on CX-10A
        packet[13+offset] = 0x02;
    else if(ppm[AUX1] < PPM_MIN_COMMAND) // mode 1
        packet[13+offset] = 0x00;
    else // mode 2
        packet[13+offset] = 0x01;
    packet[14+offset] = 0x00;
    if(current_protocol == PROTO_CX10_BLUE) {
        // snapshot (CX10-C)
        if(ppm[AUX3] < PPM_MAX_COMMAND)
            packet[13+offset] |= 0x10;
        // video recording (CX10-C)
        if(ppm[AUX4] > PPM_MAX_COMMAND)
            packet[13+offset] |= 0x08;
    }    

    XN297_WritePayload(packet, CX10_packet_length);
}
