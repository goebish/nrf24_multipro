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

// compatible with EAchine 3D X4, CG023, Attop YD-829, YD-829C

#define CG023_PAYLOAD_SIZE 15
#define CG023_PACKET_PERIOD 8200 // interval of time between start of 2 packets, in us
#define CG023_RF_BIND_CHANNEL 0x2D
#define YD829_PACKET_PERIOD  4100

enum CG023_FLAGS{
    // flags going to packet[13]
    CG023_FLIP    = 0x01, // right shoulder (3D flip switch), resets after aileron or elevator has moved and came back to neutral
    CG023_EASY    = 0x02, // left shoulder (headless mode, for CG023)
    CG023_VIDEO   = 0x04, // video camera (for YD-829C)
    CG023_STILL   = 0x08, // still camera (for YD-829C)
    CG023_LED_OFF = 0x10,
    CG023_RATE_60 = 0x20,
    CG023_RATE_100= 0x40,
};

enum{
    // flags going to packet[13] (YD-829)
    YD_FLAG_FLIP     = 0x01,
    YD_MASK_RATE     = 0x0C,
    YD_FLAG_RATE_MID = 0x04,
    YD_FLAG_RATE_HIGH= 0x08,
    YD_FLAG_HEADLESS = 0x20,
    YD_FLAG_VIDEO    = 0x40, 
    YD_FLAG_STILL    = 0x80,
};

static uint16_t CG023_txid[2];
static uint8_t CG023_rf_data_channel;
static uint16_t cg_packet_period;

uint32_t process_CG023()
{
    uint32_t nextPacket = micros()+cg_packet_period;
    XN297_Configure(_BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP));
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);     // Clear data ready, data sent, and retransmit
    NRF24L01_FlushTx();
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, CG023_rf_data_channel); // Set radio frequency
    CG023_WritePacket(0x55);
    return nextPacket;
}

void CG023_init()
{
    switch(current_protocol) {
        case PROTO_YD829:
            cg_packet_period = YD829_PACKET_PERIOD;
            break;
        case PROTO_CG023:
            cg_packet_period = CG023_PACKET_PERIOD;
            break;
    }
    const uint8_t tx_addr[] = {0x26, 0xA8, 0x67, 0x35, 0xCC};
    CG023_txid[0] = (transmitterID[0] % 40) | 0x80; // [0x80;0xBF]
    CG023_txid[1] = transmitterID[1];
    if( CG023_txid[0] == 0xAA) // avoid using same freq for bind and data channel
        CG023_txid[0] ++;
    CG023_rf_data_channel = CG023_txid[0] - 0x7D; // [0x03;0x42]
    NRF24L01_Initialize();
    NRF24L01_Reset();
    delay(10);
    XN297_SetTXAddr(tx_addr,5);
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

void CG023_bind()
{
    byte counter=255;
    while(counter--) {
        XN297_Configure(_BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP));
        NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70); // Clear data ready, data sent, and retransmit
        NRF24L01_FlushTx();
        NRF24L01_WriteReg(NRF24L01_05_RF_CH, CG023_RF_BIND_CHANNEL); // Set radio frequency
        CG023_WritePacket(0xAA); // send bind packet
        digitalWrite(ledPin, bitRead(counter,3)); //check for 0bxxxx1xxx to flash LED
        delayMicroseconds(cg_packet_period);
    }
    digitalWrite(ledPin, HIGH); // LED on at end of bind
}

void CG023_WritePacket(uint8_t init)
{
    packet[0] = init;
    packet[1] = CG023_txid[0];
    packet[2] = CG023_txid[1];
    packet[3] = 0;
    packet[4] = 0;
    packet[5] = map(ppm[THROTTLE], PPM_MIN, PPM_MAX, 0, 255) ; // throttle stick
    if(ppm[RUDDER] < PPM_MID - 9 )
        packet[6] = map(ppm[RUDDER], PPM_MID , PPM_MIN, 0x80 , 0xBC);
    else if(ppm[RUDDER] > PPM_MID + 9)
        packet[6] = map(ppm[RUDDER], PPM_MID , PPM_MAX, 0x00 , 0x3C);
    else
    packet[6] = 0x00;
    packet[7] = map(ppm[ELEVATOR], PPM_MIN, PPM_MAX, 0xBB, 0x43); // elevator stick 0xBB - 0x7F - 0x43
    packet[8] = map(ppm[AILERON], PPM_MIN, PPM_MAX, 0xBB, 0x43); // aileron stick 0xBB - 0x7F - 0x43
    packet[9] = 0x20; // throttle trim, neutral = 0x20
    packet[10] = 0x20; // rudder trim, neutral = 0x20
    packet[11] = 0x40; // elevator trim, neutral = 0x40
    packet[12] = 0x40; // aileron trim, neutral = 0x40
    
    switch(current_protocol) {
        case PROTO_CG023:
            packet[13] = CG023_RATE_100; // 100% rate
            if(ppm[AUX1]>PPM_MAX_COMMAND)
                packet[13] |= CG023_LED_OFF;
            if(ppm[AUX2]>PPM_MAX_COMMAND)
                packet[13] |= CG023_FLIP;
            if(ppm[AUX3]>PPM_MAX_COMMAND)
                packet[13] |= CG023_STILL;
            if(ppm[AUX4]>PPM_MAX_COMMAND)
                packet[13] |= CG023_VIDEO;
            if(ppm[AUX5]>PPM_MAX_COMMAND)
                packet[13] |= CG023_EASY;
            break;
        case PROTO_YD829:
            packet[13] = YD_FLAG_RATE_HIGH;
            // reverse aileron direction
            packet[8] = 0xFE - packet[8];
            if(ppm[AUX2] > PPM_MAX_COMMAND)
                packet[13] |= YD_FLAG_FLIP;
            if(ppm[AUX3] > PPM_MAX_COMMAND)
                packet[13] |= YD_FLAG_STILL;
            if(ppm[AUX4] > PPM_MAX_COMMAND)
                packet[13] |= YD_FLAG_VIDEO;
            if(ppm[AUX5] > PPM_MAX_COMMAND)
                packet[13] |= YD_FLAG_HEADLESS;
            break;
    }
    packet[14] = 0x00;
    XN297_WritePayload(packet, CG023_PAYLOAD_SIZE);
}
