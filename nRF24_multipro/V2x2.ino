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

enum {
    // flags going to byte 14
    V2x2_FLAG_CAMERA = 0x01, // also automatic Missile Launcher and Hoist in one direction
    V2x2_FLAG_VIDEO  = 0x02, // also Sprayer, Bubbler, Missile Launcher(1), and Hoist in the other dir.
    V2x2_FLAG_FLIP   = 0x04,
    V2x2_FLAG_UNK9   = 0x08,
    V2x2_FLAG_LED    = 0x10,
    V2x2_FLAG_UNK10  = 0x20,
    V2x2_FLAG_BIND   = 0xC0,
    // flags going to byte 10
    V2x2_FLAG_HEADLESS  = 0x0200,
    V2x2_FLAG_MAG_CAL_X = 0x0800,
    V2x2_FLAG_MAG_CAL_Y = 0x2000
};

#define V2x2_PAYLOADSIZE 16
#define V2x2_BIND_COUNT 1000
// Timeout for callback in uSec, 4ms=4000us for V202
#define V2x2_PACKET_PERIOD 4000

static uint8_t V2x2_tx_id[3];
static uint8_t V2x2_rf_ch_num;
static uint16_t V2x2_flags;

// This is frequency hopping table for V202 protocol
// The table is the first 4 rows of 32 frequency hopping
// patterns, all other rows are derived from the first 4.
// For some reason the protocol avoids channels, dividing
// by 16 and replaces them by subtracting 3 from the channel
// number in this case.
// The pattern is defined by 5 least significant bits of
// sum of 3 bytes comprising TX id
static const uint8_t V2x2_freq_hopping[][16] = {
    { 0x27, 0x1B, 0x39, 0x28, 0x24, 0x22, 0x2E, 0x36,
      0x19, 0x21, 0x29, 0x14, 0x1E, 0x12, 0x2D, 0x18 }, //  00
    { 0x2E, 0x33, 0x25, 0x38, 0x19, 0x12, 0x18, 0x16,
      0x2A, 0x1C, 0x1F, 0x37, 0x2F, 0x23, 0x34, 0x10 }, //  01
    { 0x11, 0x1A, 0x35, 0x24, 0x28, 0x18, 0x25, 0x2A,
      0x32, 0x2C, 0x14, 0x27, 0x36, 0x34, 0x1C, 0x17 }, //  02
    { 0x22, 0x27, 0x17, 0x39, 0x34, 0x28, 0x2B, 0x1D,
      0x18, 0x2A, 0x21, 0x38, 0x10, 0x26, 0x20, 0x1F }  //  03
};
static uint8_t V2x2_rf_channels[16];

void V2x2_init()
{
    V2x2_set_tx_id();
    CE_off;
    CS_on;
    NRF24L01_Initialize();
    NRF24L01_Reset();
    // 2-bytes CRC, radio off
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, _BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO));
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknoledgement
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x3F);  // Enable all data pipes
    NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x03);   // 5-byte RX/TX address
    NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0xFF); // 4ms retransmit t/o, 15 tries
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, 0x08);      // Channel 8
    NRF24L01_SetBitrate(NRF24L01_BR_1M);
    NRF24L01_SetPower(RF_POWER);
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);     // Clear data ready, data sent, and retransmit
    NRF24L01_WriteReg(NRF24L01_0C_RX_ADDR_P2, 0xC3); // LSB byte of pipe 2 receive address
    NRF24L01_WriteReg(NRF24L01_0D_RX_ADDR_P3, 0xC4);
    NRF24L01_WriteReg(NRF24L01_0E_RX_ADDR_P4, 0xC5);
    NRF24L01_WriteReg(NRF24L01_0F_RX_ADDR_P5, 0xC6);
    NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, V2x2_PAYLOADSIZE);   // bytes of data payload for pipe 1
    NRF24L01_WriteReg(NRF24L01_12_RX_PW_P1, V2x2_PAYLOADSIZE);
    NRF24L01_WriteReg(NRF24L01_13_RX_PW_P2, V2x2_PAYLOADSIZE);
    NRF24L01_WriteReg(NRF24L01_14_RX_PW_P3, V2x2_PAYLOADSIZE);
    NRF24L01_WriteReg(NRF24L01_15_RX_PW_P4, V2x2_PAYLOADSIZE);
    NRF24L01_WriteReg(NRF24L01_16_RX_PW_P5, V2x2_PAYLOADSIZE);
    NRF24L01_WriteReg(NRF24L01_17_FIFO_STATUS, 0x00); // Just in case, no real bits to write here
    uint8_t V2x2_rx_tx_addr[] = {0x66, 0x88, 0x68, 0x68, 0x68};
    uint8_t V2x2_rx_p1_addr[] = {0x88, 0x66, 0x86, 0x86, 0x86};
    NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, V2x2_rx_tx_addr, 5);
    NRF24L01_WriteRegisterMulti(NRF24L01_0B_RX_ADDR_P1, V2x2_rx_p1_addr, 5);
    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, V2x2_rx_tx_addr, 5);
    delay(50);
    NRF24L01_FlushTx();
    V2x2_rf_ch_num = 0;
    // Turn radio power on
    NRF24L01_SetTxRxMode(TX_EN);
    uint8_t config = _BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP);
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, config);
    delay(150);
}

void V2x2_bind()
{
    uint16_t counter=1000;
    while(counter--) {
        V2x2_send_packet(1); 
        digitalWrite(ledPin, bitRead(counter,3)); //check for 0bxxxx1xxx to flash LED
        delayMicroseconds(V2x2_PACKET_PERIOD);
    } 
    V2x2_flags = 0; 
    digitalWrite(ledPin, HIGH); // LED on at end of bind  
}

uint32_t process_V2x2()
{
    uint32_t nextPacket = micros() + V2x2_PACKET_PERIOD;
    V2x2_send_packet(0);
    return nextPacket;   
}

void V2x2_set_tx_id()
{
    uint8_t sum;
    V2x2_tx_id[0] = transmitterID[0];
    V2x2_tx_id[1] = transmitterID[1];
    V2x2_tx_id[2] = transmitterID[2];
    sum = V2x2_tx_id[0] + V2x2_tx_id[1] + V2x2_tx_id[2];
    // Base row is defined by lowest 2 bits
    const uint8_t *fh_row = V2x2_freq_hopping[sum & 0x03];
    // Higher 3 bits define increment to corresponding row
    uint8_t increment = (sum & 0x1e) >> 2;
    for (uint8_t i = 0; i < 16; ++i) {
        uint8_t val = fh_row[i] + increment;
        // Strange avoidance of channels divisible by 16
        V2x2_rf_channels[i] = (val & 0x0f) ? val : val - 3;
    }
}

void V2x2_add_pkt_checksum()
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < 15;  ++i) sum += packet[i];
    packet[15] = sum;
}

void V2x2_set_flags(uint16_t* flags)
{
    int num_channels = CHANNELS;
    // Channel 5
    if (ppm[AUX1] <= PPM_MID) *flags &= ~V2x2_FLAG_LED;
    else *flags |= V2x2_FLAG_LED;
    
    // Channel 6
    if (ppm[AUX2] <= PPM_MID) *flags &= ~V2x2_FLAG_FLIP;
    else *flags |= V2x2_FLAG_FLIP;

    // Channel 7
    if (num_channels < 7 || ppm[AUX3] <= PPM_MID) *flags &= ~V2x2_FLAG_CAMERA;
    else *flags |= V2x2_FLAG_CAMERA;

    // Channel 8
    if (num_channels < 8 || ppm[AUX4] <= PPM_MID) *flags &= ~V2x2_FLAG_VIDEO;
    else *flags |= V2x2_FLAG_VIDEO;

    // Channel 9
    if (num_channels < 9 || ppm[AUX5] <= PPM_MID) *flags &= ~V2x2_FLAG_HEADLESS;
    else *flags |= V2x2_FLAG_HEADLESS;

    // Channel 10
    if (num_channels < 10 || ppm[AUX6] <= PPM_MID) *flags &= ~V2x2_FLAG_MAG_CAL_X;
    else *flags |= V2x2_FLAG_MAG_CAL_X;

    // Channel 11
    if (num_channels < 11 || ppm[AUX7] <= PPM_MID) *flags &= ~V2x2_FLAG_MAG_CAL_Y;
    else *flags |= V2x2_FLAG_MAG_CAL_Y;
}

uint8_t V2x2_convert_channel(uint8_t num)
{
    if(ppm[num]<PPM_MID)
        return map(ppm[num],PPM_MIN,PPM_MID,0x7F,0x00);
    else
        return map(ppm[num],PPM_MID,PPM_MAX,0x80,0xFF);
}

void V2x2_send_packet(uint8_t bind)
{
    if (bind) {
        V2x2_flags     = V2x2_FLAG_BIND;
        packet[0] = 0;
        packet[1] = 0;
        packet[2] = 0;
        packet[3] = 0;
        packet[4] = 0;
        packet[5] = 0;
        packet[6] = 0;
    } else {
        // regular packet
        V2x2_set_flags(&V2x2_flags);
        packet[0] = map(ppm[THROTTLE],PPM_MIN,PPM_MAX,0,255); // 0 - 255
        packet[1] = V2x2_convert_channel(RUDDER); // 7f - [00 - 80] - ff
        packet[2] = V2x2_convert_channel(ELEVATOR); // 7f - [00 - 80] - ff
        packet[3] = V2x2_convert_channel(AILERON); // 7f - [00 - 80] - ff
        // Trims, middle is 0x40, TODO: try dynamic trims, maybe faster yaw on v272 ?
        packet[4] = 0x40; // yaw
        packet[5] = 0x40; // pitch
        packet[6] = 0x40; // roll
    }
    // TX id
    packet[7] = V2x2_tx_id[0];
    packet[8] = V2x2_tx_id[1];
    packet[9] = V2x2_tx_id[2];
    // empty
    packet[10] = V2x2_flags >> 8;
    packet[11] = 0x00;
    packet[12] = 0x00;
    packet[13] = 0x00;
    //
    packet[14] = V2x2_flags & 0xff;
    V2x2_add_pkt_checksum();

    // Each packet is repeated twice on the same
    // channel, hence >> 1
    // We're not strictly repeating them, rather we
    // send new packet on the same frequency, so the
    // receiver gets the freshest command. As receiver
    // hops to a new frequency as soon as valid packet
    // received it does not matter that the packet is
    // not the same one repeated twice - nobody checks this
    uint8_t rf_ch = V2x2_rf_channels[V2x2_rf_ch_num >> 1];
    V2x2_rf_ch_num = (V2x2_rf_ch_num + 1) & 0x1F;
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, rf_ch);
    NRF24L01_FlushTx();
    NRF24L01_WritePayload(packet, V2x2_PAYLOADSIZE);
    delayMicroseconds(15);
}

