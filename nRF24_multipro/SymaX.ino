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

#define SYMAX_BIND_COUNT 345
#define SYMAX_PACKET_PERIOD 4000
#define SYMAX_PACKET_SIZE 10
#define SYMAX_OLD_PACKET_SIZE 16
#define SYMAX_RF_CHANNELS 4
#define SYMAX_OLD_RF_CHANNELS 17

// flags going to packet[4]
#define SYMAX_FLAG_PICTURE  0x40
#define SYMAX_FLAG_VIDEO    0x80
// flags going to packet[6]
#define SYMAX_FLAG_FLIP     0x40
// flags going to packet[7]
#define SYMAX_FLAG_HEADLESS 0x80

static uint32_t SYMAX_packet_counter;
static uint8_t  SYMAX_rx_tx_addr[5];
static uint8_t  SYMAX_current_chan;
static uint8_t  SYMAX_chans[SYMAX_OLD_RF_CHANNELS];
static uint8_t  SYMAX_packet_size;
static uint8_t  SYMAX_num_rf_channel;

uint32_t process_SymaX()
{
    uint32_t nextPacket = micros() + SYMAX_PACKET_PERIOD;
    SYMAX_send_packet(false);
    return nextPacket;
}

void Symax_init()
{
    const uint8_t bind_rx_tx_addr[] = {0xab,0xac,0xad,0xae,0xaf};
    const uint8_t rx_tx_addr_x5c[] = {0x6d,0x6a,0x73,0x73,0x73};   // X5C uses same address for bind and data
    uint8_t first_packet[] = {0xf9, 0x96, 0x82, 0x1b, 0x20, 0x08, 0x08, 0xf2, 0x7d, 0xef, 0xff, 0x00, 0x00, 0x00, 0x00};
    uint8_t chans_bind[] = {0x4b, 0x30, 0x40, 0x20};     
    uint8_t chans_bind_x5c[] = {0x27, 0x1b, 0x39, 0x28, 0x24, 0x22, 0x2e, 0x36, 0x19, 0x21, 0x29, 0x14, 0x1e, 0x12, 0x2d, 0x18};
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
    if(current_protocol == PROTO_SYMAXOLD) {
        NRF24L01_SetBitrate(NRF24L01_BR_1M);
        SYMAX_packet_size = SYMAX_OLD_PACKET_SIZE;
    }
    else {
        SYMAX_packet_size = SYMAX_PACKET_SIZE;
        NRF24L01_SetBitrate(NRF24L01_BR_250K);
    }
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
    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, (current_protocol == PROTO_SYMAXOLD) ? rx_tx_addr_x5c : bind_rx_tx_addr, 5);
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
    
    if(current_protocol == PROTO_SYMAXOLD) {
        SYMAX_num_rf_channel = sizeof(chans_bind_x5c);
        memcpy(SYMAX_chans, chans_bind_x5c, SYMAX_num_rf_channel);
    }
    else {
        SYMAX_num_rf_channel = sizeof(chans_bind);
        memcpy(SYMAX_chans, chans_bind, SYMAX_num_rf_channel);
    }    
    SYMAX_current_chan = 0;
    SYMAX_packet_counter = 0;
    delay(12);
}

void SymaX_bind()
{
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

uint8_t SymaX_checksum(uint8_t *data)
{
    uint8_t sum = data[0];
    for (uint8_t i=1; i < SYMAX_packet_size-1; i++)
        if(current_protocol == PROTO_SYMAXOLD)
            sum += data[i];
        else
            sum ^= data[i];
    return sum + (current_protocol == PROTO_SYMAXOLD) ? 0 : 0x55;
}

// channels determined by last byte of tx address
void SYMAX_set_channels(u8 address) {
    static const uint8_t start_chans_1[] = {0x0a, 0x1a, 0x2a, 0x3a};
    static const uint8_t start_chans_2[] = {0x2a, 0x0a, 0x42, 0x22};
    static const uint8_t start_chans_3[] = {0x1a, 0x3a, 0x12, 0x32};
    uint8_t laddress = address & 0x1f;
    uint8_t i;
    uint32_t *pchans = (u32 *)SYMAX_chans;
    if (laddress < 0x10) {
        if (laddress == 6) laddress = 7;
        for(i=0; i < SYMAX_num_rf_channel; i++) {
            SYMAX_chans[i] = start_chans_1[i] + laddress;
        }
    } else if (laddress < 0x18) {
        for(i=0; i < SYMAX_num_rf_channel; i++) {
            SYMAX_chans[i] = start_chans_2[i] + (laddress & 0x07);
        }
        if (laddress == 0x16) {
            SYMAX_chans[0] += 1;
            SYMAX_chans[1] += 1;
        }
    } else if (laddress < 0x1e) {
        for(i=0; i < SYMAX_num_rf_channel; i++) {
            SYMAX_chans[i] = start_chans_3[i] + (laddress & 0x07);
        }
    } else if (laddress == 0x1e) {
        *pchans = 0x38184121;
    } else {
        *pchans = 0x39194121;
    } 
}

void SYMAX_build_packet(u8 bind) {
    if (bind) {
        packet[0] = SYMAX_rx_tx_addr[4];
        packet[1] = SYMAX_rx_tx_addr[3];
        packet[2] = SYMAX_rx_tx_addr[2];
        packet[3] = SYMAX_rx_tx_addr[1];
        packet[4] = SYMAX_rx_tx_addr[0];
        packet[5] = 0xaa;
        packet[6] = 0xaa;
        packet[7] = 0xaa;
        packet[8] = 0x00;
    } else {
        packet[0] = map(ppm[THROTTLE], PPM_MIN, PPM_MAX, 0x00, 0xFF); // throttle;
        if(ppm[ELEVATOR] < PPM_MID)
            packet[1] = map(ppm[ELEVATOR], PPM_MID, PPM_MIN, 0x80, 0xFF);
        else
            packet[1] = map(ppm[ELEVATOR], PPM_MID, PPM_MAX, 0x00, 0x7F);
        if(ppm[RUDDER] < PPM_MID)
            packet[2] = map(ppm[RUDDER], PPM_MID, PPM_MIN, 0x00, 0x7F);
        else
            packet[2] = map(ppm[RUDDER], PPM_MID, PPM_MAX, 0x80, 0xFF);
        if(ppm[AILERON] < PPM_MID)
            packet[3] = map(ppm[AILERON], PPM_MID, PPM_MIN, 0x00, 0x7F);
        else
            packet[3] = map(ppm[AILERON], PPM_MID, PPM_MAX, 0x80, 0xFF);
        packet[4] = 0;
        if(ppm[AUX4] > PPM_MAX_COMMAND)
            packet[4] |= SYMAX_FLAG_VIDEO;
        if(ppm[AUX3] > PPM_MAX_COMMAND)
            packet[4] |= SYMAX_FLAG_PICTURE;  
        // use trims to extend controls
        packet[5] = (packet[1] >> 2) | 0xc0;  // always high rates (bit 7 is rate control)
        packet[6] = (packet[2] >> 2);
        packet[7] = (packet[3] >> 2);
        if(ppm[AUX2] > PPM_MAX_COMMAND)
            packet[6] |= SYMAX_FLAG_FLIP;
        if(ppm[AUX5] > PPM_MAX_COMMAND)
            packet[7] |= SYMAX_FLAG_HEADLESS;
        packet[8] = 0x00;
    }
    packet[9] = SymaX_checksum(packet);
}

#define X5C_CHAN2TRIM(X) ((((X) & 0x80 ? 0xff - (X) : 0x80 + (X)) >> 2) + 0x20)

void SYMAXOLD_build_packet(uint8_t bind)
{
    if (bind) {
        memset(packet, 0, SYMAX_packet_size);
        packet[7] = 0xae;
        packet[8] = 0xa9;
        packet[14] = 0xc0;
        packet[15] = 0x17;
    } else {
        packet[0] = map(ppm[THROTTLE], PPM_MIN, PPM_MAX, 0x00, 0xFF); // throttle
        if(ppm[RUDDER] < PPM_MID)
            packet[1] = map(ppm[RUDDER], PPM_MID, PPM_MIN, 0x00, 0x7F);
        else
            packet[1] = map(ppm[RUDDER], PPM_MID, PPM_MAX, 0x80, 0xFF);
        if(ppm[ELEVATOR] < PPM_MID)
            packet[2] = map(ppm[ELEVATOR], PPM_MID, PPM_MIN, 0x80, 0xFF);
        else
            packet[2] = map(ppm[ELEVATOR], PPM_MID, PPM_MAX, 0x00, 0x7F);
        if(ppm[AILERON] < PPM_MID)
            packet[3] = map(ppm[AILERON], PPM_MID, PPM_MIN, 0x00, 0x7F);
        else
            packet[3] = map(ppm[AILERON], PPM_MID, PPM_MAX, 0x80, 0xFF);
        packet[4] = X5C_CHAN2TRIM(packet[1] ^ 0x80);     // drive trims for extra control range
        packet[5] = X5C_CHAN2TRIM(packet[2] ^ 0x80);
        packet[6] = X5C_CHAN2TRIM(packet[3] ^ 0x80);
        packet[7] = 0xae;
        packet[8] = 0xa9;
        packet[9] = 0x00;
        packet[10] = 0x00;
        packet[11] = 0x00;
        packet[12] = 0x00;
        packet[13] = 0x00;
        packet[14] = GET_FLAG(AUX4, 0x10)
                   | GET_FLAG(AUX3, 0x08)
                   | GET_FLAG(AUX2, 0x01)
                   | 0x04; // always high rates (bit 3 is rate control)
        packet[15] = SymaX_checksum(packet);
    }
}

void SYMAX_send_packet(uint8_t bind)
{
    if(current_protocol == PROTO_SYMAXOLD)
        SYMAXOLD_build_packet(bind);
    else
        SYMAX_build_packet(bind);
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, 0x2e);
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, SYMAX_chans[SYMAX_current_chan]);
    NRF24L01_FlushTx();
    NRF24L01_WritePayload(packet, SYMAX_packet_size);
    if (SYMAX_packet_counter++ % 2) {   // use each channel twice
        SYMAX_current_chan = (SYMAX_current_chan + 1) % SYMAX_num_rf_channel;
    }
}

void SYMAX_initialize_rx_tx_addr()
{
    for(uint8_t i=0; i<4; i++)
        SYMAX_rx_tx_addr[i] = transmitterID[i];
    SYMAX_rx_tx_addr[4] = 0xa2;
}
