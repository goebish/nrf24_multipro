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
    KN_WLTOYS = 0,
    KN_FEILUN,
};

enum {
    USE1MBPS_NO  = 0,
    USE1MBPS_YES = 1,
};

#define KN_FORMAT KN_WLTOYS
#define KN_USE1MBPS USE1MBPS_NO

#define KN_INIT_WAIT_MS  10000
#define KN_BINDING_PACKET_PERIOD  1000
#define WL_SENDING_PACKET_PERIOD  2000
#define FX_SENDING_PACKET_PERIOD  1200
#define WL_BIND_COUNT 500
#define FX_BIND_COUNT 200
#define KN_PAYLOADSIZE 16
#define KN_MAX_RF_CHANNEL 73
#define KN_RF_CH_COUNT 4
#define WL_PACKET_SEND_COUNT 5
#define FX_PACKET_SEND_COUNT 8
#define KN_TX_ADDRESS_SIZE 5
#define KN_NO_RF_CHANNEL_CHANGE -1

#define KN_FLAG_DR   (1 << 0) // rx dual rate
#define KN_FLAG_HOLD (1 << 1) // throttle hold
#define KN_FLAG_3D   (1 << 2) // stunt mode
#define KN_FLAG_GYRO (1 << 6) // 6G / 3G mode

enum {
    KN_STATE_PRE_BIND,
    KN_STATE_BINDING,
    KN_STATE_PRE_SEND,
    KN_STATE_SENDING,
};

static u8 kn_hopping_channels[KN_RF_CH_COUNT];
static u16 kn_sending_packet_period;
static u16 kn_bind_count;
static u8 kn_packet_send_count;
static u8 kn_tx_addr[KN_TX_ADDRESS_SIZE];
static u8 kn_tx_state = KN_STATE_PRE_BIND;

void kn_start_tx(u8 bind_yes)
{
    switch( KN_FORMAT) {
        case KN_WLTOYS:
            kn_sending_packet_period = WL_SENDING_PACKET_PERIOD;
            kn_bind_count = WL_BIND_COUNT;
            kn_packet_send_count = WL_PACKET_SEND_COUNT;
            break;
        case KN_FEILUN:
            kn_sending_packet_period = FX_SENDING_PACKET_PERIOD;
            kn_bind_count =FX_BIND_COUNT;
            kn_packet_send_count = FX_PACKET_SEND_COUNT;
            break;
    }
    
    kn_init(kn_tx_addr, kn_hopping_channels);
    if(bind_yes)
    {
        kn_tx_state = KN_STATE_PRE_BIND;
    } else {
        kn_tx_state = KN_STATE_PRE_SEND;
    }
    delayMicroseconds(KN_INIT_WAIT_MS);
}

u32 process_KN()
{
    u32 start = micros();
    static int32_t packet_sent = 0;
    static int32_t rf_ch_idx = 0;
    switch(kn_tx_state)
    {
        case KN_STATE_PRE_BIND:
            kn_bind_init(kn_tx_addr, kn_hopping_channels, packet);
            kn_tx_state = KN_STATE_BINDING;
            packet_sent = 0;
            //Do once, no break needed
        case KN_STATE_BINDING:
            if(packet_sent < kn_bind_count)
            {
                packet_sent++;
                kn_send_packet(packet, KN_NO_RF_CHANNEL_CHANGE);
                return start + KN_BINDING_PACKET_PERIOD;
            } else {
                kn_tx_state = KN_STATE_PRE_SEND;
                //Do once, no break needed
            }
        case KN_STATE_PRE_SEND:
            packet_sent = 0;
            kn_send_init(kn_tx_addr, packet);
            rf_ch_idx = 0;
            kn_tx_state = KN_STATE_SENDING;
            //Do once, no break needed
        case KN_STATE_SENDING:
            if(packet_sent >= kn_packet_send_count)
            {
                packet_sent = 0;
                rf_ch_idx++;
                if(rf_ch_idx >= KN_RF_CH_COUNT) rf_ch_idx = 0;
                    kn_update_packet_control_data(packet, 0, rf_ch_idx);
                    kn_send_packet(packet, kn_hopping_channels[rf_ch_idx]);
                } else {
                    kn_update_packet_send_count(packet, packet_sent, rf_ch_idx);
                    kn_send_packet(packet, KN_NO_RF_CHANNEL_CHANGE);
            }
            packet_sent++;
            return start + kn_sending_packet_period;
    }
    
    //Bad things happened, rest
    packet_sent = 0;
    kn_tx_state = KN_STATE_PRE_SEND;
    return start + kn_sending_packet_period;
}

void kn_init(u8 tx_addr[], u8 hopping_ch[])
{   
    kn_calculate_tx_addr(tx_addr);
    kn_calculate_freqency_hopping_channels(*((u32*)tx_addr), hopping_ch, tx_addr);
    
    NRF24L01_Initialize();

    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknowledgement
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);  // Enable data pipe 0
    NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x03);   // 5-byte RX/TX address
    NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0);    // Disable retransmit
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);     // Clear data ready, data sent, and retransmit
    NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, 0x20);   // bytes of data payload for pipe 0
    NRF24L01_SetPower(RF_POWER);

    NRF24L01_Activate(0x73);
    NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 1); // Dynamic payload for data pipe 0
    NRF24L01_WriteReg(NRF24L01_1D_FEATURE, _BV(NRF2401_1D_EN_DPL));
    
    NRF24L01_FlushTx();
    // Turn radio power on
    NRF24L01_SetTxRxMode(TX_EN);
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, _BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP));
    
}

void kn_calculate_freqency_hopping_channels(u32 seed, u8 hopping_channels[], u8 tx_addr[])
{
    u32 rnd = seed;
    u8 idx = 0;
    u8 i;
    switch( KN_FORMAT) {
        case KN_WLTOYS:
            while (idx < KN_RF_CH_COUNT) {
                rnd = rnd * 0x0019660D + 0x3C6EF35F; // Randomization
                // Drop least-significant byte for better randomization. Start from 1
                u8 next_ch = (rnd >> 8) % KN_MAX_RF_CHANNEL + 1;
                // Check that it's not duplicate nor adjacent
                for (i = 0; i < idx; i++) {
                    u8 ch = hopping_channels[i];
                    if( (ch <= next_ch + 1) && (ch >= next_ch - 1) ) break;
                }
                if (i != idx)
                    continue;
                hopping_channels[idx++] = next_ch;
            }
            break;
        case KN_FEILUN:
            hopping_channels[0] = tx_addr[0] + tx_addr[1] + tx_addr[2] + tx_addr[3] - 256;
            hopping_channels[1] = hopping_channels[0] + 32;
            hopping_channels[2] = hopping_channels[0];
            hopping_channels[3] = hopping_channels[1];
            break;

    }
}

void kn_calculate_tx_addr(u8 tx_addr[])
{
    u32 rnd = (uint32_t)transmitterID[3] << 24
            | (uint32_t)transmitterID[2] << 16
            | transmitterID[1] << 8
            | transmitterID[0];
    int32_t i;
    
    for (i=0; i<8; i++) {
        rnd = (rnd * 0x3eeb2c54a2f) + 0xE0F3AD019660;
    }

    switch( KN_FORMAT) {
        case KN_WLTOYS:
            *((u32*)tx_addr) = rnd;
            break;
        case KN_FEILUN:
            tx_addr[0] = rnd % 256;
            tx_addr[1] = 1 + rnd % (KN_MAX_RF_CHANNEL-33);
            rnd = (rnd * 0x3eeb2c54a2f) + 0xE0F3AD019660;
            if (tx_addr[0] + tx_addr[1] < 256)
                tx_addr[2] = 1 + rnd % (KN_MAX_RF_CHANNEL-33);
            else
                tx_addr[2] = 0x00;
            tx_addr[3] = 0x00;
            while((tx_addr[0] + tx_addr[1] + tx_addr[2] + tx_addr[3])<257) tx_addr[3] = tx_addr[3] + 1 + (rnd % (KN_MAX_RF_CHANNEL-33));
            break;

    }
    //The 5th byte is a constant, must be 'K'
    tx_addr[4] = 'K';
}

void kn_bind_init(u8 tx_addr[], u8 hopping_ch[], u8 bind_packet[])
{
    NRF24L01_SetBitrate(NRF24L01_BR_1M);
    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, (const u8*)"KNDZK", 5U);
    bind_packet[0]  = 'K';
    bind_packet[1]  = 'N';
    bind_packet[2]  = 'D';
    bind_packet[3]  = 'Z';
    //Use first four bytes of tx_addr
    bind_packet[4]  = tx_addr[0];
    bind_packet[5]  = tx_addr[1];
    bind_packet[6]  = tx_addr[2];
    bind_packet[7]  = tx_addr[3];
    
    switch( KN_FORMAT) {
        case KN_WLTOYS:
            bind_packet[8]  = hopping_ch[0];
            bind_packet[9]  = hopping_ch[1];
            bind_packet[10] = hopping_ch[2];
            bind_packet[11] = hopping_ch[3];
            break;
        case KN_FEILUN:
            bind_packet[8]  = 0x00;
            bind_packet[9]  = 0x00;
            bind_packet[10] = 0x00;
            bind_packet[11] = 0x00;
            break;
    }
    bind_packet[12] = 0x00;
    bind_packet[13] = 0x00;
    bind_packet[14] = 0x00;
    bind_packet[15] = (KN_USE1MBPS == USE1MBPS_YES) ? 0x01 : 0x00;
    
    //Set address and RF channel and send the first packet
    kn_send_packet(packet, 83);
}

void kn_send_packet(u8 packet[], int32_t rf_ch)
{
    if(rf_ch > 0)
    {
        NRF24L01_WriteReg(NRF24L01_05_RF_CH, (u8)rf_ch);
    }
    // Binding and sending period are 1 ms and 2 ms, the TX FIFO is guaranteed empty.
    NRF24L01_WritePayload(packet, KN_PAYLOADSIZE);
}

void kn_send_init(u8 tx_addr[], u8 packet[])
{
    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, tx_addr, KN_TX_ADDRESS_SIZE);
    u8 bit_rate = (KN_USE1MBPS == USE1MBPS_YES) ? NRF24L01_BR_1M : NRF24L01_BR_250K;
    NRF24L01_SetBitrate(bit_rate);
    kn_update_packet_control_data(packet, 0, 0);
}

void kn_update_packet_send_count(u8 packet[], int32_t packet_sent, int32_t rf_ch_idx )
{
    switch( KN_FORMAT) {
        case KN_WLTOYS:
            packet[13] = (packet_sent << 5) | (rf_ch_idx << 2);
            break;
        case KN_FEILUN:
            packet[13] = 0x00;
            break;
    }

}

void kn_update_packet_control_data(u8 packet[], int32_t packet_count, int32_t rf_ch_idx)
{
    u16 throttle, aileron, elevator, rudder;
    u8 flags=0;
    kn_read_controls(&throttle, &aileron, &elevator, &rudder, &flags);
    
    packet[0]  = (throttle >> 8) & 0xFF;
    packet[1]  = throttle & 0xFF;
    packet[2]  = (aileron >> 8) & 0xFF;
    packet[3]  = aileron  & 0xFF;
    packet[4]  = (elevator >> 8) & 0xFF;
    packet[5]  = elevator & 0xFF;
    packet[6]  = (rudder >> 8) & 0xFF;
    packet[7]  = rudder & 0xFF;
    // Trims, middle is 0x64 (100) range 0-200
    packet[8]  = map(ppm[AUX5], PPM_MIN, PPM_MAX, 0, 200); // 0x64; // Throttle trim
    packet[9]  = map(ppm[AUX6], PPM_MIN, PPM_MAX, 0, 200); // 0x64; // Aileron trim
    packet[10] = map(ppm[AUX7], PPM_MIN, PPM_MAX, 0, 200); // 0x64; // Elevator trim
    packet[11] = 0x64; // Rudder trim (neutral)
    packet[12] = flags;
    
    switch( KN_FORMAT) {
        case KN_WLTOYS:
            packet[13] = (packet_count << 5) | (rf_ch_idx << 2);
            break;
        case KN_FEILUN:
            packet[13] = 0x00;
            break;
    }
    packet[14] = 0x00;
    packet[15] = 0x00;
}

void kn_read_controls(u16* throttle, u16* aileron, u16* elevator, u16* rudder, u8* flags)
{
    *throttle = kn_convert_channel(THROTTLE);
    *aileron  = 1023 - kn_convert_channel(AILERON);
    *elevator = kn_convert_channel(ELEVATOR);
    *rudder   = 1023 - kn_convert_channel(RUDDER);
    
    *flags = GET_FLAG( AUX1, KN_FLAG_DR)
           | GET_FLAG( AUX2, KN_FLAG_HOLD)
           | GET_FLAG( AUX3, KN_FLAG_3D)
           | GET_FLAG( AUX4, KN_FLAG_GYRO);
}

u16 kn_convert_channel(u8 num)
{
    return map(ppm[num], PPM_MIN, PPM_MAX, 0, 1023);
}

