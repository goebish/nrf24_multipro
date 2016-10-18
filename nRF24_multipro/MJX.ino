
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

#define MJX_PACKET_SIZE     16
#define MJX_RF_NUM_CHANNELS 4
#define MJX_ADDRESS_LENGTH  5
#define MJX_BIND_COUNT      150
#define MJX_PACKET_PERIOD   4000 // Timeout for callback in uSec

enum {
    FORMAT_WLH08 = 0,
    FORMAT_X600,
    FORMAT_X800,
    FORMAT_H26D,
    FORMAT_E010,
};
u8 mjx_format;

#define MJX_CHANNEL_LED         AUX1
#define MJX_CHANNEL_FLIP        AUX2
#define MJX_CHANNEL_PICTURE     AUX3
#define MJX_CHANNEL_VIDEO       AUX4
#define MJX_CHANNEL_HEADLESS    AUX5
#define MJX_CHANNEL_RTH         AUX6
#define MJX_CHANNEL_AUTOFLIP    AUX7  // X800, X600
#define MJX_CHANNEL_PAN         AUX7  // H26D
#define MJX_CHANNEL_TILT        AUX8

static u16 mjx_counter;
static u8 mjx_rf_chan;
static u8 mjx_txid[3];
static u8 mjx_rf_channels[MJX_RF_NUM_CHANNELS];

// haven't figured out mjx_txid<-->rf channel mapping for MJX models
static const struct {
    u8 mjx_txid[3];
    u8 rfchan[MJX_RF_NUM_CHANNELS];
} 
mjx_tx_rf_map[] = {{{0xF8, 0x4F, 0x1C}, {0x0A, 0x46, 0x3A, 0x42}},
                  {{0xC8, 0x6E, 0x02}, {0x0A, 0x3C, 0x36, 0x3F}},
                  {{0x48, 0x6A, 0x40}, {0x0A, 0x43, 0x36, 0x3F}}};
                      
static const struct {
    u8 e010_txid[2];
    u8 rfchan[MJX_RF_NUM_CHANNELS];
}
e010_tx_rf_map[] = {{{0x4F, 0x1C}, {0x3A, 0x35, 0x4A, 0x45}},
                   {{0x90, 0x1C}, {0x2E, 0x36, 0x3E, 0x46}}, 
                   {{0x24, 0x36}, {0x32, 0x3E, 0x42, 0x4E}},
                   {{0x7A, 0x40}, {0x2E, 0x3C, 0x3E, 0x4C}},
                   {{0x61, 0x31}, {0x2F, 0x3B, 0x3F, 0x4B}},
                   {{0x5D, 0x37}, {0x33, 0x3B, 0x43, 0x4B}},
                   {{0xFD, 0x4F}, {0x33, 0x3B, 0x43, 0x4B}}, 
                   {{0x86, 0x3C}, {0x34, 0x3E, 0x44, 0x4E}}};

u8 mjx_checksum()
{
    u8 sum = packet[0];
    for (int i=1; i < MJX_PACKET_SIZE-1; i++) sum += packet[i];
    return sum;
}

// Channel values are sign + magnitude 8bit values
u8 mjx_convert_channel(u8 num)
{
    u8 val = map(ppm[num], PPM_MIN, PPM_MAX, 0, 255);
    return (val < 128 ? 127-val : val);	
}

#define PAN_TILT_COUNT     16   // for H26D - match stock tx timing
#define PAN_DOWN         0x08
#define PAN_UP           0x04
#define TILT_DOWN        0x20
#define TILT_UP          0x10

u8 mjx_pan_tilt_value()
{
    static u8 count;
    u8 pan = 0;
    
    count++;

    int32_t ch = ppm[MJX_CHANNEL_PAN];
    if ((ch < PPM_MIN_COMMAND || ch > PPM_MAX_COMMAND) && (count & PAN_TILT_COUNT))
        pan = ch < PPM_MID ? PAN_DOWN : PAN_UP;

    ch = ppm[MJX_CHANNEL_TILT];
    if ((ch < PPM_MIN_COMMAND || ch > PPM_MAX_COMMAND) && (count & PAN_TILT_COUNT))
        return pan + (ch < PPM_MID ? TILT_DOWN : TILT_UP);
    
    return pan;
}

#define CHAN2TRIM(X) (((X) & 0x80 ? (X) : 0x7f - (X)) >> 1)
void mjx_send_packet(u8 bind)
{
    packet[0] = map(ppm[THROTTLE], PPM_MIN, PPM_MAX, 0, 255);
    packet[1] = mjx_convert_channel(RUDDER);          // rudder
    packet[4] = 0x40;         // rudder does not work well with dyntrim
    packet[2] = mjx_convert_channel(ELEVATOR);   // elevator
    // driven trims cause issues when headless is enabled
    packet[5] = GET_FLAG(MJX_CHANNEL_HEADLESS, 1) ? 0x40 : CHAN2TRIM(packet[2]); // trim elevator
    packet[3] = mjx_convert_channel(AILERON);          // aileron
    packet[6] = GET_FLAG(MJX_CHANNEL_HEADLESS, 1) ? 0x40 : CHAN2TRIM(packet[3]); // trim aileron
    packet[7] = mjx_txid[0];
    packet[8] = mjx_txid[1];
    packet[9] = mjx_txid[2];

    packet[10] = 0;   // overwritten below for feature bits
    packet[11] = 0;   // overwritten below for X600
    packet[12] = 0;
    packet[13] = 0;

    packet[14] = 0xc0;  // bind value
    switch (mjx_format) {
        case FORMAT_H26D:
            packet[10] = mjx_pan_tilt_value();
            // fall through on purpose - no break
        case FORMAT_WLH08:
        case FORMAT_E010:
            packet[10] += GET_FLAG(MJX_CHANNEL_RTH, 0x02)
                        | GET_FLAG(MJX_CHANNEL_HEADLESS, 0x01);
            if (!bind) {
                packet[14] = 0x04
                | GET_FLAG(MJX_CHANNEL_FLIP, 0x01)
                | GET_FLAG(MJX_CHANNEL_PICTURE, 0x08)
                | GET_FLAG(MJX_CHANNEL_VIDEO, 0x10)
                | GET_FLAG_INV(MJX_CHANNEL_LED, 0x20); // air/ground mode
            }
        break;

        case FORMAT_X600:
            packet[10] = GET_FLAG_INV(MJX_CHANNEL_LED, 0x02);
            packet[11] = GET_FLAG(MJX_CHANNEL_RTH, 0x01);
            if (!bind) {
                packet[14] = 0x02      // always high rates by bit2 = 1
                | GET_FLAG(MJX_CHANNEL_FLIP, 0x04)
                | GET_FLAG(MJX_CHANNEL_AUTOFLIP, 0x10)
                | GET_FLAG(MJX_CHANNEL_HEADLESS, 0x20);
            }
        break;

        case FORMAT_X800:
        default:
            packet[10] = 0x10
            | GET_FLAG_INV(MJX_CHANNEL_LED, 0x02)
            | GET_FLAG(MJX_CHANNEL_AUTOFLIP, 0x01);
            if (!bind) {
                packet[14] = 0x02      // always high rates by bit2 = 1
                | GET_FLAG(MJX_CHANNEL_FLIP, 0x04)
                | GET_FLAG(MJX_CHANNEL_PICTURE, 0x08)
                | GET_FLAG(MJX_CHANNEL_VIDEO, 0x10);
            }
    }
    packet[15] = mjx_checksum();

    // Power on, TX mode, 2byte CRC
    if (mjx_format == FORMAT_H26D) {
        NRF24L01_SetTxRxMode(TX_EN);
    } else {
        XN297_Configure(_BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP));
    }

    NRF24L01_WriteReg(NRF24L01_05_RF_CH, mjx_rf_channels[mjx_rf_chan++ / 2]);
    mjx_rf_chan %= 2 * sizeof(mjx_rf_channels);  // channels repeated

    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
    NRF24L01_FlushTx();

    if (mjx_format == FORMAT_H26D) {
        NRF24L01_WritePayload(packet, MJX_PACKET_SIZE);
    } else {
        XN297_WritePayload(packet, MJX_PACKET_SIZE);
    }
}

uint32_t process_MJX()
{
    uint32_t timeout = micros() + MJX_PACKET_PERIOD;
    mjx_send_packet(0);
    return timeout;
}

void initialize_mjx_txid()
{
    if (mjx_format == FORMAT_E010) {
        memcpy(mjx_txid, e010_tx_rf_map[transmitterID[0] % (sizeof(e010_tx_rf_map)/sizeof(e010_tx_rf_map[0]))].e010_txid, 2);
        mjx_txid[2] = 0x00;
    }
    else if (mjx_format == FORMAT_WLH08) {
        // mjx_txid must be multiple of 8
        mjx_txid[0] = transmitterID[0] & 0xf8;
        mjx_txid[1] = transmitterID[1];
        mjx_txid[2] = transmitterID[2];
    } else {
        memcpy(mjx_txid, mjx_tx_rf_map[transmitterID[0] % (sizeof(mjx_tx_rf_map)/sizeof(mjx_tx_rf_map[0]))].mjx_txid, sizeof(mjx_txid));
    }
}

void MJX_init()
{
    u8 rx_tx_addr[MJX_ADDRESS_LENGTH];
    
    if (current_protocol == PROTO_E010)
        mjx_format = FORMAT_E010;
    else
        mjx_format = FORMAT_X600; // change this to desired MJX sub format
    
    initialize_mjx_txid();
    
    memcpy(rx_tx_addr, "\x6d\x6a\x77\x77\x77", sizeof(rx_tx_addr));
    if (mjx_format == FORMAT_WLH08) {
        memcpy(mjx_rf_channels, "\x12\x22\x32\x42", sizeof(mjx_rf_channels));
    } else if (mjx_format == FORMAT_H26D || mjx_format == FORMAT_E010) {
        memcpy(mjx_rf_channels, "\x36\x3e\x46\x2e", sizeof(mjx_rf_channels));
    } else {
        memcpy(mjx_rf_channels, "\x0a\x35\x42\x3d", sizeof(mjx_rf_channels));
        memcpy(rx_tx_addr, "\x6d\x6a\x73\x73\x73", sizeof(rx_tx_addr));
    }

    NRF24L01_Initialize();
    NRF24L01_SetTxRxMode(TX_EN);

    if (mjx_format == FORMAT_H26D) {
        NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, rx_tx_addr, sizeof(rx_tx_addr));
    } else {
        XN297_SetTXAddr(rx_tx_addr, sizeof(rx_tx_addr));
    }

    NRF24L01_FlushTx();
    NRF24L01_FlushRx();
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);     // Clear data ready, data sent, and retransmit
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknowldgement on all data pipes
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);
    NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0x00); // no retransmits
    NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, MJX_PACKET_SIZE);
    if (mjx_format == FORMAT_E010)
        NRF24L01_SetBitrate(NRF24L01_BR_250K);        // 250kbps
    else
        NRF24L01_SetBitrate(NRF24L01_BR_1M);          // 1Mbps
    NRF24L01_SetPower(RF_POWER);
    NRF24L01_Activate(0x73);                          // Activate feature register
    NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x00);       // Disable dynamic payload length on all pipes
    NRF24L01_WriteReg(NRF24L01_1D_FEATURE, 0x00);
    NRF24L01_Activate(0x73);
}

void mjx_init2()
{
    if (mjx_format == FORMAT_E010) {
        memcpy(mjx_rf_channels, e010_tx_rf_map[transmitterID[0] % (sizeof(e010_tx_rf_map)/sizeof(e010_tx_rf_map[0]))].rfchan, sizeof(mjx_rf_channels));
    }
    else if (mjx_format == FORMAT_H26D) {
        memcpy(mjx_rf_channels, "\x32\x3e\x42\x4e", sizeof(mjx_rf_channels));
    } else if (mjx_format != FORMAT_WLH08) {
        memcpy(mjx_rf_channels, mjx_tx_rf_map[transmitterID[0] % (sizeof(mjx_tx_rf_map)/sizeof(mjx_tx_rf_map[0]))].rfchan, sizeof(mjx_rf_channels));
    }
}

void MJX_bind()
{
    mjx_counter = MJX_BIND_COUNT;
    while(mjx_counter--) {
        mjx_send_packet(1);
        delayMicroseconds(MJX_PACKET_PERIOD);
        digitalWrite(ledPin, mjx_counter & 0x10);
    }
    mjx_init2();
    digitalWrite(ledPin, HIGH);
}
