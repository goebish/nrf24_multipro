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

#define FORMAT_YD717   0
#define FORMAT_SKYWLKR 1
#define FORMAT_XINXUN  2
#define FORMAT_NI_HUI  3
#define FORMAT_SYMAX2  4

// define desired sub-format
#define YD717_FORMAT FORMAT_SKYWLKR

#define YD717_PACKET_PERIOD   8000     // Timeout for callback in uSec
#define YD717_PACKET_CHKTIME   500     // Time to wait if packet not yet acknowledged or timed out
#define YD717_RF_CHANNEL 0x3C
#define YD717_BIND_COUNT 60

#define YD717_FLAG_FLIP     0x0F
#define YD717_FLAG_LIGHT    0x80
#define YD717_FLAG_PICTURE  0x40
#define YD717_FLAG_VIDEO    0x20
#define YD717_FLAG_HEADLESS 0x10

#define YD717_PAYLOADSIZE 8       // receive data pipes set to this size, but unused
#define YD717_MAX_PACKET_SIZE 9   // YD717 packets have 8-byte payload, Syma X4 is 9

static uint16_t YD717_counter;
static uint32_t YD717_packet_counter;
static uint8_t YD717_rx_tx_addr[5];
static uint8_t YD717_state;

enum {
    YD717_INIT1 = 0,
    YD717_BIND2,
    YD717_BIND3,
    YD717_DATA
};

// Packet ack status values
enum {
    PKT_PENDING = 0,
    PKT_ACKED,
    PKT_TIMEOUT
};

uint8_t YD717_packet_ack()
{
    switch (NRF24L01_ReadReg(NRF24L01_07_STATUS) & (_BV(NRF24L01_07_TX_DS) | _BV(NRF24L01_07_MAX_RT))) {
    case _BV(NRF24L01_07_TX_DS):
        return PKT_ACKED;
    case _BV(NRF24L01_07_MAX_RT):
        return PKT_TIMEOUT;
    }
    return PKT_PENDING;
}

void YD717_send_packet(uint8_t bind)
{
    if (bind) {
        packet[0]= YD717_rx_tx_addr[0]; // send data YD717_state address in first 4 bytes
        packet[1]= YD717_rx_tx_addr[1];
        packet[2]= YD717_rx_tx_addr[2];
        packet[3]= YD717_rx_tx_addr[3];
        packet[4] = 0x56;
        packet[5] = 0xAA;
        packet[6] = YD717_FORMAT == FORMAT_NI_HUI ? 0x00 : 0x32;
        packet[7] = 0x00;
    } else {
        packet[0] = map(ppm[THROTTLE], PPM_MIN, PPM_MAX, 0, 255);
        packet[1] = map(ppm[RUDDER], PPM_MIN, PPM_MAX, 0, 255);
        packet[3] = map(ppm[ELEVATOR], PPM_MIN, PPM_MAX, 0, 255);
        packet[4] = map(ppm[AILERON], PPM_MIN, PPM_MAX, 0, 255);
        // dynamic trims
        if(YD717_FORMAT == FORMAT_YD717) {
            packet[2] = packet[3] >> 1; // elevator trim
            packet[5] = packet[4] >> 1; // aileron trim
            packet[6] = packet[1] >> 1; // rudder trim
        } else {
            packet[2] = packet[1] >> 1; // rudder trim
            packet[5] = packet[3] >> 1; // elevator trim
            packet[6] = packet[4] >> 1; // aileron trim
        }
        packet[7] = GET_FLAG(AUX1, YD717_FLAG_LIGHT)
                  | GET_FLAG(AUX2, YD717_FLAG_FLIP)
                  | GET_FLAG(AUX3, YD717_FLAG_PICTURE)
                  | GET_FLAG(AUX4, YD717_FLAG_VIDEO)
                  | GET_FLAG(AUX5, YD717_FLAG_HEADLESS);
    }

    // clear packet status bits and TX FIFO
    NRF24L01_WriteReg(NRF24L01_07_STATUS, (_BV(NRF24L01_07_TX_DS) | _BV(NRF24L01_07_MAX_RT)));
    NRF24L01_FlushTx();

    if(YD717_FORMAT == FORMAT_YD717) {
        NRF24L01_WritePayload(packet, 8);
    } else {
        packet[8] = packet[0];  // checksum
        for(uint8_t i=1; i < 8; i++) packet[8] += packet[i];
            packet[8] = ~packet[8];
        NRF24L01_WritePayload(packet, 9);
    }
    ++YD717_packet_counter;
}

void YD717_initialize()
{
    NRF24L01_Initialize();

    // CRC, radio on
    NRF24L01_SetTxRxMode(TX_EN);
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, _BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_PWR_UP)); 
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x3F);      // Auto Acknoledgement on all data pipes
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x3F);  // Enable all data pipes
    NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x03);   // 5-byte RX/TX address
    NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0x1A); // 500uS retransmit t/o, 10 tries
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, YD717_RF_CHANNEL);      // Channel 3C
    NRF24L01_SetBitrate(NRF24L01_BR_1M);             // 1Mbps
    NRF24L01_SetPower(RF_POWER);
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);     // Clear data ready, data sent, and retransmit
    NRF24L01_WriteReg(NRF24L01_0C_RX_ADDR_P2, 0xC3); // LSB byte of pipe 2 receive address
    NRF24L01_WriteReg(NRF24L01_0D_RX_ADDR_P3, 0xC4);
    NRF24L01_WriteReg(NRF24L01_0E_RX_ADDR_P4, 0xC5);
    NRF24L01_WriteReg(NRF24L01_0F_RX_ADDR_P5, 0xC6);
    NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, YD717_PAYLOADSIZE);   // bytes of data payload for pipe 1
    NRF24L01_WriteReg(NRF24L01_12_RX_PW_P1, YD717_PAYLOADSIZE);
    NRF24L01_WriteReg(NRF24L01_13_RX_PW_P2, YD717_PAYLOADSIZE);
    NRF24L01_WriteReg(NRF24L01_14_RX_PW_P3, YD717_PAYLOADSIZE);
    NRF24L01_WriteReg(NRF24L01_15_RX_PW_P4, YD717_PAYLOADSIZE);
    NRF24L01_WriteReg(NRF24L01_16_RX_PW_P5, YD717_PAYLOADSIZE);
    NRF24L01_WriteReg(NRF24L01_17_FIFO_STATUS, 0x00); // Just in case, no real bits to write here
    NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x3F);       // Enable dynamic payload length on all pipes

    // this sequence necessary for module from stock tx
    NRF24L01_ReadReg(NRF24L01_1D_FEATURE);
    NRF24L01_Activate(0x73);                          // Activate feature register
    NRF24L01_ReadReg(NRF24L01_1D_FEATURE);
    NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x3F);       // Enable dynamic payload length on all pipes
    NRF24L01_WriteReg(NRF24L01_1D_FEATURE, 0x07);     // Set feature bits on

    NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, YD717_rx_tx_addr, 5);
    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, YD717_rx_tx_addr, 5);

    delay(50);
}

void YD717_init1()
{
    // for bind packets set address to prearranged value known to receiver
    uint8_t bind_rx_tx_addr[] = {0x65, 0x65, 0x65, 0x65, 0x65};
    if (YD717_FORMAT == FORMAT_SYMAX2)
        for(uint8_t i=0; i < 5; i++) bind_rx_tx_addr[i]  = 0x60;
    else if (YD717_FORMAT == FORMAT_NI_HUI)
        for(uint8_t i=0; i < 5; i++) bind_rx_tx_addr[i]  = 0x64;

    NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, bind_rx_tx_addr, 5);
    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, bind_rx_tx_addr, 5);
}

void YD717_init2()
{
    // set rx/tx address for data YD717_state
    NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, YD717_rx_tx_addr, 5);
    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, YD717_rx_tx_addr, 5);
}

uint32_t process_YD717()
{
    uint32_t timeout = micros();
    switch (YD717_state) {
    case YD717_INIT1:
        YD717_send_packet(0);      // receiver doesn't re-enter bind mode if connection lost...check if already bound
        YD717_state = YD717_BIND3;
        break;

    case YD717_BIND2:
        if (YD717_counter == 0) {
            if (YD717_packet_ack() == PKT_PENDING)
                return timeout + YD717_PACKET_CHKTIME; // packet send not yet complete
            YD717_init2();                       // change to data YD717_state rx/tx address
            YD717_send_packet(0);
            YD717_state = YD717_BIND3;
        } else {
            if (YD717_packet_ack() == PKT_PENDING)
                return timeout + YD717_PACKET_CHKTIME; // packet send not yet complete
            YD717_send_packet(1);
            YD717_counter -= 1;
        }
        break;

    case YD717_BIND3:
        switch (YD717_packet_ack()) {
        case PKT_PENDING:
            return timeout + YD717_PACKET_CHKTIME;     // packet send not yet complete
        case PKT_ACKED:
            YD717_state = YD717_DATA;
            break;
        case PKT_TIMEOUT:
            YD717_init1();                       // change to bind rx/tx address
            YD717_counter = YD717_BIND_COUNT;
            YD717_state = YD717_BIND2;
            YD717_send_packet(1);
        }
        break;

    case YD717_DATA:
        if (YD717_packet_ack() == PKT_PENDING)
            return timeout + YD717_PACKET_CHKTIME;     // packet send not yet complete
        YD717_send_packet(0);
        break;
    }
    return timeout + YD717_PACKET_PERIOD;              // Packet every 8ms
}

void set_rx_tx_addr(uint32_t id)
{
    YD717_rx_tx_addr[0] = (id >> 24) & 0xFF;
    YD717_rx_tx_addr[1] = (id >> 16) & 0xFF;
    YD717_rx_tx_addr[2] = (id >>  8) & 0xFF;
    YD717_rx_tx_addr[3] = (id >>  0) & 0xFF;
    YD717_rx_tx_addr[4] = 0xC1; // always uses first data port
}

void initialize_rx_tx_addr()
{
    set_rx_tx_addr((uint32_t)transmitterID[3] << 24
    | (uint32_t)transmitterID[2] << 16
    | transmitterID[1] << 8
    | transmitterID[0]);
}

void YD717_init()
{
    initialize_rx_tx_addr();
    YD717_packet_counter = 0;
    YD717_initialize();
    YD717_state = YD717_INIT1;
}
