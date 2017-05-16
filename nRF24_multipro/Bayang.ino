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

#define BAYANG_BIND_COUNT       1000
#define BAYANG_PACKET_PERIOD    1000
#define BAYANG_PACKET_SIZE      15
#define BAYANG_RF_NUM_CHANNELS  4
#define BAYANG_RF_BIND_CHANNEL  0
#define BAYANG_ADDRESS_LENGTH   5

static uint8_t Bayang_rf_chan;
static uint8_t Bayang_rf_channels[BAYANG_RF_NUM_CHANNELS] = {0,};
static uint8_t Bayang_rx_tx_addr[BAYANG_ADDRESS_LENGTH];
static uint16_t Bayang_telemetry_count=0;
static uint16_t Bayang_last_telemetry_count=0;
static uint16_t Bayang_loopcount=0;
static uint16_t Bayang_count=0;

enum{
    // flags going to packet[2]
    BAYANG_FLAG_RTH      = 0x01,
    BAYANG_FLAG_HEADLESS = 0x02,
    BAYANG_FLAG_FLIP     = 0x08,
    BAYANG_FLAG_VIDEO    = 0x10,
    BAYANG_FLAG_SNAPSHOT = 0x20,
};

enum{
    // flags going to packet[3]
    BAYANG_FLAG_INVERT   = 0x80,
};

uint32_t process_Bayang()
{
    uint32_t timeout = micros() + BAYANG_PACKET_PERIOD;
    if(current_protocol == PROTO_BAYANG_SILVERWARE) {
        // telemetry is enabled
        Bayang_loopcount++;
        if (Bayang_loopcount > 1000) {
            //calculate telemetry reception packet rate - packets per second
            Bayang_loopcount = 0;
            telemetry_data.rssi = Bayang_telemetry_count * 2;
            // set updated if lower than previous, so an alarm can be used when telemetry lost but does not go on forever
            if (Bayang_telemetry_count < Bayang_last_telemetry_count)
                telemetry_data.updated = 1;
            Bayang_last_telemetry_count = Bayang_telemetry_count;
            Bayang_telemetry_count = 0;
        }

        if (Bayang_count == 0) {
            Bayang_send_packet(0);
            } else {
            Bayang_check_rx();
        }

        Bayang_count++;
        Bayang_count %= 5;
    }
    else {
        if (Bayang_count == 0)
            Bayang_send_packet(0);
        Bayang_count++;
        Bayang_count %= 2;
    }    
    return timeout;
}

void Bayang_init()
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
    NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, BAYANG_PACKET_SIZE);
    NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0x00); // no retransmits
    NRF24L01_SetBitrate(NRF24L01_BR_1M);             // 1Mbps
    NRF24L01_SetPower(RF_POWER);
    NRF24L01_Activate(0x73);                         // Activate feature register
    NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x00);      // Disable dynamic payload length on all pipes
    NRF24L01_WriteReg(NRF24L01_1D_FEATURE, 0x01);
    NRF24L01_Activate(0x73);
    delay(150);
}

void Bayang_bind()
{
    uint16_t counter = BAYANG_BIND_COUNT;
    while(counter) {
        if (Bayang_count == 0)
            Bayang_send_packet(1);
        Bayang_count++;
        Bayang_count %= 4;
        delayMicroseconds(BAYANG_PACKET_PERIOD);
        digitalWrite(ledPin, counter-- & 0x10);
    }
    XN297_SetTXAddr(Bayang_rx_tx_addr, BAYANG_ADDRESS_LENGTH);
    XN297_SetRXAddr(Bayang_rx_tx_addr, BAYANG_ADDRESS_LENGTH);
    digitalWrite(ledPin, HIGH);
}

#define DYNTRIM(chval) ((u8)((chval >> 2) & 0xfc))

void Bayang_send_packet(u8 bind)
{
    union {
        u16 value;
        struct {
            u8 lsb;
            u8 msb;
        } bytes;
    } chanval;

    if (bind) {
        if(current_protocol == PROTO_BAYANG_SILVERWARE)
            packet[0] = 0xa3;
        else
            packet[0] = 0xa4;
        memcpy(&packet[1], Bayang_rx_tx_addr, 5);
        memcpy(&packet[6], Bayang_rf_channels, 4);
        packet[10] = transmitterID[0];
        packet[11] = transmitterID[1];
    } else {
        packet[0] = 0xa5;
        packet[1] = 0xfa;   // normal mode is 0xf7, expert 0xfa
        packet[2] = GET_FLAG(AUX2, BAYANG_FLAG_FLIP)
                  | GET_FLAG(AUX5, BAYANG_FLAG_HEADLESS)
                  | GET_FLAG(AUX6, BAYANG_FLAG_RTH)
                  | GET_FLAG(AUX3, BAYANG_FLAG_SNAPSHOT)
                  | GET_FLAG(AUX4, BAYANG_FLAG_VIDEO);
        packet[3] = GET_FLAG(AUX1, BAYANG_FLAG_INVERT);
        chanval.value = map(ppm[AILERON], PPM_MIN, PPM_MAX, 0, 0x3ff);   // aileron
        packet[4] = chanval.bytes.msb + DYNTRIM(chanval.value);
        packet[5] = chanval.bytes.lsb;
        chanval.value = map(ppm[ELEVATOR], PPM_MIN, PPM_MAX, 0, 0x3ff);   // elevator
        packet[6] = chanval.bytes.msb + DYNTRIM(chanval.value);
        packet[7] = chanval.bytes.lsb;
        chanval.value = map(ppm[THROTTLE], PPM_MIN, PPM_MAX, 0, 0x3ff);   // throttle
        packet[8] = chanval.bytes.msb + 0x7c;
        packet[9] = chanval.bytes.lsb;
        chanval.value = map(ppm[RUDDER], PPM_MIN, PPM_MAX, 0, 0x3ff);   // rudder
        packet[10] = chanval.bytes.msb + DYNTRIM(chanval.value);
        packet[11] = chanval.bytes.lsb;
    }
    packet[12] = transmitterID[2];
    packet[13] = 0x0a;
    packet[14] = Bayang_checksum();
    
    NRF24L01_SetTxRxMode(TX_EN);
    XN297_Configure(_BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP));
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, bind ? BAYANG_RF_BIND_CHANNEL : Bayang_rf_channels[Bayang_rf_chan++]);
    Bayang_rf_chan %= sizeof(Bayang_rf_channels);
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
    NRF24L01_FlushTx();
    XN297_WritePayload(packet, BAYANG_PACKET_SIZE);
    
    if(current_protocol == PROTO_BAYANG_SILVERWARE) {
        // switch radio to rx, no crc
        delayMicroseconds(450);
        NRF24L01_SetTxRxMode(RX_EN);
        NRF24L01_WriteReg(NRF24L01_00_CONFIG, 0x03);
    }
}

static uint8_t Bayang_checksum()
{
    uint8_t sum = packet[0];
    for (uint8_t i = 1; i < BAYANG_PACKET_SIZE - 1; i++)
    sum += packet[i];
    return sum;
}

static uint8_t Bayang_check_rx()
{
    union {
        uint16_t value;
        struct {
            uint8_t lsb;
            uint8_t msb;
        } bytes;
    } chanval;

    if (NRF24L01_ReadReg(NRF24L01_07_STATUS) & _BV(NRF24L01_07_RX_DR)) {
        // data received from aircraft
        XN297_ReadPayload(packet, BAYANG_PACKET_SIZE);

        NRF24L01_WriteReg(NRF24L01_07_STATUS, 0xff);

        NRF24L01_FlushRx();
        // decode data , check sum is ok as well, since there is no crc
        if (packet[0] == 0x85 && packet[14] == Bayang_checksum()) {
            // uncompensated battery volts*100
            chanval.bytes.msb = packet[3] & 0x7;
            chanval.bytes.lsb = packet[4] & 0xff;
            telemetry_data.volt1 = chanval.value;
            Bayang_telemetry_count++;
            telemetry_data.updated = 1;
            telemetry_data.lastUpdate = millis();
            return 1;
        }                       // end telemetry received
    }
    return 0;
}

