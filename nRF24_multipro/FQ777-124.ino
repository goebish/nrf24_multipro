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

/*
 * Author: Mike Morrison (bikemikem on rcgroups)
 * Thanks to btoschi on deviationtx forum for spi traces.
 */

#define FQ777124_BIND_COUNT       1000
#define FQ777124_PACKET_PERIOD    2000
#define FQ777124_PACKET_SIZE      8
#define FQ777124_RF_NUM_CHANNELS  4
#define FQ777124_ADDRESS_LENGTH   5


#define FQ777124_TRIM_CHAN_ROLL   AUX7
#define FQ777124_TRIM_CHAN_PITCH  AUX4

static uint8_t FQ777124_dpl_packet[32] = {0};
static uint8_t FQ777124_packet_count   = 0;

static uint8_t FQ777124_rf_chan;
static uint8_t FQ777124_rf_channels[FQ777124_RF_NUM_CHANNELS] = {0x4D, 0x43, 0x27, 0x07};
static uint8_t FQ777124_bind_addr [FQ777124_ADDRESS_LENGTH]   = {0xe7,0xe7,0xe7,0xe7,0x67};
static uint8_t FQ777124_rx_tx_addr[FQ777124_ADDRESS_LENGTH]   = {0xd3,0x45,0x00,0xe7,0x67};

uint8_t ssv_xor[] = {0x80,0x44,0x64,0x75,0x6C,0x71,0x2A,0x36,0x7C,0xF1,0x6E,0x52,0x9,0x9D,0x1F,0x78,0x3F,0xE1,0xEE,0x16,0x6D,0xE8,0x73,0x9,0x15,0xD7,0x92,0xE7,0x3,0xBA};


enum {
    // flags going to packet[6]
    // H7_FLAG_RATE0, // default rate, no flag
    FQ777124_FLAG_RETURN     = 0x40,  // 0x40 when not off, !0x40 when one key return
    FQ777124_FLAG_HEADLESS   = 0x04,
    FQ777124_FLAG_EXPERT     = 0x01,
    FQ777124_FLAG_FLIP       = 0x80,
};

uint32_t process_FQ777124()
{
    uint32_t timeout = micros() + FQ777124_PACKET_PERIOD;
    
    FQ777124_send_packet(0);
    return timeout;
}

void FQ777124_init()
{
    NRF24L01_Initialize();
    NRF24L01_SetTxRxMode(TX_EN);
    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, FQ777124_bind_addr, FQ777124_ADDRESS_LENGTH);
    NRF24L01_FlushTx();
    NRF24L01_FlushRx();
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknowldgement on all data pipes
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x00);
    NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x03);
    NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0x00); // no retransmits
    NRF24L01_SetBitrate(NRF24L01_BR_250K);             // 256kbps
    NRF24L01_SetPower(RF_POWER);
    NRF24L01_Activate(0x73);                         // Activate feature register
    NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x00);      // Disable dynamic payload length on all pipes
    NRF24L01_WriteReg(NRF24L01_1D_FEATURE, 0x01);
    NRF24L01_Activate(0x73);
    delay(150);

    for(uint8_t i=0; i<2; i++) {
        FQ777124_rx_tx_addr[i] = random() & 0xff;
    }
}

void FQ777124_bind()
{
    uint16_t counter = FQ777124_BIND_COUNT;
    while(counter) {
        FQ777124_send_packet(1);
        delayMicroseconds(FQ777124_PACKET_PERIOD);
        digitalWrite(ledPin, counter-- & 0x10);
    }
    
    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, FQ777124_rx_tx_addr, FQ777124_ADDRESS_LENGTH);
    digitalWrite(ledPin, HIGH);
}



// initially pass in uint16_t crc = 0x3c18;
uint16_t nrf_crc(uint8_t data[], uint8_t len, uint16_t crc)
{
    //uint16_t crc = 0x3c18;
    for (int i = 0; i < len; ++i)
    {
        crc ^= ((uint16_t)data[i]) << 8;
        for (uint8_t j = 8; j > 0; --j)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc = (crc << 1);
        }
    }
    return crc;
}

void ssv_pack_dpl(uint8_t addr[], uint8_t pid, uint8_t* len, uint8_t* payload, uint8_t* packed_payload)
{
    uint16_t pcf = (*len & 0x3f) << 3;
    pcf |= (pid & 0x3) << 1;
    pcf |= 0x00; // noack field
    
    uint8_t header[7] = {0};
    header[6] = pcf;
    header[5] = (pcf >> 7) | (addr[0] << 1);
    header[4] = (addr[0] >> 7) | (addr[1] << 1);
    header[3] = (addr[1] >> 7) | (addr[2] << 1);
    header[2] = (addr[2] >> 7) | (addr[3] << 1);
    header[1] = (addr[3] >> 7) | (addr[4] << 1);
    header[0] = (addr[4] >> 7);

    // calculate the crc
    union 
    {
        uint8_t bytes[2];
        uint16_t val;
    } crc;

    crc.val = 0x3c18;
    crc.val = nrf_crc(header, 7, crc.val);
    crc.val = nrf_crc(payload, *len, crc.val);

    // encode payload and crc
    // xor with this:
    uint8_t i = 0;
    for (i = 0; i < *len; ++i)
    {
        payload[i] ^= ssv_xor[i];
    }
    crc.bytes[1] ^= ssv_xor[i++];
    crc.bytes[0] ^= ssv_xor[i++];

    // pack the pcf, payload, and crc into packed_payload
    packed_payload[0] = pcf >> 1;
    packed_payload[1] = (pcf << 7) | (payload[0] >> 1);
    
    for (i = 0; i < *len - 1; ++i)
    {
        packed_payload[i+2] = (payload[i] << 7) | (payload[i+1] >> 1);
    }

    packed_payload[i+2] = (payload[i] << 7) | (crc.val >> 9);
    ++i;
    packed_payload[i+2] = (crc.val >> 1 & 0x80 ) | (crc.val >> 1 & 0x7F);
    ++i;
    packed_payload[i+2] = (crc.val << 7);

    *len += 4;
}



void FQ777124_send_packet(u8 bind)
{

    uint8_t packet_len = FQ777124_PACKET_SIZE;
    
    if (bind) {
        // 4,5,6 = address fields
        // last field is checksum of address fields
        packet[0] = 0x20;
        packet[1] = 0x15;
        packet[2] = 0x05;
        packet[3] = 0x06;
        packet[4] = FQ777124_rx_tx_addr[0];
        packet[5] = FQ777124_rx_tx_addr[1];
        packet[6] = FQ777124_rx_tx_addr[2];
        packet[7] = packet[4] + packet[5] + packet[6];
    } else {
        uint8_t trim_mod  = FQ777124_packet_count % 144;
        uint8_t trim_val  = 0;
        if (36 <= trim_mod && trim_mod < 72) // yaw
        {
          trim_val  = 0x20; // don't modify yaw trim
        }
        else if (108 < trim_mod && trim_mod) // pitch
        {
          trim_val = map(ppm[FQ777124_TRIM_CHAN_PITCH], PPM_MIN, PPM_MAX, 0x01, 0x3E) + 0xA0 - 0x1F;
        }
        else // roll
        {
          trim_val = map(ppm[FQ777124_TRIM_CHAN_ROLL], PPM_MIN, PPM_MAX, 0x01, 0x3E) + 0x60 - 0x1F;
        }
        // throt, yaw, pitch, roll, trims, flags/left button,00,right button
        //0-3 0x00-0x64
        //4 roll/pitch/yaw trims. cycles through one trim at a time - 0-40 trim1, 40-80 trim2, 80-C0 trim3 (center:  A0 20 60)
        //5 flags for throttle button, two buttons above throttle - def: 0x40
        //6 00 ??
        //7 checksum - add values in other fields 

        packet[0] = map(ppm[THROTTLE],PPM_MIN, PPM_MAX,0,0x64);
        packet[1] = map(ppm[RUDDER],PPM_MIN, PPM_MAX,0,0x64);
        packet[2] = map(ppm[ELEVATOR],PPM_MIN, PPM_MAX,0,0x64);
        packet[3] = map(ppm[AILERON],PPM_MIN, PPM_MAX,0,0x64);
        packet[4] = trim_val; // calculated above
        packet[5] = GET_FLAG(AUX2, FQ777124_FLAG_FLIP)
                  | GET_FLAG(AUX5, FQ777124_FLAG_HEADLESS)
                  | GET_FLAG_INV(AUX6, FQ777124_FLAG_RETURN)
                  | GET_FLAG(AUX1, FQ777124_FLAG_EXPERT);                          
                          
        packet[6] = 0x00;
        // calculate checksum
        uint8_t checksum = 0;
        for (int i = 0; i < 7; ++i)
            checksum += packet[i];
        packet[7] = checksum;

        FQ777124_packet_count++;
    }

    ssv_pack_dpl( (0 == bind) ? FQ777124_rx_tx_addr : FQ777124_bind_addr, FQ777124_rf_chan, &packet_len, packet, FQ777124_dpl_packet);
    
    NRF24L01_WriteReg(NRF24L01_00_CONFIG,_BV(NRF24L01_00_PWR_UP));
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, FQ777124_rf_channels[FQ777124_rf_chan++]);
    FQ777124_rf_chan %= sizeof(FQ777124_rf_channels);
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
    NRF24L01_FlushTx();
    NRF24L01_WritePayload(FQ777124_dpl_packet, packet_len);
    NRF24L01_WritePayload(FQ777124_dpl_packet, packet_len);
    NRF24L01_WritePayload(FQ777124_dpl_packet, packet_len);
}

