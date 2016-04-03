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

#define HISKY_BIND_COUNT 300
#define HISKY_TXID_SIZE 5
#define HISKY_PACKET_PERIOD 1000
#define HISKY_FREQUENCE_NUM  20

uint8_t hisky_hopping_frequency[HISKY_FREQUENCE_NUM];
uint8_t hisky_hopping_frequency_no;
const uint8_t  hisky_binding_adr_rf[5]={0x12,0x23,0x23,0x45,0x78}; // fixed binding ids for all planes
uint8_t hisky_rf_adr_buf[5]; 
uint8_t hisky_binding_idx;
uint8_t hisky_bind_buf_array[4][10];
uint8_t hisky_counter1ms;
uint16_t hisky_bind_counter;

uint32_t process_HiSky()
{
    uint32_t nextPacket = micros() + HISKY_PACKET_PERIOD;
    
    hisky_counter1ms++;
    if(hisky_counter1ms==1)
        NRF24L01_FlushTx();
    else if(hisky_counter1ms==2) {
        if (hisky_bind_counter>0) {
            NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, (uint8_t *)hisky_binding_adr_rf, 5);
            NRF24L01_WriteReg(NRF24L01_05_RF_CH, 81);
        }
    }else if(hisky_counter1ms==3) {
        if (hisky_bind_counter >0)
        {
            hisky_bind_counter--;
            if (! hisky_bind_counter) // binding finished
                digitalWrite(ledPin, HIGH);
            else
                digitalWrite(ledPin, bitRead(hisky_bind_counter, 3));
            NRF24L01_WritePayload(hisky_bind_buf_array[hisky_binding_idx],10);
            hisky_binding_idx++;
            if (hisky_binding_idx >= 4)
                hisky_binding_idx = 0;
        }

    } else if (hisky_counter1ms==4) {
        if (hisky_bind_counter > 0)
            NRF24L01_FlushTx();
    }else if(hisky_counter1ms==5)
        NRF24L01_SetPower(RF_POWER);
    else if (hisky_counter1ms == 6) {
        NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, hisky_rf_adr_buf, 5);
        NRF24L01_WriteReg(NRF24L01_05_RF_CH, hisky_hopping_frequency[hisky_hopping_frequency_no]);
        hisky_hopping_frequency_no++;
        if (hisky_hopping_frequency_no >= HISKY_FREQUENCE_NUM)
            hisky_hopping_frequency_no = 0;
    }
    else if (hisky_counter1ms == 7) {
        HiSky_build_ch_data();
    }
    else if(hisky_counter1ms>8){
        hisky_counter1ms = 0;
        NRF24L01_WritePayload(packet,10);
    }    
        
    return nextPacket; 
}

void HiSky_init()
{
    for (uint8_t i = 0; i < HISKY_TXID_SIZE-1; ++i) {
        hisky_rf_adr_buf[i] = transmitterID[i];
    }
    hisky_rf_adr_buf[4] = transmitterID[0] / 2;
    HiSky_calc_fh_channels();

    HiSky_build_binding_packet();
    
    hisky_bind_counter = HISKY_BIND_COUNT;
    
    NRF24L01_Initialize();

    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);  // Enable p0 rx (unused)
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknowledgment
    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, hisky_rf_adr_buf, 5);
    NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, hisky_rf_adr_buf, 5);
    NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, 10); // payload size = 10
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, 81); // binding packet must be set in channel 81

    // 2-bytes CRC, radio off
    NRF24L01_SetTxRxMode(TX_EN);
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, _BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP));
    NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x03);   // 5-byte RX/TX address (byte -2)
    NRF24L01_SetBitrate(NRF24L01_BR_1M);   // 1Mbps
    NRF24L01_SetPower(RF_POWER);
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);     // Clear data ready, data sent, and retransmit
    delay(10);
}

// HiSky protocol uses TX id as an address for nRF24L01, and uses frequency hopping sequence
// which does not depend on this id and is passed explicitly in binding sequence. So we are free
// to generate this sequence as we wish. It should be in the range [02..77]
void HiSky_calc_fh_channels()
{
    int idx = 0;
    uint32_t seed = transmitterID[3] << 24
             | transmitterID[2] << 16
             | transmitterID[1] << 8
             | transmitterID[0];
    uint32_t rnd = seed;
    while (idx < HISKY_FREQUENCE_NUM) {
        int i;
        int count_2_26 = 0, count_27_50 = 0, count_51_74 = 0;
        rnd = rnd * 0x0019660D + 0x3C6EF35F; // Randomization

        // Use least-significant byte. 73 is prime, so channels 76..77 are unused
        uint8_t next_ch = ((rnd >> 8) % 73) + 2;
        // Keep the distance 2 between the channels - either odd or even
        if (((next_ch ^ seed) & 0x01 )== 0)
            continue;
        // Check that it's not duplicate and spread uniformly
        for (i = 0; i < idx; i++) {
            if(hisky_hopping_frequency[i] == next_ch)
                break;
            if(hisky_hopping_frequency[i] <= 26)
                count_2_26++;
            else if (hisky_hopping_frequency[i] <= 50)
                count_27_50++;
            else
                count_51_74++;
        }
        if (i != idx)
            continue;
        if ((next_ch <= 26 && count_2_26 < 8)
           ||(next_ch >= 27 && next_ch <= 50 && count_27_50 < 8)
           ||(next_ch >= 51 && count_51_74 < 8))
        {
            hisky_hopping_frequency[idx++] = next_ch;
        }
    }
}

void HiSky_build_binding_packet(void)
{
    uint8_t i;
    unsigned int  sum;
    uint8_t sum_l,sum_h;

    hisky_counter1ms = 0;
    hisky_hopping_frequency_no = 0;

    sum = 0;
    for(i=0;i<5;i++)
        sum += hisky_rf_adr_buf[i];
    sum_l = (uint8_t)sum;
    sum >>= 8;
    sum_h = (uint8_t)sum;
    hisky_bind_buf_array[0][0] = 0xff;
    hisky_bind_buf_array[0][1] = 0xaa;
    hisky_bind_buf_array[0][2] = 0x55;
    for(i=3;i<8;i++)
        hisky_bind_buf_array[0][i] = hisky_rf_adr_buf[i-3];

    for(i=1;i<4;i++)
    {
        hisky_bind_buf_array[i][0] = sum_l;
        hisky_bind_buf_array[i][1] = sum_h;
        hisky_bind_buf_array[i][2] = i-1;
    }
    for(i=0;i<7;i++)
        hisky_bind_buf_array[1][i+3] = hisky_hopping_frequency[i];
    for(i=0;i<7;i++)
        hisky_bind_buf_array[2][i+3] = hisky_hopping_frequency[i+7];
    for(i=0;i<6;i++)
        hisky_bind_buf_array[3][i+3] = hisky_hopping_frequency[i+14];

    hisky_binding_idx = 0;
}

// HiSky channel sequence: AILE  ELEV  THRO  RUDD  GEAR  PITCH, channel data value is from 0 to 1000
// Channel 7 - Gyro mode, 0 - 6 axis, 3 - 3 axis
void HiSky_build_ch_data()
{
    uint16_t temp;
    uint8_t i,j;
    const uint8_t ch[]={AILERON, ELEVATOR, THROTTLE, RUDDER, AUX1, AUX2, AUX3, AUX4};
    for (i = 0; i< 8; i++) {
        j=ch[i];
        temp=map(ppm[j], PPM_MIN, PPM_MAX, 0, 1000);
        if (j == THROTTLE) // It is clear that hisky's throttle stick is made reversely, so I adjust it here on purpose
            temp = 1000 -temp;
        if (j == AUX3)
            temp = temp < 400 ? 0 : 3; // Gyro mode, 0 - 6 axis, 3 - 3 axis
        packet[i] = (uint8_t)(temp & 0xFF);
        packet[i<4?8:9] >>= 2;
        packet[i<4?8:9] |= (temp >> 2) & 0xc0;
    }
}
