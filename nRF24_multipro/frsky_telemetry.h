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

#ifndef FRSKY_TELEMETRY_H_
#define FRSKY_TELEMETRY_H_

// FrSky new DATA IDs (2 bytes)
#define ALT_FIRST_ID              0x0100
#define ALT_LAST_ID               0x010f
#define VARIO_FIRST_ID            0x0110
#define VARIO_LAST_ID             0x011f
#define CURR_FIRST_ID             0x0200
#define CURR_LAST_ID              0x020f
#define VFAS_FIRST_ID             0x0210
#define VFAS_LAST_ID              0x021f
#define CELLS_FIRST_ID            0x0300
#define CELLS_LAST_ID             0x030f
#define T1_FIRST_ID               0x0400
#define T1_LAST_ID                0x040f
#define T2_FIRST_ID               0x0410
#define T2_LAST_ID                0x041f
#define RPM_FIRST_ID              0x0500
#define RPM_LAST_ID               0x050f
#define FUEL_FIRST_ID             0x0600
#define FUEL_LAST_ID              0x060f
#define ACCX_FIRST_ID             0x0700
#define ACCX_LAST_ID              0x070f
#define ACCY_FIRST_ID             0x0710
#define ACCY_LAST_ID              0x071f
#define ACCZ_FIRST_ID             0x0720
#define ACCZ_LAST_ID              0x072f
#define GPS_LONG_LATI_FIRST_ID    0x0800
#define GPS_LONG_LATI_LAST_ID     0x080f
#define GPS_ALT_FIRST_ID          0x0820
#define GPS_ALT_LAST_ID           0x082f
#define GPS_SPEED_FIRST_ID        0x0830
#define GPS_SPEED_LAST_ID         0x083f
#define GPS_COURS_FIRST_ID        0x0840
#define GPS_COURS_LAST_ID         0x084f
#define GPS_TIME_DATE_FIRST_ID    0x0850
#define GPS_TIME_DATE_LAST_ID     0x085f
#define A3_FIRST_ID               0x0900
#define A3_LAST_ID                0x090f
#define A4_FIRST_ID               0x0910
#define A4_LAST_ID                0x091f
#define AIR_SPEED_FIRST_ID        0x0a00
#define AIR_SPEED_LAST_ID         0x0a0f
#define RBOX_BATT1_FIRST_ID       0x0b00
#define RBOX_BATT1_LAST_ID        0x0b0f
#define RBOX_BATT2_FIRST_ID       0x0b10
#define RBOX_BATT2_LAST_ID        0x0b1f
#define RBOX_STATE_FIRST_ID       0x0b20
#define RBOX_STATE_LAST_ID        0x0b2f
#define RBOX_CNSP_FIRST_ID        0x0b30
#define RBOX_CNSP_LAST_ID         0x0b3f
#define DIY_FIRST_ID              0x5000
#define DIY_LAST_ID               0x52ff
#define DIY_STREAM_FIRST_ID       0x5000
#define DIY_STREAM_LAST_ID        0x50ff
#define RSSI_ID                   0xf101
#define ADC1_ID                   0xf102
#define ADC2_ID                   0xf103
#define SP2UART_A_ID              0xfd00
#define SP2UART_B_ID              0xfd01
#define BATT_ID                   0xf104
#define SWR_ID                    0xf105
#define XJT_VERSION_ID            0xf106
#define FUEL_QTY_FIRST_ID         0x0a10
#define FUEL_QTY_LAST_ID          0x0a1f

#endif /* FRSKY_TELEMETRY_H_ */

