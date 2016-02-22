/*
 * CX10_GreenBlue.h
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef PROTO_CX10_GREENBLUE_H_
#define PROTO_CX10_GREENBLUE_H_

#define CX10_GREEN_PACKET_LENGTH 15
#define CX10_BLUE_PACKET_LENGTH 19
#define CX10_BLUE_PACKET_PERIOD 6000
#define CX10_GREEN_PACKET_PERIOD 3000
#define CX10_GREEN_BIND_COUNT 1000
#define CX10_RF_BIND_CHANNEL 0x02
#define CX10_NUM_RF_CHANNELS    4

class protCX10_GREENBLUE : public protAPI {

    protected:
        t_protocols version;

    public:
        protCX10_GREENBLUE(t_protocols _version) {
            version = _version;
            CX10_current_chan = 0;
        };
        ~protCX10_GREENBLUE() {}

        void init(void);
        void bind(void);
        uint32_t loop(void);

    private:
        uint8_t CX10_txid[4]; // transmitter ID
        uint8_t CX10_freq[4]; // frequency hopping table
        uint8_t CX10_current_chan;
        uint8_t CX10_packet_length;
        uint32_t CX10_packet_period;
        void CX10_Write_Packet(uint8_t init);
};



#endif /* PROTO_CX10_GREENBLUE_H_ */
