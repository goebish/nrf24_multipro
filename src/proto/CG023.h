/*
 * CG023.h
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


#ifndef PROTO_CG023_H_
#define PROTO_CG023_H_

// compatible with EAchine 3D X4, CG023, Attop YD-829, YD-829C

#define CG023_PAYLOAD_SIZE 15
#define CG023_PACKET_PERIOD 8200 // interval of time between start of 2 Ppackets, in us
#define CG023_RF_BIND_CHANNEL 0x2D
#define YD829_PACKET_PERIOD  4100

class protCG023 : public protAPI {
    public:
        t_protocols version;

        protCG023(t_protocols _version) {
            version = _version;
        };

        ~protCG023() {}

        void init(void);
        void bind(void);
        uint32_t loop(void);
    private:
        uint16_t cg_packet_period;
        void CG023_WritePacket(uint8_t init);
};

#endif /* PROTO_CG023_H_ */
