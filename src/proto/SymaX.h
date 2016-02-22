/*
 * SymaX.h
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

#ifndef PROTO_SYMAX_H_
#define PROTO_SYMAX_H_

#define SYMAX_BIND_COUNT 345
#define SYMAX_PACKET_PERIOD 4000
#define SYMAX_PACKET_SIZE 10
#define SYMAX_RF_CHANNELS 4

// flags going to packet[4]
#define SYMAX_FLAG_PICTURE  0x40
#define SYMAX_FLAG_VIDEO    0x80
// flags going to packet[6]
#define SYMAX_FLAG_FLIP     0x40
// flags going to packet[7]
#define SYMAX_FLAG_HEADLESS 0x80

class protSYMAX : public protAPI {
    public:
        protSYMAX() {};
        ~protSYMAX() {}

        void init(void);
        void bind(void);
        uint32_t loop(void);
};



#endif /*PROTO_SYMAX_H_ */
