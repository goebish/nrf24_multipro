/*
 * V2x2.h
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

#ifndef PROTO_V2X2_H_
#define PROTO_V2X2_H_

#define V2x2_PAYLOADSIZE 16
#define V2x2_BIND_COUNT 1000
// Timeout for callback in uSec, 4ms=4000us for V202
#define V2x2_PACKET_PERIOD 4000

class protV2x2 : public protAPI {
    public:
        protV2x2() {};
        ~protV2x2() {}

        void init(void);
        void bind(void);
        uint32_t loop(void);
};

#endif /* PROTO_V2X2_H_ */
