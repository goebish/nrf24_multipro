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

#ifndef PROTO_H8_3D_H_
#define PROTO_H8_3D_H_

#define H8_3D_PACKET_SIZE      20
#define H8_3D_RF_NUM_CHANNELS  4
#define H8_3D_ADDRESS_LENGTH   5
#define H8_3D_BIND_COUNT       1000
#define H8_3D_PACKET_PERIOD    1800 // Timeout for callback in uSec

class proH8_3D : public protAPI {
    public:
        proH8_3D() {};
        ~proH8_3D() {}

        void init(void);
        void bind(void);
        uint32_t loop(void);
};


#endif /* PROTO_H8_3D_H_ */
