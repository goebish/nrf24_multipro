/*
 * Bayang.h
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

#ifndef PROTO_BAYANG_H_
#define PROTO_BAYANG_H_

#define BAYANG_BIND_COUNT       1000
#define BAYANG_PACKET_PERIOD    2000
#define BAYANG_PACKET_SIZE      15
#define BAYANG_RF_NUM_CHANNELS  4
#define BAYANG_RF_BIND_CHANNEL  0
#define BAYANG_ADDRESS_LENGTH   5


class protBAYANG : public protAPI {
    public:
        protBAYANG() {};
        ~protBAYANG() {}

        void init(void);
        void bind(void);
        uint32_t loop(void);
};


#endif /* PROTO_BAYANG_H_ */
