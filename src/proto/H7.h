/*
 * H7.h
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

#ifndef PROTO_H7_H_
#define PROTO_H7_H_

#define H7_PACKET_PERIOD 2625
#define H7_PAYPLOAD_SIZE 9

class protH7 : public protAPI {
    public:
        protH7() {};
        ~protH7() {}

        void init(void);
        void bind(void);
        uint32_t loop(void);
};

#endif /* PROTO_H7_H_ */
