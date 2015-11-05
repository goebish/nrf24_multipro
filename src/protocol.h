/*
 * protocol.h
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

#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#include "radio/iface_nrf24l01.h"
#include "radio/XN297_emu.h"

extern uint8_t Ppacket[32];


// supported protocols
typedef enum {
    PROTO_V2X2 = 0,     //!< WLToys V2x2, JXD JD38x, JD39x, JJRC H6C, Yizhan Tarantula X6 ...
    PROTO_CG023,        //!< EAchine CG023, CG032, 3D X4
    PROTO_CX10_BLUE,    //!< Cheerson CX-10 blue board, newer red board, CX-10A, Floureon FX-10, CX-Stars (todo: add DM007 variant)
    PROTO_CX10_GREEN,   //!< Cheerson CX-10 green board
    PROTO_H7,           //!< EAchine H7, MoonTop M99xx
    PROTO_BAYANG,       //!< EAchine H8 mini, H10, BayangToys X6, X7, X9, JJRC JJ850
    PROTO_SYMAX5C1,     //!< Syma X5C-1 (not older X5C), X11, X11C, X12
    PROTO_YD829,        //!< YD-829, YD-829C, YD-822 ...
    PROTO_H7_HIGH,      //!< high speed mode for EAchine H7, MoonTop M99xx
    PROTO_END
} t_protocols;


class protAPI {
    protected:
        protAPI() {};

    public:
        virtual ~protAPI() {}

        virtual void init(void) = 0;
        virtual void bind(void) = 0;
        virtual uint32_t loop(void) = 0;
};

extern protAPI * protocols[PROTO_END];

#endif /* PROTOCOL_H_ */
