/*
 * protocol.cpp
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
#include <Arduino.h>
#include "nrf24_multipro.h"
#include "protocol.h"

#include "proto/V2x2.h"
#include "proto/CG023.h"
#include "proto/Bayang.h"
#include "proto/CX10_GreenBlue.h"
#include "proto/H7.h"
#include "proto/SymaX.h"
#include "proto/H8_3D.h"

/**
 * global packed buffer
 */
uint8_t Ppacket[32];

protV2x2 V2x2;
protCG023 CG023 = protCG023(PROTO_CG023);
protBAYANG BAYANG;
protCX10_GREENBLUE CX10_BLUE = protCX10_GREENBLUE(PROTO_CX10_BLUE);
protCX10_GREENBLUE CX10_GREEN = protCX10_GREENBLUE(PROTO_CX10_GREEN);

protH7 H7 = protH7(PROTO_H7);
protH7 H7_HIGH = protH7(PROTO_H7_HIGH);

protSYMAX SYMAX5C1;
protCG023 YD829 = protCG023(PROTO_YD829);

proH8_3D H8_3D;

protAPI * protocols[PROTO_END] = {
    &V2x2,          //!< WLToys V2x2, JXD JD38x, JD39x, JJRC H6C, Yizhan Tarantula X6 ...
    &CG023,         //!< EAchine CG023, CG032, 3D X4
    &CX10_BLUE,     //!< Cheerson CX-10 blue board, newer red board, CX-10A, CX-10C, Floureon FX-10, CX-Stars (todo: add DM007 variant)
    &CX10_GREEN,    //!< Cheerson CX-10 green board
    &H7,            //!< EAchine H7, MoonTop M99xx
    &BAYANG,        //!<  EAchine H8(C) mini, H10, BayangToys X6, X7, X9, JJRC JJ850, Floureon H101
    &SYMAX5C1,      //!< Syma X5C-1 (not older X5C), X11, X11C, X12
    &YD829,         //!< YD-829, YD-829C, YD-822 ...
    &H7_HIGH,       //!< high speed mode for EAchine H7, MoonTop M99xx
    &H8_3D          //!< EAchine H8 mini 3D, JJRC H20, H22
};

