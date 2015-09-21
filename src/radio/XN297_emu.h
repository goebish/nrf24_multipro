/*
 * XN297_emu.h
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


#ifndef RADIO_XN297_EMU_H_
#define RADIO_XN297_EMU_H_



void XN297_SetTXAddr(const uint8_t* addr, uint8_t len);
void XN297_SetRXAddr(const uint8_t* addr, uint8_t len);
void XN297_Configure(uint8_t flags);
uint8_t XN297_WritePayload(uint8_t* msg, uint8_t len);
uint8_t XN297_ReadPayload(uint8_t* msg, uint8_t len);

#endif /* RADIO_XN297_EMU_H_ */
