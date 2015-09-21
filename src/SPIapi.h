/*
 * SPIapi.h
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
#ifndef SPIAPI_H_
#define SPIAPI_H_

uint8_t spi_read(void);
uint8_t spi_write(uint8_t command);
void spi_read_bytes(uint8_t *data, uint8_t length);
void spi_write_address(uint8_t address, uint8_t data);
uint8_t spi_read_address(uint8_t address);

#endif /* SPIAPI_H_ */
