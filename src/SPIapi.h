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

#ifdef SOFTSPI
void spi_config(int _pinMOSI, int _pinSCK, int _pinMISO, int _pinCE , int _pinCS);
#else
void spi_config(int _pinCE, int _pinCS);
#endif


void spi_begin(void);

uint8_t spi_read(void);
uint8_t spi_write(uint8_t command);
void spi_read_bytes(uint8_t *data, uint8_t length);
void spi_write_address(uint8_t address, uint8_t data);
uint8_t spi_read_address(uint8_t address);

uint8_t spi_write_byte(uint8_t byte);

void spi_write_address_bytes(uint8_t address, uint8_t * data, uint8_t len);
void spi_read_address_bytes(uint8_t address, uint8_t * data, uint8_t len);


void spi_CE_on(void);
void spi_CE_off(void);

#endif /* SPIAPI_H_ */
