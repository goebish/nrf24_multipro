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

#define NOP() __asm__ __volatile__("nop")

void Read_Packet(uint8_t *data, uint8_t length) 
{
    uint8_t i;
    CS_off;
    spi_write(0x61); // Read RX payload
    for (i=0;i<length;i++) {
        data[i]=spi_read();
    }
    CS_on;
}

uint8_t spi_write(uint8_t command) 
{
    uint8_t result=0;
    uint8_t n=8;
    SCK_off;
    MOSI_off;
    while(n--) {
        if(command & 0x80)
            MOSI_on;
        else
            MOSI_off;
        if(MISO_on)
            result = (result<<1)|0x01;
        else
            result = result<<1;
        SCK_on;
        NOP();
        SCK_off;
        command = command << 1;
    }
    MOSI_on;
    return result;
}

void spi_write_address(uint8_t address, uint8_t data) 
{
    CS_off;
    spi_write(address);
    NOP();
    spi_write(data);
    CS_on;
}

// read one byte from MISO
uint8_t spi_read()
{
    uint8_t result=0;
    uint8_t i;
    MOSI_off;
    NOP();
    for(i=0;i<8;i++) {
        if(MISO_on) // if MISO is HIGH
        result = (result<<1)|0x01;
        else
        result = result<<1;
        SCK_on;
        NOP();
        SCK_off;
        NOP();
    }
    return result;
}

uint8_t spi_read_address(uint8_t address) 
{
    uint8_t result;
    CS_off;
    spi_write(address);
    result = spi_read();
    CS_on;
    return(result);
}
