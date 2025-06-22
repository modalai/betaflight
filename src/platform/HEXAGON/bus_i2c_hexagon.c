/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/nvic.h"
#include "drivers/time.h"

#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_impl.h"

#include "sl_client.h"

static uint8_t i2cBusAddr[3];
static uint8_t i2cBusMap[3] = {1, 2, 5};

const i2cHardware_t i2cHardware[1];

i2cDevice_t i2cDevice[1];

/*
  4 I2C buses

  bus1: mag
  bus2: power manager
  bus5: barometer (internal)*
  bus4: external spare bus (unused) - Not on VOXL 2 mini
*/
void i2cInit(I2CDevice device)
{
	int busDevice = i2cBusMap[device];

	int fd = sl_client_config_i2c_bus(busDevice, 0, 100);
	printf("I2C %d %d FD %d", device, busDevice, fd);
}

void i2cPinConfigure(const struct i2cConfig_s *i2cConfig)
{
	(void) i2cConfig;
}

uint16_t i2cGetErrorCounter(void)
{
    return 0;
}

// Blocking write
bool i2cWrite(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t data)
{
    (void) device;
    (void) addr_;
    (void) reg_;
    (void) data;
    return true;
}

bool i2cWriteCommand16(I2CDevice device, uint8_t addr_, uint16_t cmd_)
{
	if (i2cBusAddr[device] != addr_) {
        sl_client_set_address_i2c_bus(device, addr_);
        i2cBusAddr[device] = addr_;
	}

	uint8_t buff[2];
	buff[0] = (cmd_ >> 8) & 0xff;
	buff[1] = cmd_ & 0xff;
    return (sl_client_i2c_transfer(device, buff, 2, NULL, 0) == 0);
}

// Non-blocking write
bool i2cWriteBuffer(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data)
{
    (void) device;
    (void) addr_;
    (void) reg_;
    (void) len_;
    (void) data;
    return true;
}

bool i2cWriteBuffer16(I2CDevice device, uint8_t addr_, uint16_t reg_, uint8_t len_, uint8_t *data)
{
	if (len_ > 3) {
		printf("ERROR: Cannot write %d bytes", len_);
		return false;
	}

	if (i2cBusAddr[device] != addr_) {
        sl_client_set_address_i2c_bus(device, addr_);
        i2cBusAddr[device] = addr_;
	}

	uint8_t buf[5];
	buf[0] = (reg_ >> 8) & 0xff;
	buf[1] = reg_ & 0xff;
	memcpy(&buf[2], data, len_);
    return (sl_client_i2c_transfer(device, buf, len_ + 2, NULL, 0) == 0);
}


// Blocking read
bool i2cRead(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{
    (void) device;
    (void) addr_;
    (void) reg_;
    (void) len;
    (void) buf;
    return true;
}

bool i2cRead16(I2CDevice device, uint8_t addr_, uint16_t reg_, uint8_t len, uint8_t* buf)
{
	if (i2cBusAddr[device] != addr_) {
        sl_client_set_address_i2c_bus(device, addr_);
        i2cBusAddr[device] = addr_;
	}

	uint8_t buff[2];
	buff[0] = (reg_ >> 8) & 0xff;
	buff[1] = reg_ & 0xff;
    return (sl_client_i2c_transfer(device, buff, 2, (uint8_t*) buf, len) == 0);
}

// Non-blocking read
bool i2cReadBuffer(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{
    (void) device;
    (void) addr_;
    (void) reg_;
    (void) len;
    (void) buf;
    return true;
}

bool i2cRawReadBuffer(I2CDevice device, uint8_t addr_, uint8_t len, uint8_t* buf)
{
	if (i2cBusAddr[device] != addr_) {
        sl_client_set_address_i2c_bus(device, addr_);
        i2cBusAddr[device] = addr_;
	}

    return (sl_client_i2c_transfer(device, NULL, 0, buf, len) == 0);
}

bool i2cBusy(I2CDevice device, bool *error)
{
    (void) device;
    (void) error;
    return false;
}

