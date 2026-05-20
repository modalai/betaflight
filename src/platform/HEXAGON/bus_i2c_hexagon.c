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
#include <pthread.h>
#include <semaphore.h>

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
static bool i2cBusBusy[3];

const i2cHardware_t i2cHardware[1];

i2cDevice_t i2cDevice[1];

// Async barometer read state. The HEXAGON sl_client_i2c_transfer call is
// synchronous and blocks for the duration of the transfer, so the periodic
// pressure read is dispatched to a worker thread to keep the scheduler
// loop responsive. Only the BARO_I2C_INSTANCE uses this path; other I2C
// transfers run synchronously in the caller's context.
static sem_t barometerReadSemaphore;
static bool barometerDataReady;
static uint8_t barometerReg;
static uint8_t *barometerData;
static uint8_t barometerDataLen;
static bool barometerReadThreadStarted;

void *barometerReadThread(void *arg)
{
	(void) arg;

	printf("Barometer i2c read thread starting");

	while (true) {
		sem_wait(&barometerReadSemaphore);
    	(void) sl_client_i2c_transfer(BARO_I2C_INSTANCE, &barometerReg, 1, barometerData, barometerDataLen);
		barometerDataReady = true;
	}

	return NULL;
}

/*
  4 I2C buses

  bus1: mag
  bus2: power manager
  bus5: barometer (internal)*
  bus4: external spare bus (unused) - Not on VOXL 2 mini
*/
void i2cInit(i2cDevice_e device)
{
	int busDevice = i2cBusMap[device];

	int fd = sl_client_config_i2c_bus(busDevice, 0, 100);
	printf("I2C %d %d FD %d", device, busDevice, fd);

	if (!barometerReadThreadStarted) {
		// Initialize the binary semaphore for inter-thread use, with an initial value of 1
		int result = sem_init(&barometerReadSemaphore, 0, 0);
		if (result != 0) {
		    perror("Semaphore initialization failed");
		    // Handle error
		}

		// Create i2c asynchronous read thread
	    int i2c_rx_priority = sched_get_priority_max(SCHED_FIFO) - 21;
	    printf("Setting i2c rx pthread priority to %d", i2c_rx_priority);

	    struct sched_param param = { .sched_priority = i2c_rx_priority };
	    pthread_attr_t attr;
	    pthread_attr_init(&attr);
	    pthread_attr_setschedparam(&attr, &param);
		
	    const uint32_t stack_size = 4096U;
	    pthread_attr_setstacksize(&attr, stack_size);

		pthread_t ctx = 0;
	    pthread_create(&ctx, &attr, &barometerReadThread, NULL);
	    pthread_attr_destroy(&attr);

		barometerReadThreadStarted = true;
	}
}

void i2cPinConfigure(const struct i2cConfig_s *i2cConfig)
{
	(void) i2cConfig;
}

uint16_t i2cGetErrorCounter(void)
{
    return 0;
}

#define I2C_WRITE_SCRATCH_MAX 32

static void i2cSelectAddress(i2cDevice_e device, uint8_t addr_)
{
	if (i2cBusAddr[device] != addr_) {
        sl_client_set_address_i2c_bus(device, addr_);
        i2cBusAddr[device] = addr_;
	}
}

// Blocking write of a single register
bool i2cWrite(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t data)
{
	i2cSelectAddress(device, addr_);
	uint8_t buf[2] = { reg_, data };
    return (sl_client_i2c_transfer(device, buf, sizeof(buf), NULL, 0) == 0);
}

// Blocking write of a register followed by a payload
bool i2cWriteBuffer(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data)
{
	if ((size_t)len_ + 1 > I2C_WRITE_SCRATCH_MAX) {
		printf("ERROR: i2cWriteBuffer payload too large (%u)", len_);
		return false;
	}

	i2cSelectAddress(device, addr_);
	uint8_t buf[I2C_WRITE_SCRATCH_MAX];
	buf[0] = reg_;
	memcpy(&buf[1], data, len_);
    return (sl_client_i2c_transfer(device, buf, len_ + 1, NULL, 0) == 0);
}

// Blocking register read
bool i2cRead(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{
	i2cSelectAddress(device, addr_);
    return (sl_client_i2c_transfer(device, &reg_, 1, buf, len) == 0);
}

// Non-blocking register read. For BARO_I2C_INSTANCE this hands the transfer
// off to the worker thread so the caller does not stall on sl_client_i2c_transfer.
// busBusy()/i2cBusy() reports completion once the worker sets barometerDataReady.
// Other instances fall back to a synchronous transfer.
bool i2cReadBuffer(i2cDevice_e device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{
	i2cSelectAddress(device, addr_);

	if (device == BARO_I2C_INSTANCE) {
		if (!i2cBusBusy[BARO_I2C_INSTANCE]) {
			barometerDataReady = false;
			barometerReg = reg_;
			barometerData = buf;
			barometerDataLen = len;
			i2cBusBusy[BARO_I2C_INSTANCE] = true;
			sem_post(&barometerReadSemaphore);
		}
		return true;
	}

	return (sl_client_i2c_transfer(device, &reg_, 1, buf, len) == 0);
}

bool i2cBusy(i2cDevice_e device, bool *error)
{
	if (error) *error = false;

	if (device == BARO_I2C_INSTANCE) {
		if (barometerDataReady) {
			i2cBusBusy[BARO_I2C_INSTANCE] = false;
		}
	}

	return i2cBusBusy[device];
}

