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

static sem_t barometerReadSemaphore;
static bool barometerDataReady;
static uint8_t *barometerData;
static uint8_t barometerDataLen;
static bool barometerReadThreadStarted;

void *barometerReadThread(void *arg)
{
	(void) arg;

	printf("Barometer i2c read thread starting");

	while (true) {
		sem_wait(&barometerReadSemaphore);
    	// (void) sl_client_i2c_transfer(BARO_I2C_INSTANCE, NULL, 0, barometerData, barometerDataLen);
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
void i2cInit(I2CDevice device)
{
	int busDevice = i2cBusMap[device];

	// int fd = sl_client_config_i2c_bus(busDevice, 0, 100);
	int fd = -1;
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
	(void) device;
	(void) addr_;
	(void) cmd_;
	printf("In i2cWriteCommand16");
	// if (i2cBusAddr[device] != addr_) {
    //     sl_client_set_address_i2c_bus(device, addr_);
    //     i2cBusAddr[device] = addr_;
	// }
	// 
	// uint8_t buf[2];
	// buf[0] = (cmd_ >> 8) & 0xff;
	// buf[1] = cmd_ & 0xff;
    // return (sl_client_i2c_transfer(device, buf, 2, NULL, 0) == 0);
    return false;
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
	(void) device;
	(void) addr_;
	(void) reg_;
	(void) len_;
	(void) data;
	printf("In i2cWriteBuffer16");
	// if (len_ > 3) {
	// 	printf("ERROR: Cannot write %d bytes", len_);
	// 	return false;
	// }
	// 
	// if (i2cBusAddr[device] != addr_) {
    //     sl_client_set_address_i2c_bus(device, addr_);
    //     i2cBusAddr[device] = addr_;
	// }
	// 
	// uint8_t buf[5];
	// buf[0] = (reg_ >> 8) & 0xff;
	// buf[1] = reg_ & 0xff;
	// memcpy(&buf[2], data, len_);
    // return (sl_client_i2c_transfer(device, buf, len_ + 2, NULL, 0) == 0);
	return false;
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
	(void) device;
	(void) addr_;
	(void) reg_;
	(void) len;
	(void) buf;
	printf("In i2cRead16");
	// if (i2cBusAddr[device] != addr_) {
    //     sl_client_set_address_i2c_bus(device, addr_);
    //     i2cBusAddr[device] = addr_;
	// }
	// 
	// uint8_t regBuf[2];
	// regBuf[0] = (reg_ >> 8) & 0xff;
	// regBuf[1] = reg_ & 0xff;
    // return (sl_client_i2c_transfer(device, regBuf, 2, buf, len) == 0);
	return false;
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

bool i2cReadBufferNoRegister(I2CDevice device, uint8_t addr_, uint8_t len, uint8_t* buf)
{
	if (i2cBusAddr[device] != addr_) {
        // sl_client_set_address_i2c_bus(device, addr_);
        i2cBusAddr[device] = addr_;
	}

	if (device == BARO_I2C_INSTANCE) {
		if (!i2cBusBusy[BARO_I2C_INSTANCE]) {
			barometerDataReady = false;
			barometerData = buf;
			barometerDataLen = len;
			i2cBusBusy[BARO_I2C_INSTANCE] = true;
			sem_post(&barometerReadSemaphore);
		}
	}

	return true;
}

bool i2cBusy(I2CDevice device, bool *error)
{
	if (error) *error = false;

	if (device == BARO_I2C_INSTANCE) {
		if (barometerDataReady) {
			i2cBusBusy[BARO_I2C_INSTANCE] = false;
		}
	}

	// return i2cBusBusy[device];
	return false;
}

