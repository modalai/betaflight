/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "common/utils.h"
#include "common/maths.h"

#include "drivers/bus.h"
#include "drivers/bus_spi.h"
#include "drivers/bus_spi_impl.h"
#include "drivers/dma.h"
#include "drivers/io.h"
#include "pg/bus_spi.h"

#include "sl_client.h"

SPI_TypeDef hexagon_spi_bus;

void spiInitDevice(SPIDevice device)
{
	printf("In spiInitDevice: %d", device);

	spiDevice[device].dev = &hexagon_spi_bus;

	hexagon_spi_bus.fd = sl_client_config_spi_bus();
}

// DMA transfer setup and start
void spiSequenceStart(const extDevice_t *dev)
{
	if (dev == NULL) {
		printf("**** NULL dev pointer");
		return;
	}

	busDevice_t *bus = dev->bus;

	if (bus == NULL) {
		printf("**** NULL bus pointer");
		return;
	}

	SPI_TypeDef *instance = bus->busType_u.spi.instance;

	if (instance == NULL) {
		printf("**** NULL instance pointer");
		return;
	}

	volatile struct busSegment_s *volatile curSegment = bus->curSegment;

	if (curSegment == NULL) {
		printf("**** NULL curSegment pointer");
		return;
	}

	bool single_segment = false;

	if (curSegment[0].negateCS == true) {
		if (!curSegment[0].u.buffers.txData || !curSegment[0].u.buffers.rxData) {
			printf("**** Only a single record but a buffer missing");
			return;
		} else {
			single_segment = true;
		}
	}

	if (!single_segment && curSegment[0].len != 1) {
		printf("**** Register address is not length 1");
		return;
	}

	if (!single_segment && curSegment[1].negateCS != true) {
		printf("**** More than 2 records");
		return;
	}

	int length = 0;
	if (single_segment) length = curSegment[0].len;
	else                length = curSegment[1].len;

	if (length > 12) {
		printf("**** Max length is 12. %d", length);
		return;
	}
	// TODO: Add more checks

	uint8_t send[2];
	uint8_t recv[16];

	// Send[0] has register address
	send[0] = *curSegment[0].u.buffers.txData;

	// If high bit on register address is set then it is a read
	bool is_read = (*curSegment[0].u.buffers.txData) & 0x80;

	if (!is_read) {
		if (length != 1) {
			printf("**** Only writes of length 1 are supported. %d", length);
			return;
		}
		if (single_segment) {
			printf("**** Single segment writes not supported");
			return;
		}
		recv[1] = *curSegment[1].u.buffers.txData;
		// printf("Writing 0x%0.2x to register 0x%0.2x", recv[1], send[0]);
	}

	int spi_fd = bus->busType_u.spi.instance->fd;

	int len = length;
	if (!single_segment) len = len + 1;

	sl_client_spi_transfer(spi_fd, &send[0], &recv[0], len);

	// if (is_read && length == 7) {
	// 	printf("0x%0.2x 0x%0.2x 0x%0.2x 0x%0.2x 0x%0.2x 0x%0.2x 0x%0.2x 0x%0.2x 0x%0.2x",
	// 		   (send[0] & 0x7f), recv[0], recv[1], recv[2], recv[3], recv[4], recv[5], recv[6], recv[7]);
	// }

	if (is_read) {
		uint8_t *destination = curSegment[1].u.buffers.rxData;
		uint8_t *source = &recv[1];
		if (single_segment) {
			destination = curSegment[0].u.buffers.rxData;
			source = recv;
		}
		memcpy(destination, source, length);
	}

	// This marks the transaction as completed
	dev->bus->curSegment = (busSegment_t *)BUS_SPI_FREE;
}

void spiPinConfigure(const spiPinConfig_t *pConfig)
{
	// printf("In spiPinConfigure");
	(void) pConfig;
}

uint16_t spiCalculateDivider(uint32_t freq)
{
	// printf("In spiCalculateDivider");
	(void) freq;
	return 1;
}

void spiInternalInitStream(const extDevice_t *dev, bool preInit)
{
	(void) dev;
	(void) preInit;
	printf("In spiInternalInitStream");
}

void spiInternalStartDMA(const extDevice_t *dev)
{
	(void) dev;
	printf("In spiInternalStartDMA");
}

bool spiInternalReadWriteBufPolled(SPI_TypeDef *instance, const uint8_t *txData, uint8_t *rxData, int len)
{
	(void) instance;
	(void) txData;
	(void) rxData;
	(void) len;
	printf("In spiInternalReadWriteBufPolled");
	return false;
}
