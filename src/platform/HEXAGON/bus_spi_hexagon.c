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

struct spiResource_s hexagon_spi_bus;

#define HEXAGON_SPI_MAX_TRANSFER_LEN 512

void spiInitDevice(spiDevice_e device)
{
	printf("In spiInitDevice: %d", device);

	spiDevice[device].dev = &hexagon_spi_bus;

	hexagon_spi_bus.fd = sl_client_config_spi_bus();
}

// The BMI270 powers up in I2C mode and switches to SPI on the first
// CS rising edge. On HEXAGON the CS line is owned by SLPI, so the
// driver-level IOLo/IOHi toggle is a no-op; instead we issue a
// throwaway SPI read of CHIP_ID, which causes sl_client_spi_transfer()
// to drive CS through a full low->high transition. The data returned
// is undefined and is discarded by the caller.
bool hexagonSpiPerformBmi270DummyRead(void)
{
	uint8_t transferBuf[3] = {0x80, 0, 0};
	sl_client_spi_transfer(hexagon_spi_bus.fd, transferBuf, transferBuf, sizeof(transferBuf));
	return true;
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

	spiResource_t *instance = bus->busType_u.spi.instance;

	if (instance == NULL) {
		printf("**** NULL instance pointer");
		return;
	}

	volatile struct busSegment_s *volatile curSegment = bus->curSegment;

	if (curSegment == NULL) {
		printf("**** NULL curSegment pointer");
		return;
	}

	if (curSegment[0].len == 0) {
		printf("**** Zero length segment");
		return;
	}

	// Concatenate all segments up to and including the one that negates CS
	// into a single buffer. sl_client_spi_transfer is full-duplex and the
	// same buffer is reused for both directions; per-segment rxData is
	// distributed back below.
	uint8_t transferBuf[HEXAGON_SPI_MAX_TRANSFER_LEN] = {0};
	unsigned transferLen = 0;
	unsigned segmentCount = 0;

	for (volatile const struct busSegment_s *segment = curSegment; ; segment++) {
		if (segment->len == 0) {
			printf("**** Zero length terminator before negateCS");
			return;
		}

		if (transferLen + segment->len > sizeof(transferBuf)) {
			printf("**** Transfer too long. %u", transferLen + segment->len);
			return;
		}

		if (segment->u.buffers.txData) {
			memcpy(&transferBuf[transferLen], segment->u.buffers.txData, segment->len);
		} else {
			memset(&transferBuf[transferLen], 0, segment->len);
		}

		transferLen += segment->len;
		segmentCount++;

		if (segment->negateCS) {
			break;
		}
	}

	// A register transfer must start with the register address in the first byte.
	if (curSegment[0].u.buffers.txData == NULL) {
		printf("**** First segment missing tx buffer");
		return;
	}

	int spi_fd = bus->busType_u.spi.instance->fd;

	sl_client_spi_transfer(spi_fd, transferBuf, transferBuf, transferLen);

	unsigned offset = 0;
	for (volatile const struct busSegment_s *segment = curSegment; segmentCount > 0; segment++, segmentCount--) {
		if (segment->u.buffers.rxData) {
			memcpy(segment->u.buffers.rxData, &transferBuf[offset], segment->len);
		}
		offset += segment->len;
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

void spiInternalInitStream(const extDevice_t *dev, volatile busSegment_t *segment)
{
	(void) dev;
	(void) segment;
	printf("In spiInternalInitStream");
}

void spiInternalStartDMA(const extDevice_t *dev)
{
	(void) dev;
	printf("In spiInternalStartDMA");
}

bool spiInternalReadWriteBufPolled(spiResource_t *instance, const uint8_t *txData, uint8_t *rxData, int len)
{
	(void) instance;
	(void) txData;
	(void) rxData;
	(void) len;
	printf("In spiInternalReadWriteBufPolled");
	return false;
}
