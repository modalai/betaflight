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
#include <sys/stat.h>
#include <stdbool.h>
#include <stdint.h>
#include <pthread.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/atomic.h"

#include "common/utils.h"
#include "drivers/inverter.h"
#include "drivers/nvic.h"

#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/serial_uart_impl.h"

#include "sl_client.h"

USART_TypeDef hexagon_uart[NUM_HEXAGON_UART];

const uartHardware_t uartHardware[UARTDEV_COUNT] = {
    {
        .identifier = SERIAL_PORT_USART1,
        .reg = USART1,
        .txBuffer = uart1TxBuffer,
        .rxBuffer = uart1RxBuffer,
        .txBufferSize = sizeof(uart1TxBuffer),
        .rxBufferSize = sizeof(uart1RxBuffer),
    },
    {
        .identifier = SERIAL_PORT_USART2,
        .reg = USART2,
        .txBuffer = uart1TxBuffer,
        .rxBuffer = uart1RxBuffer,
        .txBufferSize = sizeof(uart1TxBuffer),
        .rxBufferSize = sizeof(uart1RxBuffer),
    },
    {
        .identifier = SERIAL_PORT_USART3,
        .reg = USART3,
        .txBuffer = uart1TxBuffer,
        .rxBuffer = uart1RxBuffer,
        .txBufferSize = sizeof(uart1TxBuffer),
        .rxBufferSize = sizeof(uart1RxBuffer),
	},
    {
        .identifier = SERIAL_PORT_UART4,
        .reg = UART4,
        .txBuffer = uart1TxBuffer,
        .rxBuffer = uart1RxBuffer,
        .txBufferSize = sizeof(uart1TxBuffer),
        .rxBufferSize = sizeof(uart1RxBuffer),
	},
    {
        .identifier = SERIAL_PORT_UART5,
        .reg = UART5,
        .txBuffer = uart1TxBuffer,
        .rxBuffer = uart1RxBuffer,
        .txBufferSize = sizeof(uart1TxBuffer),
        .rxBufferSize = sizeof(uart1RxBuffer),
	},
    {
        .identifier = SERIAL_PORT_UART7,
        .reg = UART7,
        .txBuffer = uart1TxBuffer,
        .rxBuffer = uart1RxBuffer,
        .txBufferSize = sizeof(uart1TxBuffer),
        .rxBufferSize = sizeof(uart1RxBuffer),
	}
};

typedef enum {
	UART_FUNCTION_UNKNOWN,
	UART_FUNCTION_HW,
	UART_FUNCTION_MSP,
	UART_FUNCTION_OSD,
	UART_FUNCTION_ESC_SENSOR,
	UART_FUNCTION_BLACKBOX
} hexagonUartFunction_e;

// This is how to map the three hardware ports to actual DSP QUP index
static int hwIndexMap[3] = {2, 6, 7};

// Blackbox binary log file support
#define MAX_LOG_BUFFERS 10
#define MAX_LOG_BUFFER_SIZE 1024
static uint8_t log_buffers[MAX_LOG_BUFFERS][MAX_LOG_BUFFER_SIZE];
static int log_buffer;
static int log_buffer_index;
static int log_write_buffer;
static bool logging_initialized;
static bool log_data_received;
static const char dir_path[] = "/data/betaflight";
static const char logfile[] = "log.bin";
static char full_log_path[128];

// Receive buffer support for MSP data coming from betaflight configurator
#define VIRTUAL_RX_BUFFER_LEN 1024
static uint8_t virtual_rx_buffer[VIRTUAL_RX_BUFFER_LEN];
static uint32_t _rx_write = 0;
static uint32_t _rx_read = 0;
static pthread_mutex_t _lock = PTHREAD_MUTEX_INITIALIZER;

// Called by the SLPI LINK server when there is a new message for us from host side
int slpi_link_client_receive(const uint8_t *data, int data_len_in_bytes) __attribute__ ((visibility ("default")));

int slpi_link_client_receive(const uint8_t *data, int data_len_in_bytes)
{
	// printf("Got %d bytes from host side. First 3 bytes: 0x%0.2x 0x%0.2x 0x%0.2x",
	// 		data_len_in_bytes, data[0], data[1], data[2]);

	bool saved_data = true;
	pthread_mutex_lock(&_lock);
	if (_rx_write + data_len_in_bytes < VIRTUAL_RX_BUFFER_LEN) {
		memcpy(&virtual_rx_buffer[_rx_write], data, data_len_in_bytes);
		_rx_write += data_len_in_bytes;
	} else {
		saved_data = false;
	}
	pthread_mutex_unlock(&_lock);

	if (!saved_data) {
		printf("ERROR: Dropped %d incoming bytes on virtual serial port", data_len_in_bytes);
	}

    return 0;
}

static int getUARTHWindex(serialPortIdentifier_e port) {
	int hw_index = -1;

	switch (port) {
	case SERIAL_PORT_USART2:
		hw_index = 1;
		break;
	case SERIAL_PORT_USART3:
		hw_index = 2;
		break;
	default:
		printf("ERROR: Invalid port identifier in getUARTHWindex");
		break;
	}

	return hw_index;
}

static hexagonUartFunction_e getUARTfunction(serialPortIdentifier_e port) {
	hexagonUartFunction_e func = UART_FUNCTION_UNKNOWN;

	switch (port) {
	case SERIAL_PORT_USART1:
		func = UART_FUNCTION_MSP;
		break;
	case SERIAL_PORT_USART2:
		func = UART_FUNCTION_HW;
		break;
	case SERIAL_PORT_USART3:
		func = UART_FUNCTION_HW;
		break;
	case SERIAL_PORT_UART4:
		func = UART_FUNCTION_ESC_SENSOR;
		break;
	case SERIAL_PORT_UART5:
		func = UART_FUNCTION_OSD;
		break;
	case SERIAL_PORT_UART7:
		func = UART_FUNCTION_BLACKBOX;
		break;
	default:
		printf("ERROR: Invalid port identifier in getUARTfunction %d", port);
		break;
	}

	return func;
}

uint32_t hexagonSerialTotalRxWaiting(const serialPort_t *instance) {
	serialPortIdentifier_e port_number = instance->identifier;
	hexagonUartFunction_e uartFunction = getUARTfunction(port_number);
	uint32_t bytes_waiting = 0;

	if (uartFunction == UART_FUNCTION_MSP) {
		// Virtual port
		pthread_mutex_lock(&_lock);
		bytes_waiting = _rx_write - _rx_read;
		pthread_mutex_unlock(&_lock);
		// printf("%lu total bytes waiting on virtual serial port to read", bytes_waiting);
	} else if (uartFunction == UART_FUNCTION_HW) {
		int hw_index = getUARTHWindex(port_number);
		if (hw_index != -1) {
			(void) sl_client_uart_rx_available(uartHardware[hw_index].reg->fd, &bytes_waiting);
		}
	}

	return bytes_waiting;
}

uint8_t hexagonSerialRead(serialPort_t *instance) {
	serialPortIdentifier_e port_number = instance->identifier;
	hexagonUartFunction_e uartFunction = getUARTfunction(port_number);
	uint8_t byte_data = 0;

	if (uartFunction == UART_FUNCTION_MSP) {
		pthread_mutex_lock(&_lock);
		if (_rx_write > _rx_read) {
			byte_data = virtual_rx_buffer[_rx_read++];
			// If read is all caught up then put indexes back to beginning of buffer.
			if (_rx_read == _rx_write) {
				_rx_read = 0;
				_rx_write = 0;
			}
		}
		pthread_mutex_unlock(&_lock);
		// printf("Reading a byte from MSP virtual serial port buffer");
	} else if (uartFunction == UART_FUNCTION_HW) {
		int hw_index = getUARTHWindex(port_number);
		if (hw_index != -1) {
			(void) sl_client_uart_read(uartHardware[hw_index].reg->fd, (char*) &byte_data, 1);
		}
	} else {
		printf("Unknown port in hexagonSerialRead %d", port_number);
	}

	return byte_data;
}

bool hexagonIsSerialTransmitBufferEmpty(const serialPort_t *instance) {
	(void) instance;
	return true;
}

uint32_t hexagonSerialTotalTxFree(const serialPort_t *instance) {
	(void) instance;
	return 255;
}

static bool _reset = false;

static int osdPacketBufferIndex = 0;
static uint8_t osdPacketBuffer[256];
static int osdTxBufferIndex = 0;
static uint8_t osdTxBuffer[4096];
static bool osdFlush = false;
static pthread_mutex_t osd_mutex = PTHREAD_MUTEX_INITIALIZER;

static int mspTxBufferIndex = 0;
static uint8_t mspTxBuffer[4096];
static bool mspFlush = false;
static pthread_mutex_t msp_mutex = PTHREAD_MUTEX_INITIALIZER;

static bool tx_thread_started = false;

void *tx_thread(void *arg)
{
	(void) arg;

	while (true) {
		// Handle reset
		if (_reset) {
			delayMicroseconds(10000);
			uint8_t resetCmd[1] = { 0x42 };
			(void) sl_client_send_data(resetCmd, 1);
		}

		// Handle OSD processing
		if (osdFlush) {
			if (osdTxBufferIndex == 1) printf("OSD sending. %d bytes", osdTxBufferIndex);
			pthread_mutex_lock(&osd_mutex);
			(void) sl_client_send_data(osdTxBuffer, osdTxBufferIndex);
			osdFlush = false;
			osdTxBufferIndex = 1;
			pthread_mutex_unlock(&osd_mutex);
		}

		// Handle MSP processing for betaflight configurator connection
		if (mspFlush) {
			// printf("MSP sending. %d bytes", mspTxBufferIndex);
			pthread_mutex_lock(&msp_mutex);
			(void) sl_client_send_data(mspTxBuffer, mspTxBufferIndex);
			mspFlush = false;
			mspTxBufferIndex = 1;
			pthread_mutex_unlock(&msp_mutex);
		}

		// Handle log data processing. Flush data to file as needed

		// Setup the logging directory and the log file once we start
		// receiving log data
		if ((!logging_initialized) && (log_data_received)){
			full_log_path[0] = 0;
			struct stat statbuf;
		    if (stat(dir_path, &statbuf) == 0) {
				if (S_ISDIR(statbuf.st_mode)) {
					strcpy(full_log_path, dir_path);
					strcat(full_log_path, "/");
					strcat(full_log_path, logfile);
		    		printf("Setting up log file %s", full_log_path);
				    FILE *fp = fopen(full_log_path, "wb"); // Open the file in binary write mode
				    if (fp == NULL) {
		    			printf("Error opening the log file");
						full_log_path[0] = 0;
				    } else {
				    	fclose(fp); // Close the file
					}
				} else {
		    		printf("ERROR: %s exists but is not a directory", dir_path);
				}
		    }
			logging_initialized = true;
		}

		// TODO: Also consider the case where the buffer is only partially written
		//       when the drone is disarmed. Could attempt to recover that as well
		if ((full_log_path[0] != 0) && (log_buffer != log_write_buffer)) {
			while (log_buffer != log_write_buffer) {
				// printf("Writing buffer %d", log_write_buffer);
			    FILE *fp = fopen(full_log_path, "ab"); // Open the file in binary append mode
			    if (fp == NULL) {
	    			printf("Error opening the log file in write thread");
					full_log_path[0] = 0;
			    } else {
					fwrite(log_buffers[log_write_buffer], 1, MAX_LOG_BUFFER_SIZE, fp);
			    	fclose(fp); // Close the file to make sure it is flushed
				}
				log_write_buffer++;
				if (log_write_buffer == MAX_LOG_BUFFERS) log_write_buffer = 0;
			}
		}

		// Send virtual port updates no faster than every 1ms
		delayMicroseconds(1000);
	}

    return NULL;
}

void hexagonReset() {
	_reset = true;
}

void hexagonStartBuf(serialPort_t *instance) {
	(void) instance;
	// printf("hexagonStartBuf");
}

void hexagonEndBuf(serialPort_t *instance) {
	hexagonUartFunction_e uartFunction = getUARTfunction(instance->identifier);

	if (uartFunction == UART_FUNCTION_OSD) {
		if (osdTxBufferIndex + osdPacketBufferIndex > 4096) {
			printf("ERROR: OSD transmit buffer overflow");
		} else {
			// printf("Copying %d bytes into osdTxBuffer", osdPacketBufferIndex);
			pthread_mutex_lock(&osd_mutex);
			memcpy(&osdTxBuffer[osdTxBufferIndex], osdPacketBuffer, osdPacketBufferIndex);
			osdTxBufferIndex += osdPacketBufferIndex;
			osdFlush = true;
			pthread_mutex_unlock(&osd_mutex);
		}
		osdPacketBufferIndex = 0;
	}
}

void hexagonWriteBuf(serialPort_t *instance, const void *data, int count) {
	serialPortIdentifier_e port_number = instance->identifier;
	hexagonUartFunction_e uartFunction = getUARTfunction(port_number);

	if (uartFunction == UART_FUNCTION_MSP) {
		if (mspTxBufferIndex + count > 4096) {
			printf("ERROR: MSP tx buffer overflow");
		} else {
			pthread_mutex_lock(&msp_mutex);
			memcpy(&mspTxBuffer[mspTxBufferIndex], data, count);
			mspTxBufferIndex += count;
			mspFlush = true;
			pthread_mutex_unlock(&msp_mutex);
		}
	} else if (uartFunction == UART_FUNCTION_OSD) {
		if (osdPacketBufferIndex + count > 256) {
			printf("ERROR: OSD packet buffer overflow");
		} else {
			memcpy(&osdPacketBuffer[osdPacketBufferIndex], data, count);
			osdPacketBufferIndex += count;
		}
	} else if (uartFunction == UART_FUNCTION_HW) {
		int hw_index = getUARTHWindex(port_number);
		(void) sl_client_uart_write(uartHardware[hw_index].reg->fd, (const char*) data, (const unsigned) count);
	}
}

void hexagonSerialWrite(serialPort_t *instance, uint8_t ch) {
	if (getUARTfunction(instance->identifier) == UART_FUNCTION_BLACKBOX) {
		log_data_received = true;
		if (log_buffer_index == MAX_LOG_BUFFER_SIZE) {
			log_buffer_index = 0;
			log_buffer++;
			if (log_buffer == MAX_LOG_BUFFERS) log_buffer = 0;
		}
		log_buffers[log_buffer][log_buffer_index] = ch;
		log_buffer_index++;
	}
}

static struct serialPortVTable hexagon_uart_vtable = {
    // void (*serialWrite)(serialPort_t *instance, uint8_t ch);
    // uint32_t (*serialTotalRxWaiting)(const serialPort_t *instance);
    // uint32_t (*serialTotalTxFree)(const serialPort_t *instance);
    // uint8_t (*serialRead)(serialPort_t *instance);
    // void (*serialSetBaudRate)(serialPort_t *instance, uint32_t baudRate);
    // bool (*isSerialTransmitBufferEmpty)(const serialPort_t *instance);
    // void (*setMode)(serialPort_t *instance, portMode_e mode);
    // void (*setCtrlLineStateCb)(serialPort_t *instance, void (*cb)(void *instance, uint16_t ctrlLineState), void *context);
    // void (*setBaudRateCb)(serialPort_t *instance, void (*cb)(serialPort_t *context, uint32_t baud), serialPort_t *context);
    // void (*writeBuf)(serialPort_t *instance, const void *data, int count);
    // void (*beginWrite)(serialPort_t *instance);
    // void (*endWrite)(serialPort_t *instance);
	.serialWrite = hexagonSerialWrite,
	.serialTotalRxWaiting = hexagonSerialTotalRxWaiting,
	.serialTotalTxFree = hexagonSerialTotalTxFree,
	.serialRead = hexagonSerialRead,
	.serialSetBaudRate = NULL,
	.isSerialTransmitBufferEmpty = hexagonIsSerialTransmitBufferEmpty,
	.setMode = NULL,
	.setCtrlLineStateCb = NULL,
	.setBaudRateCb = NULL,
	.writeBuf = hexagonWriteBuf,
	.beginWrite = hexagonStartBuf,
	.endWrite = hexagonEndBuf
};

uartPort_t *serialUART(uartDevice_t *uartdev, uint32_t baudRate, portMode_e mode, portOptions_e options)
{
	(void)mode;
	(void)options;

    uartPort_t *uart_port = &uartdev->port;
	serialPort_t serial_port = uart_port->port;
	serialPortIdentifier_e port_number = serial_port.identifier;
	hexagonUartFunction_e uartFunction = getUARTfunction(port_number);

	if (uartFunction == UART_FUNCTION_HW) {
		int hw_index = getUARTHWindex(port_number);
		uartHardware[hw_index].reg->fd = -1;
		uint8_t sl_port_number = hwIndexMap[hw_index];
		int fd = sl_client_config_uart(sl_port_number, baudRate);
		printf("====== In serialUART. id %u port %u baudRate %lu", port_number, sl_port_number, baudRate);
		uartHardware[hw_index].reg->fd = fd;
		uart_port->USARTx = uartHardware[hw_index].reg;
	} else if (uartFunction == UART_FUNCTION_MSP) {
		printf("Configuring MSP virtual port");
	} else if (uartFunction == UART_FUNCTION_OSD) {
		printf("Configuring OSD virtual display port");
	} else if (uartFunction == UART_FUNCTION_ESC_SENSOR) {
		printf("Configuring ESC sensor virtual port");
	} else if (uartFunction == UART_FUNCTION_BLACKBOX) {
		printf("Configuring Blackbox virtual port");
	} else { // UART_FUNCTION_UNKNOWN
		printf("ERROR: Invalid port number %u", port_number);
		return NULL;
	}

	uartdev->port.port.vTable = &hexagon_uart_vtable;

	return uart_port;
}

void uartEnableTxInterrupt(uartPort_t *uartPort)
{
	int fd = uartPort->USARTx->fd;
	printf("====== In uartEnableTxInterrupt, fd %d", fd);
}

extern void registerTelemCallback(serialReceiveCallbackPtr cb);

void uartReconfigure(uartPort_t *uartPort)
{
	serialPort_t serial_port = uartPort->port;
	serialPortIdentifier_e port_number = serial_port.identifier;
	hexagonUartFunction_e uartFunction = getUARTfunction(port_number);

	printf("====== In uartReconfigure for function %d", uartFunction);

	if (uartFunction == UART_FUNCTION_HW) {
		int fd = uartPort->USARTx->fd;
		int hw_index = getUARTHWindex(port_number);
		printf("Reconfiguring HW port %d, fd %d", hw_index, fd);
    	(void) sl_client_register_uart_callback(fd, uartPort->port.rxCallback, uartPort->port.rxCallbackData);
	} else if (uartFunction == UART_FUNCTION_MSP) {
		printf("Reconfiguring MSP virtual port");
		mspTxBuffer[0] = 0x5A;
		mspTxBufferIndex = 1;
	} else if (uartFunction == UART_FUNCTION_ESC_SENSOR) {
		// For ESC telemetry
		registerTelemCallback(uartPort->port.rxCallback);
	} else if (uartFunction == UART_FUNCTION_OSD) {
		printf("Reconfiguring OSD virtual display port");
		// TODO: Perhaps move this init to somewhere else?
		if (!tx_thread_started) {
			osdTxBuffer[0] = 0xA5;
			osdTxBufferIndex = 1;

		    int osd_tx_priority = 128;
		    // int osd_tx_priority = sched_get_priority_max(SCHED_FIFO) - 1;
		    printf("Setting OSD Tx pthread priority to %d", osd_tx_priority);

		    struct sched_param param = { .sched_priority = osd_tx_priority };
		    pthread_attr_t attr;
		    pthread_attr_init(&attr);
		    pthread_attr_setschedparam(&attr, &param);
			
		    const uint32_t stack_size = 4096U;
		    pthread_attr_setstacksize(&attr, stack_size);
			
			pthread_t ctx = 0;
		    pthread_create(&ctx, &attr, &tx_thread, NULL);
		    pthread_attr_destroy(&attr);
			tx_thread_started = true;
		}
	}
}

void uartIrqHandler(uartPort_t *s)
{
	int fd = s->USARTx->fd;
	printf("====== In uartIrqHandler, fd %d", fd);
}

