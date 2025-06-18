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

#include "drivers/nvic.h"
#include "drivers/io_impl.h"
#include "drivers/exti.h"

#include "sl_client.h"

static extiCallbackRec_t *exti_cb;
static extiHandlerCallback *exti_fn;

static int intrCB(int int1, void* ptr1, void* ptr2) {
	(void) int1;
	(void) ptr1;
	(void) ptr2;

	// printf("^^^ In interrupt callback ^^^");

	if (exti_fn) exti_fn(exti_cb);

	return 0;
}

void EXTIInit(void)
{
	// printf("In EXTIInit");
}

void EXTIHandlerInit(extiCallbackRec_t *cb, extiHandlerCallback *fn)
{
	exti_cb = cb;
	exti_fn = fn;

	// printf("In EXTIHandlerInit");
}

void EXTIConfig(IO_t io, extiCallbackRec_t *cb, int irqPriority, ioConfig_t config, extiTrigger_t trigger)
{
	(void) io;
	(void) cb;
	(void) irqPriority;
	(void) config;
	(void) trigger;
	// printf("In EXTIConfig");
}

void EXTIRelease(IO_t io)
{
	(void) io;
	// printf("In EXTIRelease");
}

void EXTIEnable(IO_t io)
{
	(void) io;
	// printf("In EXTIEnable");

	(void) sl_client_register_interrupt_callback(intrCB, NULL);
}

void EXTIDisable(IO_t io)
{
	(void) io;
	// printf("In EXTIDisable");
}


