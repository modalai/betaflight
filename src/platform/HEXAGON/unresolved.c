#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <errno.h>

#define _PROVIDE_POSIX_TIME_DECLS 1
#include <time.h>

#include "pg/pg.h"
#include "drivers/serial.h"
#include "drivers/io.h"
#include "pg/bus_spi.h"
#include "drivers/system.h"

#include "config/feature.h"
#include "config/config.h"
#include "config/config_streamer.h"
#include "config/config_streamer_impl.h"
#include "config/config_eeprom_impl.h"

// TODO: Why can't we use 806?
//       Seems to generate math problems in scheduler with 860...
#define HEXAGON_SYS_CLOCK_MULT 100

void *dmaDescriptors;

uint32_t SystemCoreClock;

bool useDshotTelemetry = false;

char _estack;
char _Min_Stack_Size;

const mcuTypeInfo_t *getMcuTypeInfo(void)
{
    static const mcuTypeInfo_t info = { .id = MCU_TYPE_SIMULATOR, .name = "SIMULATOR" };
    return &info;
}

// This is Posix but not in Qurt?
int nanosleep(const struct timespec *duration, struct timespec *_Nullable rem)
{
	(void) rem;

	if ((duration->tv_nsec < 1000) && (duration->tv_sec == 0)) {
		// Minimum sleep is 1uS
		qurt_timer_sleep(1);
	} else {
		qurt_timer_sleep((duration->tv_sec * 1000000) + (duration->tv_nsec / 1000));
	}

	return 0;
}

// Why isn't this in Qurt?
size_t strnlen(const char *s, size_t maxlen)
{
	(void) s;
	(void) maxlen;
	return 0;
}

static void microsleep(uint32_t usec)
{
    struct timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec = usec*1000UL;
    while (nanosleep(&ts, &ts) == -1 && errno == EINTR);
}

void delayMicroseconds(uint32_t us)
{
    microsleep(us);
}

void delayMicroseconds_real(uint32_t us) {
    delayMicroseconds(us);
}

void delay(uint32_t ms)
{
    uint64_t start = millis64();

    while ((millis64() - start) < ms) {
        microsleep(1000);
    }
}

static struct timespec start_time;

uint32_t clockMicrosToCycles(uint32_t micros)
{
    return micros * HEXAGON_SYS_CLOCK_MULT;
}

uint64_t micros64(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return 1.0e6*((ts.tv_sec + (ts.tv_nsec*1.0e-9)) - (start_time.tv_sec + (start_time.tv_nsec*1.0e-9)));
}

uint32_t micros(void)
{
    return micros64() & 0xFFFFFFFF;
}

uint32_t microsISR(void)
{
	return micros();
}

uint64_t millis64(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return 1.0e3*((ts.tv_sec + (ts.tv_nsec*1.0e-9)) - (start_time.tv_sec + (start_time.tv_nsec*1.0e-9)));
}

uint32_t millis(void)
{
    return millis64() & 0xFFFFFFFF;
}

uint32_t getCycleCounter(void)
{
    return (uint32_t) ((micros64() & 0xFFFFFFFF) * HEXAGON_SYS_CLOCK_MULT);
}

int32_t clockCyclesToMicros(int32_t clockCycles)
{
    return clockCycles / HEXAGON_SYS_CLOCK_MULT;
}

float clockCyclesToMicrosf(int32_t clockCycles)
{
    return (float) clockCyclesToMicros(clockCycles);
}

// virtual EEPROM
static FILE *eepromFd = NULL;

bool loadEEPROMFromFile(void)
{
    if (eepromFd != NULL) {
        fprintf(stderr, "[FLASH_Unlock] eepromFd != NULL\n");
        return false;
    }

    // open or create
    eepromFd = fopen(EEPROM_FILENAME, "r+");
    if (eepromFd != NULL) {
        // obtain file size:
        fseek(eepromFd, 0, SEEK_END);
        size_t lSize = ftell(eepromFd);
        rewind(eepromFd);

        size_t n = fread(eepromData, 1, sizeof(eepromData), eepromFd);
        if (n == lSize) {
            printf("[FLASH_Unlock] loaded '%s', size = %d / %d\n", EEPROM_FILENAME, lSize, sizeof(eepromData));
        } else {
            fprintf(stderr, "[FLASH_Unlock] failed to load '%s'\n", EEPROM_FILENAME);
            return false;
        }
    } else {
        printf("[FLASH_Unlock] created '%s', size = %d\n", EEPROM_FILENAME, sizeof(eepromData));
        if ((eepromFd = fopen(EEPROM_FILENAME, "w+")) == NULL) {
            fprintf(stderr, "[FLASH_Unlock] failed to create '%s'\n", EEPROM_FILENAME);
            return false;
        }

        if (fwrite(eepromData, sizeof(eepromData), 1, eepromFd) != 1) {
            fprintf(stderr, "[FLASH_Unlock] write failed: %s\n", strerror(errno));
            return false;
        }
    }
    return true;
}

void configUnlock(void)
{
    loadEEPROMFromFile();
}

void configLock(void)
{
    // flush & close
    if (eepromFd != NULL) {
        fseek(eepromFd, 0, SEEK_SET);
        fwrite(eepromData, 1, sizeof(eepromData), eepromFd);
        fclose(eepromFd);
        eepromFd = NULL;
        printf("[FLASH_Lock] saved '%s'\n", EEPROM_FILENAME);
    } else {
        fprintf(stderr, "[FLASH_Lock] eeprom is not unlocked\n");
    }
}

configStreamerResult_e configWriteWord(uintptr_t address, config_streamer_buffer_type_t *buffer)
{
	// (void) address;
	// (void) buffer;
    STATIC_ASSERT(CONFIG_STREAMER_BUFFER_SIZE == sizeof(uint32_t), "CONFIG_STREAMER_BUFFER_SIZE does not match written size");
	
    if ((address >= (uintptr_t)eepromData) && (address + sizeof(uint32_t) <= (uintptr_t)ARRAYEND(eepromData))) {
        memcpy((void*)address, buffer, sizeof(config_streamer_buffer_type_t));
        printf("[FLASH_ProgramWord]%p = %08lx\n", (void*)address, *((uint32_t*)address));
    } else {
        printf("[FLASH_ProgramWord]%p out of range!\n", (void*)address);
    }
    return CONFIG_RESULT_SUCCESS;
}

void indicateFailure(failureMode_e mode, int repeatCount)
{
    UNUSED(repeatCount);
    printf("Failure LED flash for: [failureMode]!!! %d\n", mode);
}

void failureMode(failureMode_e mode)
{
    printf("[failureMode]!!! %d\n", mode);
    while (1);
}

void debugInit(void)
{
    printf("debugInit\n");
}

void timerInit(void)
{
}

void unusedPinsInit(void)
{
    printf("unusedPinsInit\n");
}

// uint8_t mpuGyroReadRegister(const extDevice_t *dev, uint8_t reg)
// {
// 	(void) dev;
// 	(void) reg;
// 	return 0;
// }

void IOConfigGPIO(IO_t io, ioConfig_t cfg)
{
    UNUSED(io);
    UNUSED(cfg);
}

int IO_GPIOPortIdx(IO_t io)
{
    UNUSED(io);
    return -1;
}

bool IORead(IO_t io)
{
    UNUSED(io);
	return false;
}

void IOHi(IO_t io)
{
    UNUSED(io);
}

void IOLo(IO_t io)
{
    UNUSED(io);
}

void IOInitGlobal(void)
{
    // NOOP
}

IO_t IOGetByTag(ioTag_t tag)
{
    UNUSED(tag);
    return NULL;
}

// system
void systemInit(void)
{
    // int ret;
	// 
    clock_gettime(CLOCK_MONOTONIC, &start_time);
    printf("[system]Init...\n");
	// 
    SystemCoreClock = HEXAGON_SYS_CLOCK_MULT * 1e6;
	// 
    // if (pthread_mutex_init(&updateLock, NULL) != 0) {
    //     printf("Create updateLock error!\n");
    //     exit(1);
    // }
	// 
    // if (pthread_mutex_init(&mainLoopLock, NULL) != 0) {
    //     printf("Create mainLoopLock error!\n");
    //     exit(1);
    // }
	// 
    // ret = pthread_create(&tcpWorker, NULL, tcpThread, NULL);
    // if (ret != 0) {
    //     printf("Create tcpWorker error!\n");
    //     exit(1);
    // }
	// 
    // ret = udpInit(&pwmLink, simulator_ip, PORT_PWM, false);
    // printf("[SITL] init PwmOut UDP link to gazebo %s:%d...%d\n", simulator_ip, PORT_PWM, ret);
	// 
    // ret = udpInit(&pwmRawLink, simulator_ip, PORT_PWM_RAW, false);
    // printf("[SITL] init PwmOut UDP link to RF9 %s:%d...%d\n", simulator_ip, PORT_PWM_RAW, ret);
	// 
    // ret = udpInit(&stateLink, NULL, PORT_STATE, true);
    // printf("[SITL] start UDP server @%d...%d\n", PORT_STATE, ret);
	// 
    // ret = udpInit(&rcLink, NULL, PORT_RC, true);
    // printf("[SITL] start UDP server for RC input @%d...%d\n", PORT_RC, ret);
	// 
    // ret = pthread_create(&udpWorker, NULL, udpThread, NULL);
    // if (ret != 0) {
    //     printf("Create udpWorker error!\n");
    //     exit(1);
    // }
	// 
    // ret = pthread_create(&udpWorkerRC, NULL, udpRCThread, NULL);
    // if (ret != 0) {
    //     printf("Create udpRCThread error!\n");
    //     exit(1);
    // }
}

extern void hexagonReset();

void systemResetToBootloader(bootloaderRequestType_e requestType)
{
    UNUSED(requestType);

    printf("systemResetToBootloader!");
	hexagonReset();
}

void systemReset(void)
{
    printf("systemReset!");
	hexagonReset();
}

// Should go in separate IO source file
void IOWrite(IO_t io, bool hi)
{
	(void) io;
	(void) hi;
}

void IOToggle(IO_t io)
{
	(void) io;
}

PG_REGISTER_WITH_RESET_FN(serialPinConfig_t, serialPinConfig, PG_SERIAL_PIN_CONFIG, 0);

void pgResetFn_serialPinConfig(serialPinConfig_t *serialPinConfig)
{
	(void) serialPinConfig;
}

void uartPinConfigure(const serialPinConfig_t *pSerialPinConfig) {
	(void) pSerialPinConfig;
}

