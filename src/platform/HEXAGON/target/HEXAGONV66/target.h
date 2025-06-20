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

// SITL (software in the loop) simulator

#pragma once

#include <stdint.h>
#include <stddef.h>

#include "common/utils.h"

#include "/opt/hexagon-sdk/4.1.0.4-lite/rtos/qurt/computev66/include/qurt/qurt_timer.h"


#define USE_I2C_OLED_DISPLAY

#define TARGET_BOARD_IDENTIFIER "HEXAGON"

#define UART_TRAIT_AF_PIN 1
#define UART_TRAIT_AF_PORT 1
#define UARTHARDWARE_MAX_PINS 4

#define DEFIO_GPIOID__A 0
#define DEFIO_TAG__PA4 DEFIO_TAG_MAKE(DEFIO_GPIOID__A, 4)
#define GYRO_1_CS_PIN PA4

#define DEFIO_GPIOID__C 2
#define DEFIO_TAG__PC4 DEFIO_TAG_MAKE(DEFIO_GPIOID__C, 4)
#define GYRO_1_EXTI_PIN PC4

#define SIMULATOR_MULTITHREAD

#define SYSTEM_HSE_MHZ 0
#define DEFAULT_CPU_OVERCLOCK 1
#define DMA_RAM
#define DMA_RW_AXI
#define DMA_RAM_R
#define DMA_RAM_W
#define DMA_RAM_RW

#define DMA_DATA_ZERO_INIT
#define DMA_DATA
#define STATIC_DMA_DATA_AUTO

#define UART_TX_BUFFER_ATTRIBUTE
#define UART_RX_BUFFER_ATTRIBUTE
#define DMA_GetCurrDataCounter(dmaResource) ((uint16_t) 0)
#define FAST_IRQ_HANDLER

#define USE_ESC_SENSOR

// use simulatior's attitude directly
// disable this if wants to test AHRS algorithm
#undef USE_IMU_CALC

#undef USE_DSHOT

extern int ffs(int i);

extern char *strcasestr(const char *haystack, const char *needle);

extern size_t strnlen(const char s[], size_t maxlen);

// file name to save config
#define EEPROM_FILENAME "/data/betaflight/eeprom.bin"
#define CONFIG_IN_FILE
#define EEPROM_SIZE     32768

#define U_ID_0 0
#define U_ID_1 1
#define U_ID_2 2

#define TASK_GYROPID_DESIRED_PERIOD     500
#define BRUSHLESS_MOTORS_PWM_RATE 2000

#define SCHEDULER_DELAY_LIMIT           1

#undef USE_MULTI_GYRO
#define GYRO_1_SPI_INSTANCE SPI1

#define USE_ACC
#define USE_ACC_SPI_ICM42688P

#define USE_GYRO
#define USE_SPI_GYRO
#define USE_GYRO_SPI_ICM42688P

#define USE_MAG
#define USE_VIRTUAL_MAG

#define USE_BARO
#define USE_BARO_ICP10100

#define USE_GPS
#define USE_GPS_UBLOX
#define USE_GPS_LAP_TIMER
#define USE_GPS_RESCUE

#define USABLE_TIMER_CHANNEL_COUNT 0

#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_UART4
#define USE_UART5
#undef USE_UART6
#define USE_UART7
#undef USE_UART8

#define USE_SERIALRX
#define USE_SERIALRX_CRSF

#undef USE_DMA

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define DEFAULT_FEATURES        (FEATURE_GPS | FEATURE_TELEMETRY | FEATURE_ESC_SENSOR)

#define USE_PARAMETER_GROUPS

#ifndef USE_PWM_OUTPUT
#define USE_PWM_OUTPUT
#endif

#define USE_TELEMETRY

#undef USE_STACK_CHECK // I think SITL don't need this
#define USE_DASHBOARD
#undef USE_TELEMETRY_LTM
#undef USE_ADC
#undef USE_VCP
#define USE_OSD
#define USE_MSP_DISPLAYPORT
#define USE_OSD_OVER_MSP_DISPLAYPORT
#undef USE_RX_PPM
#undef USE_RX_PWM
#undef USE_SERIALRX_GHST
#undef USE_SERIALRX_IBUS
#undef USE_SERIALRX_SBUS
#undef USE_SERIALRX_SPEKTRUM
#undef USE_SERIALRX_SUMD
#undef USE_SERIALRX_SUMH
#undef USE_SERIALRX_XBUS
#undef USE_LED_STRIP
#undef USE_TELEMETRY_FRSKY_HUB
#define USE_TELEMETRY_HOTT
#define USE_HOTT_TEXTMODE
#undef USE_TELEMETRY_SMARTPORT
#undef USE_TELEMETRY_MAVLINK
#undef USE_RESOURCE_MGMT
#define USE_CMS
#undef USE_TELEMETRY_CRSF
#undef USE_TELEMETRY_GHST
#undef USE_TELEMETRY_IBUS
#undef USE_TELEMETRY_JETIEXBUS
#undef USE_TELEMETRY_SRXL
#undef USE_SERIALRX_JETIEXBUS
#undef USE_VTX_COMMON
#undef USE_VTX_CONTROL
#undef USE_VTX_SMARTAUDIO
#undef USE_VTX_TRAMP
#undef USE_VTX_MSP
#undef USE_CAMERA_CONTROL
#undef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
#undef USE_SERIAL_4WAY_SK_BOOTLOADER

#define PLATFORM_NO_LIBC 0

#define USE_I2C
#define USE_I2C_DEVICE_1
#define USE_I2C_DEVICE_2
#define USE_I2C_DEVICE_3

#define BARO_I2C_INSTANCE I2CDEV_3
#define DEFAULT_BARO_DEVICE BARO_ICP10100

#define USE_SPI
#define USE_SPI_DEVICE_1

#define TARGET_FLASH_SIZE 8192

#define USE_SDCARD

#define DEFIO_NO_PORTS   // suppress 'no pins defined' warning

#define FLASH_PAGE_SIZE (0x400)

// belows are internal stuff

extern uint32_t SystemCoreClock;

typedef enum
{
    Mode_TEST = 0x0,
    Mode_Out_PP = 0x10
} GPIO_Mode;

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;
typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;
typedef enum {TEST_IRQ = 0 } IRQn_Type;
typedef enum {
    EXTI_Trigger_Rising = 0x08,
    EXTI_Trigger_Falling = 0x0C,
    EXTI_Trigger_Rising_Falling = 0x10
} EXTITrigger_TypeDef;

typedef struct
{
  uint32_t IDR;
  uint32_t ODR;
  uint32_t BSRR;
  uint32_t BRR;
} GPIO_TypeDef;

#define GPIOA_BASE ((intptr_t)0x0001)

typedef struct
{
    void* test;
} TIM_TypeDef;

typedef struct
{
    void* test;
} TIM_OCInitTypeDef;

typedef struct {
    void* test;
} DMA_TypeDef;

typedef struct {
    void* test;
} DMA_Channel_TypeDef;

typedef struct {
    void* test;
} DMA_InitTypeDef;

uint8_t DMA_GetFlagStatus(void *);
void DMA_Cmd(DMA_Channel_TypeDef*, FunctionalState );
void DMA_ClearFlag(uint32_t);

typedef struct
{
    int fd;
} SPI_TypeDef;

extern SPI_TypeDef hexagon_spi_bus;

#define SPI1 ((SPI_TypeDef *) &hexagon_spi_bus)

#define __set_BASEPRI(VALUE) ((void) VALUE)
#define __get_BASEPRI(VALUE) (0)
#define __set_BASEPRI_MAX(VALUE) ((void) VALUE)

typedef struct
{
    int fd;
	uint8_t port_number;
	uint32_t speed;
} USART_TypeDef;

#define NUM_HEXAGON_UART 6

extern USART_TypeDef hexagon_uart[NUM_HEXAGON_UART];

#define USART1 ((USART_TypeDef *) &hexagon_uart[0])
#define USART2 ((USART_TypeDef *) &hexagon_uart[1])
#define USART3 ((USART_TypeDef *) &hexagon_uart[2])
#define USART4 ((USART_TypeDef *) NULL)
#define USART5 ((USART_TypeDef *) NULL)
#define USART6 ((USART_TypeDef *) NULL)
#define USART7 ((USART_TypeDef *) NULL)
#define USART8 ((USART_TypeDef *) NULL)

#define UART4 ((USART_TypeDef *) &hexagon_uart[3])
#define UART5 ((USART_TypeDef *) &hexagon_uart[4])
#define UART7 ((USART_TypeDef *) &hexagon_uart[5])
#define UART8 ((USART_TypeDef *) NULL)

#define SERIALRX_UART SERIAL_PORT_USART3
#define MSP_UART SERIAL_PORT_USART1 // Virtual UART
#define ESC_SENSOR_UART SERIAL_PORT_UART4 // Virtual UART
#define MSP_DISPLAYPORT_UART SERIAL_PORT_UART5 // Virtual UART

#define SIMULATOR_MAX_RC_CHANNELS   16
#define SIMULATOR_MAX_PWM_CHANNELS  16

typedef struct
{
    void* test;
} I2C_TypeDef;

typedef struct
{
    void* test;
} ADC_TypeDef;

typedef struct {
    double timestamp;                   // in seconds
    double imu_angular_velocity_rpy[3]; // rad/s -> range: +/- 8192; +/- 2000 deg/se
    double imu_linear_acceleration_xyz[3];    // m/s/s NED, body frame -> sim 1G = 9.80665, FC 1G = 256
    double imu_orientation_quat[4];     //w, x, y, z
    double velocity_xyz[3];             // m/s, earth frame. ENU (Ve, Vn, Vup) for virtual GPS mode (USE_VIRTUAL_GPS)!
    double position_xyz[3];             // meters, NED from origin. Longitude, Latitude, Altitude (ENU) for virtual GPS mode (USE_VIRTUAL_GPS)!
    double pressure;
} fdm_packet;

uint64_t nanos64_real(void);
uint64_t micros64_real(void);
uint64_t millis64_real(void);
void delayMicroseconds(uint32_t us);
void delayMicroseconds_real(uint32_t us);
void delay(uint32_t ms);
uint64_t micros64(void);
uint64_t millis64(void);

int lockMainPID(void);

int targetParseArgs(int argc, char * argv[]);
