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
#include <math.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "barometer.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/io.h"
#include "drivers/time.h"

#include "barometer_icp10100.h"

#if defined(USE_BARO) && defined(USE_BARO_ICP10100)

#define ICP10100_OTP_ADDR_LEN 3
#define ICP10100_OTP_SCAL_LEN 3
#define ICP10100_NUM_OTP_SCAL 4

#define ICP10100_I2C_ADDR                      (0x63)
#define ICP10100_DEFAULT_CHIP_ID               (0x08)

#define ICP10100_CHIP_ID_REG                   (0xefc8)  /* Chip ID Register */
#define ICP10100_SOFT_RESET_CMD                (0x805d)  /* Soft reset command */
#define ICP10100_SET_ADDR_CMD                  (0xc595)  /* Set address command */
#define ICP10100_READ_OTP_REG                  (0xc7f7)
#define ICP10100_LN_MEASURE_CMD                (0x70df)
#define ICP10100_ULN_MEASURE_CMD               (0x7866)


// #define ICP10100_STAT_REG                      (0xF3)  /* Status Register */
// #define ICP10100_CTRL_MEAS_REG                 (0xF4)  /* Ctrl Measure Register */
// #define ICP10100_CONFIG_REG                    (0xF5)  /* Configuration Register */
// #define ICP10100_PRESSURE_MSB_REG              (0xF7)  /* Pressure MSB Register */
// #define ICP10100_PRESSURE_LSB_REG              (0xF8)  /* Pressure LSB Register */
// #define ICP10100_PRESSURE_XLSB_REG             (0xF9)  /* Pressure XLSB Register */
// #define ICP10100_TEMPERATURE_MSB_REG           (0xFA)  /* Temperature MSB Reg */
// #define ICP10100_TEMPERATURE_LSB_REG           (0xFB)  /* Temperature LSB Reg */
// #define ICP10100_TEMPERATURE_XLSB_REG          (0xFC)  /* Temperature XLSB Reg */
// #define ICP10100_FORCED_MODE                   (0x01)
// 
// #define ICP10100_TEMPERATURE_CALIB_DIG_T1_LSB_REG             (0x88)
// #define ICP10100_PRESSURE_TEMPERATURE_CALIB_DATA_LENGTH       (24)
// #define ICP10100_DATA_FRAME_SIZE               (6)
// 
// #define ICP10100_OVERSAMP_SKIPPED          (0x00)
// #define ICP10100_OVERSAMP_1X               (0x01)
// #define ICP10100_OVERSAMP_2X               (0x02)
// #define ICP10100_OVERSAMP_4X               (0x03)
// #define ICP10100_OVERSAMP_8X               (0x04)
// #define ICP10100_OVERSAMP_16X              (0x05)
// 
// // configure pressure and temperature oversampling, forced sampling mode
// #define ICP10100_PRESSURE_OSR              (ICP10100_OVERSAMP_8X)
// #define ICP10100_TEMPERATURE_OSR           (ICP10100_OVERSAMP_1X)
// #define ICP10100_MODE                      (ICP10100_PRESSURE_OSR << 2 | ICP10100_TEMPERATURE_OSR << 5 | ICP10100_FORCED_MODE)
// 
// #define T_INIT_MAX                       (20)
// // 20/16 = 1.25 ms
// #define T_MEASURE_PER_OSRS_MAX           (37)
// // 37/16 = 2.3125 ms
// #define T_SETUP_PRESSURE_MAX             (10)
// // 10/16 = 0.625 ms

// typedef struct icp10100_calib_param_s {
//     uint16_t dig_T1; /* calibration T1 data */
//     int16_t dig_T2; /* calibration T2 data */
//     int16_t dig_T3; /* calibration T3 data */
//     uint16_t dig_P1; /* calibration P1 data */
//     int16_t dig_P2; /* calibration P2 data */
//     int16_t dig_P3; /* calibration P3 data */
//     int16_t dig_P4; /* calibration P4 data */
//     int16_t dig_P5; /* calibration P5 data */
//     int16_t dig_P6; /* calibration P6 data */
//     int16_t dig_P7; /* calibration P7 data */
//     int16_t dig_P8; /* calibration P8 data */
//     int16_t dig_P9; /* calibration P9 data */
// } __attribute__((packed)) icp10100_calib_param_t; // packed as we read directly from the device into this structure.

// STATIC_ASSERT(sizeof(icp10100_calib_param_t) == ICP10100_PRESSURE_TEMPERATURE_CALIB_DATA_LENGTH, icp10100_calibration_structure_incorrectly_packed);

// STATIC_UNIT_TESTED int32_t t_fine; /* calibration t_fine data */

static bool icp10100StartUT(baroDev_t *baro);
static bool icp10100ReadUT(baroDev_t *baro);
static bool icp10100GetUT(baroDev_t *baro);
static bool icp10100StartUP(baroDev_t *baro);
static bool icp10100ReadUP(baroDev_t *baro);
static bool icp10100GetUP(baroDev_t *baro);
static void icp10100Calculate(int32_t *pressure, int32_t *temperature);

static int16_t _scal[ICP10100_NUM_OTP_SCAL];
static uint8_t icp10100BarometerData[9];

static uint16_t icp10100_chip_id = 0;

// STATIC_UNIT_TESTED icp10100_calib_param_t icp10100_cal;
// uncompensated pressure and temperature
// int32_t icp10100_up = 0;
// int32_t icp10100_ut = 0;
// static DMA_DATA_ZERO_INIT uint8_t sensor_data[ICP10100_DATA_FRAME_SIZE];

// static bool icp10100StartUT(baroDev_t *baro);
// static bool icp10100ReadUT(baroDev_t *baro);
// static bool icp10100GetUT(baroDev_t *baro);
// static bool icp10100StartUP(baroDev_t *baro);
// static bool icp10100ReadUP(baroDev_t *baro);
// static bool icp10100GetUP(baroDev_t *baro);

// STATIC_UNIT_TESTED void icp10100Calculate(int32_t *pressure, int32_t *temperature);

static int8_t cal_crc(uint8_t seed, uint8_t data)
{
	int8_t poly = 0x31;
	int8_t var2;
	uint8_t i;

	for (i = 0; i < 8; i++) {
		if ((seed & 0x80) ^ (data & 0x80)) {
			var2 = 1;

		} else {
			var2 = 0;
		}

		seed = (seed & 0x7F) << 1;
		data = (data & 0x7F) << 1;
		seed = seed ^ (uint8_t)(poly * var2);
	}

	return (int8_t)seed;
}

bool icp10100Detect(baroDev_t *baro)
{
    extDevice_t *dev = &baro->dev;
    bool defaultAddressApplied = false;

    if ((dev->bus->busType == BUS_TYPE_I2C) && (dev->busType_u.i2c.address == 0)) {
        // Default address for ICP10100
        dev->busType_u.i2c.address = ICP10100_I2C_ADDR;
        defaultAddressApplied = true;
    }

    busReadRegisterBuffer16(dev, ICP10100_CHIP_ID_REG, (uint8_t*) &icp10100_chip_id, 2);  /* read Chip Id */

	uint8_t chipId = (icp10100_chip_id >> 8) & 0x3f;

	if (chipId != ICP10100_DEFAULT_CHIP_ID) {
        if (defaultAddressApplied) {
            dev->busType_u.i2c.address = 0;
        }
		printf("Didn't detect icp10100");
        return false;
    } else {
		printf("Detected icp10100!!!");
	}

    busDeviceRegister(dev);

	busWriteCommand16(dev, ICP10100_SOFT_RESET_CMD);

	delay(100);

    busReadRegisterBuffer16(dev, ICP10100_CHIP_ID_REG, (uint8_t*) &icp10100_chip_id, 2);  /* read Chip Id */

	chipId = (icp10100_chip_id >> 8) & 0x3f;

	if (chipId != ICP10100_DEFAULT_CHIP_ID) {
        if (defaultAddressApplied) {
            dev->busType_u.i2c.address = 0;
        }
		printf("Didn't detect icp10100 after reset");
        return false;
    } else {
		printf("Detected icp10100 after reset!!!");
	}
	
    // read OTP
	uint8_t addrOTPCmd[ICP10100_OTP_ADDR_LEN] = {0x00, 0x66, 0x9c};
	uint8_t otpBuf[ICP10100_OTP_SCAL_LEN];
	uint8_t crc;
	// bool success = true;

    busWriteRegisterBuffer16(dev, ICP10100_SET_ADDR_CMD, addrOTPCmd, ICP10100_OTP_ADDR_LEN);
	
	for (uint8_t i = 0; i < ICP10100_NUM_OTP_SCAL; i++) {
    	busReadRegisterBuffer16(dev, ICP10100_READ_OTP_REG, otpBuf, ICP10100_OTP_SCAL_LEN);

		crc = 0xFF;

		for (int j = 0; j < ICP10100_OTP_SCAL_LEN - 1; j++) {
			crc = (uint8_t) cal_crc(crc, otpBuf[j]);
		}

		if (crc != otpBuf[2]) {
			printf("ERROR, CRC failed on OTP scale factor");
			// TODO
		} else {
			printf("CRC passed on OTP scale factor %d", i);
		}

		_scal[i] = (otpBuf[0] << 8) | otpBuf[1];
	}

    // Start sampling in ultra-low noise (ULN) mode
    busWriteCommand16(dev, ICP10100_ULN_MEASURE_CMD);
    // busWriteCommand16(dev, ICP10100_LN_MEASURE_CMD);
	
    // these are dummy as temperature is measured as part of pressure
    baro->combined_read = true;
    baro->ut_delay = 0;
    baro->start_ut = icp10100StartUT;
    baro->get_ut = icp10100GetUT;
    baro->read_ut = icp10100ReadUT;
    // only _up part is executed, and gets both temperature and pressure
    baro->start_up = icp10100StartUP;
    baro->get_up = icp10100GetUP;
    baro->read_up = icp10100ReadUP;
    baro->up_delay = 96 * 1000;  // 95ms ultra low noise (but 65ms is max)
    // baro->up_delay = 26 * 1000;  // 26ms low noise
    baro->calculate = icp10100Calculate;

    return true;
}

static bool icp10100StartUT(baroDev_t *baro)
{
    UNUSED(baro);
    // dummy

	printf("In icp10100StartUT");

    return true;
}

static bool icp10100ReadUT(baroDev_t *baro)
{
    UNUSED(baro);
    // dummy
	printf("In icp10100ReadUT");

    return true;
}

static bool icp10100GetUT(baroDev_t *baro)
{
    UNUSED(baro);
    // dummy
	printf("In icp10100GetUT");

    return true;
}

static bool icp10100StartUP(baroDev_t *baro)
{
    // start measurement
    // Start sampling in ultra-low noise (ULN) mode
    busWriteCommand16(&baro->dev, ICP10100_ULN_MEASURE_CMD);
    // busWriteCommand16(&baro->dev, ICP10100_LN_MEASURE_CMD);

	// printf("In icp10100StartUP");

	return true;
}

static bool icp10100ReadUP(baroDev_t *baro)
{
    if (busBusy(&baro->dev, NULL)) {
        return false;
    }

    // read data from sensor
	// TODO: This should just be the start call? See bmp388 driver
    return busRawReadBuffer(&baro->dev, icp10100BarometerData, 9);
}

static bool icp10100GetUP(baroDev_t *baro)
{
	(void) baro;
    return true;
	// TODO: This should wait for busBusy to be false then copy over the data?
}

static void icp10100Calculate(int32_t *pressure, int32_t *temperature)
{
	// printf("In icp10100Calculate");

	uint16_t _raw_t = (icp10100BarometerData[0] << 8) | icp10100BarometerData[1];
	uint32_t L_res_buf3 = icp10100BarometerData[3];	// expand result bytes to 32bit to fix issues on 8-bit MCUs
	uint32_t L_res_buf4 = icp10100BarometerData[4];
	uint32_t L_res_buf6 = icp10100BarometerData[6];
	uint32_t _raw_p = (L_res_buf3 << 16) | (L_res_buf4 << 8) | L_res_buf6;

	// constants for presure calculation
	static float _pcal[3] = { 45000.0, 80000.0, 105000.0 };
	static float _lut_lower = 3.5 * 0x100000;	// 1<<20
	static float _lut_upper = 11.5 * 0x100000;	// 1<<20
	static float _quadr_factor = 1 / 16777216.0;
	static float _offst_factor = 2048.0;

	// calculate temperature
	float _temperature_C = -45.f + 175.f / 65536.f * _raw_t;

	// calculate pressure
	float t = (float)(_raw_t - 32768);
	float s1 = _lut_lower + (float)(_scal[0] * t * t) * _quadr_factor;
	float s2 = _offst_factor * _scal[3] + (float)(_scal[1] * t * t) * _quadr_factor;
	float s3 = _lut_upper + (float)(_scal[2] * t * t) * _quadr_factor;
	float c = (s1 * s2 * (_pcal[0] - _pcal[1]) +
		   s2 * s3 * (_pcal[1] - _pcal[2]) +
		   s3 * s1 * (_pcal[2] - _pcal[0])) /
		  (s3 * (_pcal[0] - _pcal[1]) +
		   s1 * (_pcal[1] - _pcal[2]) +
		   s2 * (_pcal[2] - _pcal[0]));
	float a = (_pcal[0] * s1 - _pcal[1] * s2 - (_pcal[1] - _pcal[0]) * c) / (s1 - s2);
	float b = (_pcal[0] - a) * (s1 + c);
	float _pressure_Pa = a + b / (c + _raw_p);

	*pressure = (int32_t) round((double)(_pressure_Pa * 256.0f));
	*temperature = (int32_t) (_temperature_C * 100.0f);

	// printf("Pressure: %f Pa, temp: %f C", (double) *pressure / 256.0, (double) *temperature / 100.0);
}

#endif
