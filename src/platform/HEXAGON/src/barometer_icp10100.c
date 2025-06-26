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

#include "drivers/barometer/barometer.h"

#include "drivers/bus_i2c.h"
#include "drivers/io.h"
#include "drivers/time.h"

#include "barometer_icp10100.h"

extern bool i2cWriteCommand16(I2CDevice device, uint8_t addr_, uint16_t cmd_);
extern bool i2cWriteBuffer16(I2CDevice device, uint8_t addr_, uint16_t reg_, uint8_t len_, uint8_t *data);
extern bool i2cReadBufferNoRegister(I2CDevice device, uint8_t addr_, uint8_t len, uint8_t* buf);
extern bool i2cRead16(I2CDevice device, uint8_t addr_, uint16_t reg, uint8_t len, uint8_t* buf);

extern bool i2cRead16(I2CDevice device, uint8_t addr_, uint16_t reg, uint8_t len, uint8_t* buf);
extern bool i2cWriteCommand16(I2CDevice device, uint8_t addr_, uint16_t cmd_);
extern bool i2cWriteBuffer16(I2CDevice device, uint8_t addr_, uint16_t reg_, uint8_t len_, uint8_t *data);
extern bool i2cReadBufferNoRegister(I2CDevice device, uint8_t addr_, uint8_t len, uint8_t* buf);

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

static bool icp10100StartUT(baroDev_t *baro);
static bool icp10100ReadUT(baroDev_t *baro);
static bool icp10100GetUT(baroDev_t *baro);
static bool icp10100StartUP(baroDev_t *baro);
static bool icp10100ReadUP(baroDev_t *baro);
static bool icp10100GetUP(baroDev_t *baro);
static void icp10100Calculate(int32_t *pressure, int32_t *temperature);

static int16_t _scal[ICP10100_NUM_OTP_SCAL];
static uint8_t icp10100BarometerData[9];

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

static bool validateChipId(extDevice_t *dev)
{
	uint16_t readChipId = 0;

    // busReadRegisterBuffer16(dev, ICP10100_CHIP_ID_REG, (uint8_t*) &readChipId, 2);  /* read Chip Id */
    i2cRead16(dev->bus->busType_u.i2c.device, dev->busType_u.i2c.address, ICP10100_CHIP_ID_REG, 2, (uint8_t*) &readChipId);  /* read Chip Id */

	uint8_t chipId = (readChipId >> 8) & 0x3f;
	if (chipId != ICP10100_DEFAULT_CHIP_ID) {
		printf("Didn't detect icp10100");
        return false;
    }

	printf("Detected icp10100!!!");
	return true;
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

	if (!validateChipId(dev)) {
        if (defaultAddressApplied) {
            dev->busType_u.i2c.address = 0;
        }
		return false;
	}

    busDeviceRegister(dev);

	// busWriteCommand16(dev, ICP10100_SOFT_RESET_CMD);
	i2cWriteCommand16(dev->bus->busType_u.i2c.device, dev->busType_u.i2c.address, ICP10100_SOFT_RESET_CMD);

	delay(100);

	if (!validateChipId(dev)) {
        if (defaultAddressApplied) {
            dev->busType_u.i2c.address = 0;
        }
		return false;
	}
	
    // read OTP
	uint8_t addrOTPCmd[ICP10100_OTP_ADDR_LEN] = {0x00, 0x66, 0x9c};
	uint8_t otpBuf[ICP10100_OTP_SCAL_LEN];
	uint8_t crc;

    // busWriteRegisterBuffer16(dev, ICP10100_SET_ADDR_CMD, addrOTPCmd, ICP10100_OTP_ADDR_LEN);
	i2cWriteBuffer16(dev->bus->busType_u.i2c.device, dev->busType_u.i2c.address, ICP10100_SET_ADDR_CMD, ICP10100_OTP_ADDR_LEN, addrOTPCmd);

	for (uint8_t i = 0; i < ICP10100_NUM_OTP_SCAL; i++) {
    	// busReadRegisterBuffer16(dev, ICP10100_READ_OTP_REG, otpBuf, ICP10100_OTP_SCAL_LEN);
		i2cRead16(dev->bus->busType_u.i2c.device, dev->busType_u.i2c.address, ICP10100_READ_OTP_REG, ICP10100_OTP_SCAL_LEN, otpBuf);

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
    // busWriteCommand16(dev, ICP10100_ULN_MEASURE_CMD);
	i2cWriteCommand16(dev->bus->busType_u.i2c.device, dev->busType_u.i2c.address, ICP10100_ULN_MEASURE_CMD);
	
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
    baro->up_delay = 96 * 1000;  // 96ms for ultra low noise measurement
    baro->calculate = icp10100Calculate;

    return true;
}

static bool icp10100StartUT(baroDev_t *baro)
{
    UNUSED(baro);
    // dummy

    return true;
}

static bool icp10100ReadUT(baroDev_t *baro)
{
    UNUSED(baro);
    // dummy

    return true;
}

static bool icp10100GetUT(baroDev_t *baro)
{
    UNUSED(baro);
    // dummy

    return true;
}

static bool icp10100StartUP(baroDev_t *baro)
{
    // start measurement
    // Start sampling in ultra-low noise (ULN) mode
    // busWriteCommand16(&baro->dev, ICP10100_ULN_MEASURE_CMD);
	i2cWriteCommand16(baro->dev.bus->busType_u.i2c.device, baro->dev.busType_u.i2c.address, ICP10100_ULN_MEASURE_CMD);

	return true;
}

static bool icp10100ReadUP(baroDev_t *baro)
{
    // if (busBusy(&baro->dev, NULL)) {
    if (i2cBusy(baro->dev.bus->busType_u.i2c.device, NULL)) {
        return false;
    }

    // read data from sensor
    // return busReadBufferStart(&baro->dev, icp10100BarometerData, 9);
	return i2cReadBufferNoRegister(baro->dev.bus->busType_u.i2c.device, baro->dev.busType_u.i2c.address, 9, icp10100BarometerData);
}

static bool icp10100GetUP(baroDev_t *baro)
{
	// Wait until we know the transaction has been completed
    // if (busBusy(&baro->dev, NULL)) {
    if (i2cBusy(baro->dev.bus->busType_u.i2c.device, NULL)) {
        return false;
    }

    return true;
}

static void icp10100Calculate(int32_t *pressure, int32_t *temperature)
{
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
