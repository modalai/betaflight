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

#define ICP10100_I2C_ADDR                      (0x63)
#define ICP10100_DEFAULT_CHIP_ID               (0x08)

#define ICP10100_CHIP_ID_REG                   (0xefc8)  /* Chip ID Register */




#define ICP10100_RST_REG                       (0xE0)  /* Softreset Register */
#define ICP10100_STAT_REG                      (0xF3)  /* Status Register */
#define ICP10100_CTRL_MEAS_REG                 (0xF4)  /* Ctrl Measure Register */
#define ICP10100_CONFIG_REG                    (0xF5)  /* Configuration Register */
#define ICP10100_PRESSURE_MSB_REG              (0xF7)  /* Pressure MSB Register */
#define ICP10100_PRESSURE_LSB_REG              (0xF8)  /* Pressure LSB Register */
#define ICP10100_PRESSURE_XLSB_REG             (0xF9)  /* Pressure XLSB Register */
#define ICP10100_TEMPERATURE_MSB_REG           (0xFA)  /* Temperature MSB Reg */
#define ICP10100_TEMPERATURE_LSB_REG           (0xFB)  /* Temperature LSB Reg */
#define ICP10100_TEMPERATURE_XLSB_REG          (0xFC)  /* Temperature XLSB Reg */
#define ICP10100_FORCED_MODE                   (0x01)

#define ICP10100_TEMPERATURE_CALIB_DIG_T1_LSB_REG             (0x88)
#define ICP10100_PRESSURE_TEMPERATURE_CALIB_DATA_LENGTH       (24)
#define ICP10100_DATA_FRAME_SIZE               (6)

#define ICP10100_OVERSAMP_SKIPPED          (0x00)
#define ICP10100_OVERSAMP_1X               (0x01)
#define ICP10100_OVERSAMP_2X               (0x02)
#define ICP10100_OVERSAMP_4X               (0x03)
#define ICP10100_OVERSAMP_8X               (0x04)
#define ICP10100_OVERSAMP_16X              (0x05)

// configure pressure and temperature oversampling, forced sampling mode
#define ICP10100_PRESSURE_OSR              (ICP10100_OVERSAMP_8X)
#define ICP10100_TEMPERATURE_OSR           (ICP10100_OVERSAMP_1X)
#define ICP10100_MODE                      (ICP10100_PRESSURE_OSR << 2 | ICP10100_TEMPERATURE_OSR << 5 | ICP10100_FORCED_MODE)

#define T_INIT_MAX                       (20)
// 20/16 = 1.25 ms
#define T_MEASURE_PER_OSRS_MAX           (37)
// 37/16 = 2.3125 ms
#define T_SETUP_PRESSURE_MAX             (10)
// 10/16 = 0.625 ms

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

static void icp10100BusInit(const extDevice_t *dev)
{
    UNUSED(dev);
}

static void icp10100BusDeinit(const extDevice_t *dev)
{
    UNUSED(dev);
}

#include "drivers/time.h"

bool icp10100Detect(baroDev_t *baro)
{
    delay(20);

	printf("In icp10100Detect");

	if (baro == NULL) {
		printf("baro is NULL");
		return false;
	}

    extDevice_t *dev = &baro->dev;

	if (dev == NULL) {
		printf("dev is NULL");
		return false;
	}

    bool defaultAddressApplied = false;

    icp10100BusInit(dev);

	if (dev->bus == NULL) {
		printf("dev->bus is NULL");
		return false;
	}

    if ((dev->bus->busType == BUS_TYPE_I2C) && (dev->busType_u.i2c.address == 0)) {
        // Default address for ICP10100
        dev->busType_u.i2c.address = ICP10100_I2C_ADDR;
        defaultAddressApplied = true;
    }

	printf("Reading chip id in icp10100Detect");

    busReadRegisterBuffer16(dev, ICP10100_CHIP_ID_REG, &icp10100_chip_id, 1);  /* read Chip Id */

	printf("Read 0x%0.4x chip id in icp10100Detect", icp10100_chip_id);

	uint8_t chipId = (icp10100_chip_id >> 8) & 0x3f;

	if (chipId != ICP10100_DEFAULT_CHIP_ID) {
        icp10100BusDeinit(dev);
        if (defaultAddressApplied) {
            dev->busType_u.i2c.address = 0;
        }
		printf("Didn't detect icp10100");
        return false;
    } else {
		printf("Detected icp10100!!!");
	}

    // busDeviceRegister(dev);
	// 
    // // read calibration
    // busReadRegisterBuffer(dev, ICP10100_TEMPERATURE_CALIB_DIG_T1_LSB_REG, (uint8_t *)&icp10100_cal, sizeof(icp10100_calib_param_t));
	// 
    // // set oversampling + power mode (forced), and start sampling
    // busWriteRegister(dev, ICP10100_CTRL_MEAS_REG, ICP10100_MODE);
	// 
    // // these are dummy as temperature is measured as part of pressure
    // baro->combined_read = true;
    // baro->ut_delay = 0;
    // baro->start_ut = icp10100StartUT;
    // baro->get_ut = icp10100GetUT;
    // baro->read_ut = icp10100ReadUT;
    // // only _up part is executed, and gets both temperature and pressure
    // baro->start_up = icp10100StartUP;
    // baro->get_up = icp10100GetUP;
    // baro->read_up = icp10100ReadUP;
    // baro->up_delay = ((T_INIT_MAX + T_MEASURE_PER_OSRS_MAX * (((1 << ICP10100_TEMPERATURE_OSR) >> 1) + ((1 << ICP10100_PRESSURE_OSR) >> 1)) + (ICP10100_PRESSURE_OSR ? T_SETUP_PRESSURE_MAX : 0) + 15) / 16) * 1000;
    // baro->calculate = icp10100Calculate;

    return true;
}

// static bool icp10100StartUT(baroDev_t *baro)
// {
//     UNUSED(baro);
//     // dummy
// 
//     return true;
// }
// 
// static bool icp10100ReadUT(baroDev_t *baro)
// {
//     UNUSED(baro);
//     // dummy
//     return true;
// }
// 
// static bool icp10100GetUT(baroDev_t *baro)
// {
//     UNUSED(baro);
//     // dummy
//     return true;
// }
// 
// static bool icp10100StartUP(baroDev_t *baro)
// {
//     // start measurement
//     // set oversampling + power mode (forced), and start sampling
//     return busWriteRegisterStart(&baro->dev, ICP10100_CTRL_MEAS_REG, ICP10100_MODE);
// }
// 
// static bool icp10100ReadUP(baroDev_t *baro)
// {
//     if (busBusy(&baro->dev, NULL)) {
//         return false;
//     }
// 
//     // read data from sensor
//     return busReadRegisterBufferStart(&baro->dev, ICP10100_PRESSURE_MSB_REG, sensor_data, ICP10100_DATA_FRAME_SIZE);
// }
// 
// static bool icp10100GetUP(baroDev_t *baro)
// {
//     if (busBusy(&baro->dev, NULL)) {
//         return false;
//     }
// 
//     icp10100_up = (int32_t)(sensor_data[0] << 12 | sensor_data[1] << 4 | sensor_data[2] >> 4);
//     icp10100_ut = (int32_t)(sensor_data[3] << 12 | sensor_data[4] << 4 | sensor_data[5] >> 4);
// 
//     return true;
// }
// 
// // Returns temperature in DegC, resolution is 0.01 DegC. Output value of "5123" equals 51.23 DegC
// // t_fine carries fine temperature as global value
// static int32_t icp10100CompensateTemperature(int32_t adc_T)
// {
//     int32_t var1, var2, T;
// 
//     var1 = ((((adc_T >> 3) - ((int32_t)icp10100_cal.dig_T1 << 1))) * ((int32_t)icp10100_cal.dig_T2)) >> 11;
//     var2  = (((((adc_T >> 4) - ((int32_t)icp10100_cal.dig_T1)) * ((adc_T >> 4) - ((int32_t)icp10100_cal.dig_T1))) >> 12) * ((int32_t)icp10100_cal.dig_T3)) >> 14;
//     t_fine = var1 + var2;
//     T = (t_fine * 5 + 128) >> 8;
// 
//     return T;
// }
// 
// // Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// // Output value of "24674867" represents 24674867/256 = 96386.2 Pa = 963.862 hPa
// static uint32_t icp10100CompensatePressure(int32_t adc_P)
// {
//     int64_t var1, var2, p;
//     var1 = ((int64_t)t_fine) - 128000;
//     var2 = var1 * var1 * (int64_t)icp10100_cal.dig_P6;
//     var2 = var2 + ((var1*(int64_t)icp10100_cal.dig_P5) << 17);
//     var2 = var2 + (((int64_t)icp10100_cal.dig_P4) << 35);
//     var1 = ((var1 * var1 * (int64_t)icp10100_cal.dig_P3) >> 8) + ((var1 * (int64_t)icp10100_cal.dig_P2) << 12);
//     var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)icp10100_cal.dig_P1) >> 33;
//     if (var1 == 0)
//         return 0;
//     p = 1048576 - adc_P;
//     p = (((p << 31) - var2) * 3125) / var1;
//     var1 = (((int64_t)icp10100_cal.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
//     var2 = (((int64_t)icp10100_cal.dig_P8) * p) >> 19;
//     p = ((p + var1 + var2) >> 8) + (((int64_t)icp10100_cal.dig_P7) << 4);
//     return (uint32_t)p;
// }
// 
// STATIC_UNIT_TESTED void icp10100Calculate(int32_t *pressure, int32_t *temperature)
// {
//     // calculate
//     int32_t t;
//     uint32_t p;
//     t = icp10100CompensateTemperature(icp10100_ut);
//     p = icp10100CompensatePressure(icp10100_up);
// 
//     if (pressure)
//         *pressure = (int32_t)(p / 256);
//     if (temperature)
//         *temperature = t;
// }

#endif
