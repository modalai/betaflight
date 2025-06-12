

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "drivers/motor_impl.h"
#include "pg/motor.h"
#include "sl_client.h"
#include "drivers/time.h"
#include "scheduler/scheduler.h"
#include "fc/tasks.h"

#define HEXAGON_MAX_MOTORS 4

// #define ESC_BAUDRATE 921600
#define ESC_BAUDRATE 2000000

// ESC specific definitions
#define ESC_PACKET_TYPE_PWM_CMD 1
#define ESC_PACKET_TYPE_FB_RESPONSE 128
#define ESC_PACKET_TYPE_FB_POWER_STATUS 132

// IO board specific definitions
#define IO_PACKET_TYPE_PWM_HIRES_CMD 6

// Generic definitions
#define PKT_HEADER 0xAF
#define PACKET_TYPE_VERSION_EXT_REQUEST 24
#define PACKET_TYPE_VERSION_EXT_RESPONSE 131

static bool motorEnabled[HEXAGON_MAX_MOTORS];
static float motorSpeed[HEXAGON_MAX_MOTORS];
static unsigned motorMap[HEXAGON_MAX_MOTORS];
static bool reverseMotors;
static int motor_fd = -1;
static serialReceiveCallbackPtr motorTelemCB;
static uint8_t last_fb_idx;
static uint32_t last_fb_req_ms;

struct __attribute__((packed)) extended_version_info {
    uint8_t  id;
    uint16_t sw_version;
    uint16_t hw_version;
    uint8_t  unique_id[12];
    char     firmware_git_version[12];
    char     bootloader_git_version[12];
    uint16_t bootloader_version;
    uint16_t crc;
};

struct __attribute__((packed)) esc_response_v2 {
    uint8_t  id_state;     // bits 0:3 = state, bits 4:7 = ID

    uint16_t rpm;          // Current RPM of the motor
    uint8_t  cmd_counter;  // Number of commands received by the ESC
    uint8_t  power;        // Applied power [0..100]

    uint16_t voltage;      // Voltage measured by the ESC in mV
    int16_t  current;      // Current measured by the ESC in 8mA resolution
    int16_t  temperature;  // Temperature measured by the ESC in 0.01 degC resolution
};

struct __attribute__((packed)) esc_power_status {
    uint8_t  id;       //ESC Id (could be used as system ID elsewhere)
    uint16_t voltage;  //Input voltage (Millivolts)
    int16_t  current;  //Total Current (8mA resolution)
};

struct motor_status {
	bool fb_active;
	struct __attribute__((packed)) esc_response_v2 fb;
} motorFeedback[HEXAGON_MAX_MOTORS];

struct power_status {
	bool ps_active;
	struct __attribute__((packed)) esc_power_status ps;
} powerFeedback;

static uint32_t motorDebug;

void registerTelemCallback(serialReceiveCallbackPtr cb) {
	motorTelemCB = cb;
}

// Calculate Modbus CRC16 for array of bytes
uint16_t calc_crc_modbus(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t pos = 0; pos < len; pos++) {
        crc ^= (uint16_t) buf[pos]; // XOR byte into least sig. byte of crc
        for (uint8_t i = 8; i != 0; i--) { // Loop over each bit
            if ((crc & 0x0001) != 0) { // If the LSB is set
                crc >>= 1; // Shift right and XOR 0xA001
                crc ^= 0xA001;
            } else {
                // Else LSB is not set
                crc >>= 1; // Just shift right
            }
        }
    }
    return crc;
}

// Send a packet with CRC to the ESC or IO board
static void send_packet(uint8_t type, uint8_t *data, uint16_t size)
{
    uint16_t packet_size = size + 5;
    uint8_t out[packet_size];

    out[0] = PKT_HEADER;
    out[1] = packet_size;
    out[2] = type;

    memcpy(&out[3], data, size);

    uint16_t crc = calc_crc_modbus(&out[1], packet_size - 3);

    memcpy(&out[packet_size - 2], &crc, sizeof(uint16_t));

    sl_client_uart_write(motor_fd, (const char *)out, packet_size);
}

// Convert 1000 to 2000 PWM to 0 to 800 for ModalAI ESCs
static int16_t pwm_to_esc(uint16_t pwm)
{
    float p = (pwm-1000)*0.001;
	if (p < 0.0f) p = 0.0f;
	else if (p > 1.0f) p = 1.0f;

    return (int16_t)(800.0f * p);
}

// check for responses
static void check_response(void)
{
    uint8_t buf[256];
    struct __attribute__((packed)) data_packet {
        uint8_t header;
        uint8_t length;
        uint8_t type;
        union {
            struct extended_version_info ver_info;
            struct esc_response_v2 resp_v2;
            struct esc_power_status power_status;
        } u;
    };
    int n = sl_client_uart_read(motor_fd, (char *)buf, sizeof(buf));
    // TODO: Maintain a count of total received bytes over time
    // printf("Motor response bytes: %d", n);
    while (n >= 3) {
        const struct data_packet *pkt = (struct data_packet *)buf;
        if (pkt->header != PKT_HEADER || pkt->length > n) {
            return;
        }
        const uint16_t crc = calc_crc_modbus(&pkt->length, pkt->length-3);
        const uint16_t crc2 = buf[pkt->length-2] | buf[pkt->length-1]<<8;
        if (crc != crc2) {
            // TODO: Maintain a count of failed CRCs over time
            printf("Motor CRC fail on input");
            return;
        }
        switch (pkt->type) {
        case PACKET_TYPE_VERSION_EXT_RESPONSE:
            printf("Motor version response");
            // handle_version_feedback(pkt->u.ver_info);
            break;
        case ESC_PACKET_TYPE_FB_RESPONSE:
		{
			uint8_t motor_id = pkt->u.resp_v2.id_state >> 4;
			if (motor_id < HEXAGON_MAX_MOTORS) {
				motorFeedback[motor_id].fb_active = true;
				motorFeedback[motor_id].fb = pkt->u.resp_v2;
            	// printf("Motor feedback response. id: %u, volts: %u", motor_id, motorFeedback[motor_id].fb.voltage);
			} else {
            	printf("Bad id in motor feedback response: %u", motor_id);
			}
            break;
		}
        case ESC_PACKET_TYPE_FB_POWER_STATUS:
            // printf("Motor power status");
			powerFeedback.ps_active = true;
			powerFeedback.ps = pkt->u.power_status;
        	// printf("Motor power status. volts: %u, current: %d", powerFeedback.ps.voltage, powerFeedback.ps.current);
            break;
        default:
            printf("Unknown pkt %u", pkt->type);
            break;
        }
        if (n == pkt->length) {
            break;
        }
        memmove(&buf[0], &buf[pkt->length], n - pkt->length);
        n -= pkt->length;
    }
}

static void send_esc_command(void)
{
    int16_t data[5];

    // We don't set any LEDs
    data[4] = 0;

    for (uint8_t i = 0; i < HEXAGON_MAX_MOTORS; i++) {

        data[i] = pwm_to_esc(motorSpeed[motorMap[i]]);

		if (reverseMotors) data[i] *= -1;

        // Make sure feedback request bit is cleared for all ESCs
        data[i] &= 0xFFFE;
    }

	// TODO: If not armed zero out all PWM data
	//       But also need to be able to spin motors from configurator!
	// data[0] = data[1] = data[2] = data[3] = 0;

    const uint32_t now_ms = millis();
    if (now_ms - last_fb_req_ms > 5) {
        last_fb_req_ms = now_ms;
        // request feedback from one ESC
        last_fb_idx = (last_fb_idx+1) % 4;
        data[last_fb_idx] |= 1;
    }

    send_packet(ESC_PACKET_TYPE_PWM_CMD, (uint8_t *)data, sizeof(data));

	if (data[last_fb_idx] & 1) check_response();
}

bool hexagonMotorEnabled(unsigned index) {
	// printf("In hexagonMotorEnabled, index: %u", index);
	if (index < HEXAGON_MAX_MOTORS)	return motorEnabled[index];
	return false;
}

bool hexagonMotorEnable(void) {
	printf("In hexagonMotorEnable");
	for (int i = 0; i < HEXAGON_MAX_MOTORS; i++) {
		motorEnabled[i] = true;
	}
	return true;
}

void hexagonMotorDisable(void) {
	printf("In hexagonMotorDisable");
	for (int i = 0; i < HEXAGON_MAX_MOTORS; i++) {
		motorEnabled[i] = false;
	}
}

void hexagonMotorWrite(uint8_t index, float value) {
	if (index < HEXAGON_MAX_MOTORS)	{
		if (motorEnabled[index]) {
			motorSpeed[index] = value;
		}
	}
}

void hexagonMotorUpdateComplete(void) {
	motorDebug++;

	send_esc_command();

	// if (motorDebug == 10000) {
	if (motorDebug == 2000) {
		printf("Gyro actual %lu, desired %lu, PID %lu",
				getTaskDeltaTimeUs(TASK_GYRO), getTask(TASK_GYRO)->attribute->desiredPeriodUs,
				getTaskDeltaTimeUs(TASK_PID));
		printf("Motors: %u, %u, %u, %u", (uint16_t) motorSpeed[0], (uint16_t) motorSpeed[1],
				(uint16_t) motorSpeed[2], (uint16_t) motorSpeed[3]);
		motorDebug = 0;
	}
}

float hexagonConvertExternalToMotor(uint16_t externalValue)
{
    return (float)externalValue;
}

uint16_t hexagonConvertMotorToExternal(float motorValue)
{
    return (uint16_t)motorValue;
}

float erpmToRpm(uint32_t erpm)
{
    return (float) erpm;
}

static uint8_t updateCrc8(uint8_t crc, uint8_t crc_seed)
{
    uint8_t crc_u = crc;
    crc_u ^= crc_seed;

    for (int i=0; i<8; i++) {
        crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 );
    }

    return (crc_u);
}

static uint8_t calculateCrc8(const uint8_t *Buf, const uint8_t BufLen)
{
    uint8_t crc = 0;
    for (int i = 0; i < BufLen; i++) {
        crc = updateCrc8(Buf[i], crc);
    }

    return crc;
}

void hexagonRequestTelemetry(unsigned index) {
	// printf("Got telemetry request for motor %u", index);

	if (motorTelemCB && index < HEXAGON_MAX_MOTORS) {
		uint8_t response[10];

		if (motorFeedback[index].fb_active) {

            // Temperature in deg C
			response[0] = motorFeedback[index].fb.temperature / 100;

			if (!powerFeedback.ps_active) {
        		// Voltage 0.01V
				uint16_t v = motorFeedback[index].fb.voltage / 10;
				response[1] = v >> 8;
				response[2] = v & 0xFF;
				// printf("Sending ESC voltage: %u %u %u", v, response[1], response[2]);
                // Current 0.01A
				int16_t c = (motorFeedback[index].fb.current * 8) / 10;
				response[3] = c >> 8;
				response[4] = c & 0xFF;
			} else {
				uint16_t v = powerFeedback.ps.voltage / 10;
        		// Voltage 0.01V
				response[1] = v >> 8;
				response[2] = v & 0xFF;
				// printf("Sending ESC ps voltage: %u %u %u", v, response[1], response[2]);
                // Current 0.01A
				int16_t c = ((motorFeedback[index].fb.current * 8) / 10) / 4;
				response[3] = c >> 8;
				response[4] = c & 0xFF;
			}

        	// mAh ???
			response[5] = 0x00;
			response[6] = 0x00;

        	// 0.01erpm
			response[7] = motorFeedback[index].fb.rpm >> 8;
			response[8] = motorFeedback[index].fb.rpm & 0xFF;

			response[9] = calculateCrc8(response, 10 - 1); // CRC

			for (int i = 0; i < 10; i++) {
				motorTelemCB(response[i], NULL);
			}
		}
	}
}

void hexagonMotorShutdown() {
	return;
}

static const motorVTable_t vTable = {
    .postInit = NULL,
    .convertExternalToMotor = hexagonConvertExternalToMotor,
    .convertMotorToExternal = hexagonConvertMotorToExternal,
    .enable = hexagonMotorEnable,
    .disable = hexagonMotorDisable,
    .isMotorEnabled = hexagonMotorEnabled,
    .decodeTelemetry = NULL,
    .write = hexagonMotorWrite,
    .writeInt = NULL,
    .updateComplete = hexagonMotorUpdateComplete,
    .shutdown = hexagonMotorShutdown,
    .requestTelemetry = hexagonRequestTelemetry,
    .isMotorIdle = NULL,
    .getMotorIO = NULL,
};

bool motorPwmDevInit(motorDevice_t *device, const motorDevConfig_t *motorConfig, uint16_t _idlePulse)
{
	(void) _idlePulse;

	if (!device) {
		return false;
	}

	device->vTable = &vTable;

	motor_fd = sl_client_config_uart(2, ESC_BAUDRATE);

	if (motor_fd == -1) {
		printf("ERROR: Failed to open ESC serial port");
		return false;
	}

	for (int i = 0; i < HEXAGON_MAX_MOTORS; i++) {
		motorMap[i] = motorConfig->motorOutputReordering[i];
	}

	// Can only change spin direction for all motors, not each motor
	// BTW: This parameter isn't supposed to control spin direction but
	// we use it for that on VOXL
	if (motorConfig->motorInversion) reverseMotors = true;

	(void) sl_client_disable_uart_tx_wait(motor_fd);

	// TODO: Handshake with ESC, check for IO board

	printf("Initialized motor count %d", device->count);

	return true;
}