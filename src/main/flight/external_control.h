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

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/axis.h"
#include "common/time.h"

#include "flight/imu.h"

#define EXTERNAL_CONTROL_DEFAULT_TIMEOUT_US 200000

typedef enum {
    EXTERNAL_CONTROL_REJECT_NONE = 0,
    EXTERNAL_CONTROL_REJECT_TARGET,
    EXTERNAL_CONTROL_REJECT_TYPE_MASK,
    EXTERNAL_CONTROL_REJECT_ATTITUDE_NOT_FINITE,
    EXTERNAL_CONTROL_REJECT_ATTITUDE_NORM,
    EXTERNAL_CONTROL_REJECT_THRUST_NOT_FINITE,
    EXTERNAL_CONTROL_REJECT_THRUST_RANGE,
    EXTERNAL_CONTROL_REJECT_YAW_RATE_NOT_FINITE,
} externalControlRejectReason_e;

// Transport-neutral input. The quaternion uses Betaflight's body-FLU to
// earth-NWU convention. Transport adapters must convert into this convention.
typedef struct externalControlSetpointInput_s {
    quaternion_t attitude;
    float thrust;
    float yawRate;
    uint32_t sourceTimestampMs;
    bool hasYawRate;
} externalControlSetpointInput_t;

typedef struct externalControlSetpoint_s {
    quaternion_t attitude;
    float thrust;
    float yawRate;
    uint32_t sourceTimestampMs;
    timeUs_t receivedAtUs;
    bool hasYawRate;
} externalControlSetpoint_t;

typedef struct externalControlDiagnostics_s {
    uint32_t acceptedCount;
    uint32_t rejectedCount;
    timeDelta_t lastReceiveIntervalUs;
    uint32_t lastRoundTripTimeUs;
    float lastQuaternionNormError;
    externalControlRejectReason_e lastRejectReason;
    bool roundTripTimeValid;
} externalControlDiagnostics_t;

// Output of the passive attitude-controller calculation. Rates use the same
// body-FLU, degrees/second convention consumed by Betaflight's rate PID. This
// structure is diagnostic-only until a later, explicitly gated integration
// connects it to the PID and mixer paths.
typedef struct externalControlShadowCommand_s {
    float rate[XYZ_AXIS_COUNT];
    float attitudeError[XYZ_AXIS_COUNT]; // shortest-path body error vector, radians
    float thrust;
    timeUs_t computedAtUs;
    uint8_t saturationMask;
    bool valid;
} externalControlShadowCommand_t;

bool externalControlPublishSetpoint(const externalControlSetpointInput_t *input, timeUs_t receivedAtUs);
void externalControlRecordRejection(externalControlRejectReason_e reason);
bool externalControlGetSetpoint(externalControlSetpoint_t *setpoint);
bool externalControlGetDiagnostics(externalControlDiagnostics_t *diagnostics);
bool externalControlIsFresh(timeUs_t currentTimeUs, timeDelta_t timeoutUs);
void externalControlUpdateShadowCommand(timeUs_t currentTimeUs);
bool externalControlGetShadowCommand(externalControlShadowCommand_t *command);
void externalControlUpdateDebug(timeUs_t currentTimeUs);
