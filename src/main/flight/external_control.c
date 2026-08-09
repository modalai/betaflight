/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "platform.h"

#ifdef USE_EXTERNAL_CONTROL

#include <limits.h>
#include <math.h>

#include "build/atomic.h"
#include "build/debug.h"

#include "common/maths.h"

#include "drivers/nvic.h"
#include "drivers/time.h"

#include "flight/external_control.h"

#define EXTERNAL_CONTROL_QUATERNION_NORM_SQUARED_MIN 0.5f
#define EXTERNAL_CONTROL_QUATERNION_NORM_SQUARED_MAX 1.5f
#define EXTERNAL_CONTROL_MAX_RTT_MS 10000U

// Provisional controller values match PX4's multicopter defaults. They remain
// compile-time constants until bench testing establishes the parameter ranges
// needed for a dedicated external-control configuration group.
static const float externalControlAttitudeGain[XYZ_AXIS_COUNT] = { 4.0f, 4.0f, 2.8f };
static const float externalControlRateLimit[XYZ_AXIS_COUNT] = { 220.0f, 220.0f, 200.0f };

enum {
    EXTERNAL_CONTROL_STATE_VALID = 1 << 0,
    EXTERNAL_CONTROL_STATE_FRESH = 1 << 1,
    EXTERNAL_CONTROL_STATE_HAS_YAW_RATE = 1 << 2,
    EXTERNAL_CONTROL_STATE_RTT_VALID = 1 << 3,
    EXTERNAL_CONTROL_STATE_MODE_REQUESTED = 1 << 7,
    EXTERNAL_CONTROL_STATE_MODE_ACTIVE = 1 << 8,
    EXTERNAL_CONTROL_STATE_ANGLE_FALLBACK = 1 << 9,
};

static externalControlSetpoint_t setpoint;
static externalControlDiagnostics_t diagnostics;
static externalControlShadowCommand_t shadowCommand;
static bool setpointValid;
static bool modeRequested;
static externalControlModeState_e modeState;

static int16_t externalControlDebugValue(float value)
{
    if (value >= INT16_MAX) {
        return INT16_MAX;
    }
    if (value <= INT16_MIN) {
        return INT16_MIN;
    }
    return lrintf(value);
}

static int16_t externalControlStateFlags(bool valid, bool fresh,
    const externalControlSetpoint_t *currentSetpoint,
    const externalControlDiagnostics_t *currentDiagnostics)
{
    int16_t state = valid ? EXTERNAL_CONTROL_STATE_VALID : 0;
    state |= fresh ? EXTERNAL_CONTROL_STATE_FRESH : 0;
    state |= valid && currentSetpoint->hasYawRate ? EXTERNAL_CONTROL_STATE_HAS_YAW_RATE : 0;
    state |= currentDiagnostics->roundTripTimeValid ? EXTERNAL_CONTROL_STATE_RTT_VALID : 0;
    return state;
}

// Return twice the vector part of current^-1 * desired, selecting the
// quaternion sign that gives the shortest rotation. For small errors this is
// the body-frame rotation vector in radians.
static void externalControlCalculateAttitudeError(const quaternion_t *current,
    const quaternion_t *desired, float error[XYZ_AXIS_COUNT])
{
    const float errorW = current->w * desired->w + current->x * desired->x +
        current->y * desired->y + current->z * desired->z;
    const float errorX = current->w * desired->x - current->x * desired->w -
        current->y * desired->z + current->z * desired->y;
    const float errorY = current->w * desired->y + current->x * desired->z -
        current->y * desired->w - current->z * desired->x;
    const float errorZ = current->w * desired->z - current->x * desired->y +
        current->y * desired->x - current->z * desired->w;
    const float errorSign = errorW < 0.0f ? -2.0f : 2.0f;

    error[FD_ROLL] = errorSign * errorX;
    error[FD_PITCH] = errorSign * errorY;
    error[FD_YAW] = errorSign * errorZ;
}

void externalControlRecordRejection(externalControlRejectReason_e reason)
{
    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        diagnostics.rejectedCount++;
        diagnostics.lastRejectReason = reason;
    }
}

bool externalControlPublishSetpoint(const externalControlSetpointInput_t *input, timeUs_t receivedAtUs)
{
    if (!isfinite(input->attitude.w) || !isfinite(input->attitude.x) ||
        !isfinite(input->attitude.y) || !isfinite(input->attitude.z)) {
        externalControlRecordRejection(EXTERNAL_CONTROL_REJECT_ATTITUDE_NOT_FINITE);
        return false;
    }

    const float normSquared = sq(input->attitude.w) + sq(input->attitude.x) +
        sq(input->attitude.y) + sq(input->attitude.z);
    const float normError = fabsf(sqrtf(normSquared) - 1.0f);

    if (normSquared < EXTERNAL_CONTROL_QUATERNION_NORM_SQUARED_MIN ||
        normSquared > EXTERNAL_CONTROL_QUATERNION_NORM_SQUARED_MAX) {
        ATOMIC_BLOCK(NVIC_PRIO_MAX) {
            diagnostics.lastQuaternionNormError = normError;
        }
        externalControlRecordRejection(EXTERNAL_CONTROL_REJECT_ATTITUDE_NORM);
        return false;
    }

    if (!isfinite(input->thrust)) {
        externalControlRecordRejection(EXTERNAL_CONTROL_REJECT_THRUST_NOT_FINITE);
        return false;
    }
    if (input->thrust < 0.0f || input->thrust > 1.0f) {
        externalControlRecordRejection(EXTERNAL_CONTROL_REJECT_THRUST_RANGE);
        return false;
    }
    if (input->hasYawRate && !isfinite(input->yawRate)) {
        externalControlRecordRejection(EXTERNAL_CONTROL_REJECT_YAW_RATE_NOT_FINITE);
        return false;
    }

    const float inverseNorm = 1.0f / sqrtf(normSquared);
    externalControlSetpoint_t newSetpoint = {
        .attitude = {
            .w = input->attitude.w * inverseNorm,
            .x = input->attitude.x * inverseNorm,
            .y = input->attitude.y * inverseNorm,
            .z = input->attitude.z * inverseNorm,
        },
        .thrust = input->thrust,
        .yawRate = input->hasYawRate ? input->yawRate : 0.0f,
        .sourceTimestampMs = input->sourceTimestampMs,
        .receivedAtUs = receivedAtUs,
        .hasYawRate = input->hasYawRate,
    };

    bool roundTripTimeValid = false;
    uint32_t roundTripTimeUs = 0;
    if (input->sourceTimestampMs != 0) {
        const uint32_t elapsedMs = millis() - input->sourceTimestampMs;
        if (elapsedMs <= EXTERNAL_CONTROL_MAX_RTT_MS) {
            roundTripTimeValid = true;
            roundTripTimeUs = elapsedMs * 1000U;
        }
    }

    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        diagnostics.lastReceiveIntervalUs = setpointValid ? cmpTimeUs(receivedAtUs, setpoint.receivedAtUs) : 0;
        setpoint = newSetpoint;
        setpointValid = true;
        diagnostics.acceptedCount++;
        diagnostics.lastQuaternionNormError = normError;
        diagnostics.lastRoundTripTimeUs = roundTripTimeUs;
        diagnostics.roundTripTimeValid = roundTripTimeValid;
    }

    return true;
}

bool externalControlGetSetpoint(externalControlSetpoint_t *result)
{
    bool valid;
    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        valid = setpointValid;
        if (valid) {
            *result = setpoint;
        }
    }
    return valid;
}

bool externalControlGetDiagnostics(externalControlDiagnostics_t *result)
{
    bool valid;
    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        valid = setpointValid;
        *result = diagnostics;
    }
    return valid;
}

bool externalControlIsFresh(timeUs_t currentTimeUs, timeDelta_t timeoutUs)
{
    externalControlSetpoint_t currentSetpoint;
    return externalControlGetSetpoint(&currentSetpoint) &&
        cmpTimeUs(currentTimeUs, currentSetpoint.receivedAtUs) <= timeoutUs;
}

bool externalControlUpdateMode(bool requested, bool permitted, timeUs_t currentTimeUs)
{
    externalControlModeState_e currentState;
    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        currentState = modeState;
    }

    externalControlModeState_e nextState;
    if (!requested) {
        // Cycling the pilot's switch is the only way to clear a fallback latch.
        nextState = EXTERNAL_CONTROL_MODE_DISABLED;
    } else if (currentState == EXTERNAL_CONTROL_MODE_ANGLE_FALLBACK) {
        nextState = EXTERNAL_CONTROL_MODE_ANGLE_FALLBACK;
    } else if (permitted && externalControlIsFresh(currentTimeUs, EXTERNAL_CONTROL_DEFAULT_TIMEOUT_US)) {
        nextState = EXTERNAL_CONTROL_MODE_ACTIVE;
    } else {
        nextState = EXTERNAL_CONTROL_MODE_ANGLE_FALLBACK;
    }

    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        modeRequested = requested;
        modeState = nextState;
    }
    return nextState != currentState;
}

bool externalControlIsActive(void)
{
    bool active;
    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        active = modeState == EXTERNAL_CONTROL_MODE_ACTIVE && shadowCommand.valid;
    }
    return active;
}

bool externalControlIsAngleFallbackActive(void)
{
    bool active;
    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        active = modeState == EXTERNAL_CONTROL_MODE_ANGLE_FALLBACK;
    }
    return active;
}

void externalControlUpdateShadowCommand(timeUs_t currentTimeUs)
{
    externalControlSetpoint_t currentSetpoint = { 0 };
    externalControlDiagnostics_t currentDiagnostics;
    const bool valid = externalControlGetSetpoint(&currentSetpoint);
    externalControlGetDiagnostics(&currentDiagnostics);
    const bool fresh = valid &&
        cmpTimeUs(currentTimeUs, currentSetpoint.receivedAtUs) <= EXTERNAL_CONTROL_DEFAULT_TIMEOUT_US;

    externalControlShadowCommand_t command = {
        .computedAtUs = currentTimeUs,
        .valid = fresh,
    };

    if (fresh) {
        quaternion_t currentAttitude;
        getQuaternion(&currentAttitude);
        externalControlCalculateAttitudeError(&currentAttitude, &currentSetpoint.attitude, command.attitudeError);

        // VFC/PX4 treats yaw_sp_move_rate as rotation about the earth vertical.
        // Project that axis into Betaflight's current body-FLU frame before
        // adding it as rate feed-forward.
        float yawFeedforward[XYZ_AXIS_COUNT] = { 0.0f, 0.0f, 0.0f };
        if (currentSetpoint.hasYawRate) {
            const float w = currentAttitude.w;
            const float x = currentAttitude.x;
            const float y = currentAttitude.y;
            const float z = currentAttitude.z;
            yawFeedforward[FD_ROLL] = 2.0f * (x * z - w * y) * currentSetpoint.yawRate;
            yawFeedforward[FD_PITCH] = 2.0f * (y * z + w * x) * currentSetpoint.yawRate;
            yawFeedforward[FD_YAW] = (1.0f - 2.0f * (x * x + y * y)) * currentSetpoint.yawRate;
        }

        for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
            const float unconstrainedRate = RADIANS_TO_DEGREES(
                externalControlAttitudeGain[axis] * command.attitudeError[axis] + yawFeedforward[axis]);
            if (fabsf(unconstrainedRate) > externalControlRateLimit[axis]) {
                command.saturationMask |= 1U << axis;
            }
            command.rate[axis] = constrainf(unconstrainedRate,
                -externalControlRateLimit[axis], externalControlRateLimit[axis]);
        }
        command.thrust = currentSetpoint.thrust;
    }

    bool currentModeRequested;
    externalControlModeState_e currentModeState;
    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        shadowCommand = command;
        // Drop authority at PID-loop cadence rather than waiting for the slower
        // RX mode task. The fallback remains latched until the switch is cycled.
        if (modeState == EXTERNAL_CONTROL_MODE_ACTIVE && !command.valid) {
            modeState = EXTERNAL_CONTROL_MODE_ANGLE_FALLBACK;
        }
        currentModeRequested = modeRequested;
        currentModeState = modeState;
    }

    int16_t state = externalControlStateFlags(valid, fresh, &currentSetpoint, &currentDiagnostics);
    state |= (int16_t)command.saturationMask << 4;
    state |= currentModeRequested ? EXTERNAL_CONTROL_STATE_MODE_REQUESTED : 0;
    state |= currentModeState == EXTERNAL_CONTROL_MODE_ACTIVE ? EXTERNAL_CONTROL_STATE_MODE_ACTIVE : 0;
    state |= currentModeState == EXTERNAL_CONTROL_MODE_ANGLE_FALLBACK ? EXTERNAL_CONTROL_STATE_ANGLE_FALLBACK : 0;
    DEBUG_SET(DEBUG_EXTERNAL_CONTROL_COMMAND, 0, state);
    DEBUG_SET(DEBUG_EXTERNAL_CONTROL_COMMAND, 1, externalControlDebugValue(command.thrust * 10000.0f));
    DEBUG_SET(DEBUG_EXTERNAL_CONTROL_COMMAND, 2, externalControlDebugValue(command.rate[FD_ROLL] * 10.0f));
    DEBUG_SET(DEBUG_EXTERNAL_CONTROL_COMMAND, 3, externalControlDebugValue(command.rate[FD_PITCH] * 10.0f));
    DEBUG_SET(DEBUG_EXTERNAL_CONTROL_COMMAND, 4, externalControlDebugValue(command.rate[FD_YAW] * 10.0f));
    DEBUG_SET(DEBUG_EXTERNAL_CONTROL_COMMAND, 5,
        externalControlDebugValue(RADIANS_TO_DEGREES(command.attitudeError[FD_ROLL]) * 10.0f));
    DEBUG_SET(DEBUG_EXTERNAL_CONTROL_COMMAND, 6,
        externalControlDebugValue(RADIANS_TO_DEGREES(command.attitudeError[FD_PITCH]) * 10.0f));
    DEBUG_SET(DEBUG_EXTERNAL_CONTROL_COMMAND, 7,
        externalControlDebugValue(RADIANS_TO_DEGREES(command.attitudeError[FD_YAW]) * 10.0f));
}

bool externalControlGetShadowCommand(externalControlShadowCommand_t *result)
{
    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        *result = shadowCommand;
    }
    return result->valid;
}

bool externalControlGetActiveCommand(externalControlShadowCommand_t *result)
{
    bool active;
    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        *result = shadowCommand;
        active = modeState == EXTERNAL_CONTROL_MODE_ACTIVE && shadowCommand.valid;
    }
    return active;
}

void externalControlUpdateDebug(timeUs_t currentTimeUs)
{
    externalControlSetpoint_t currentSetpoint = { 0 };
    externalControlDiagnostics_t currentDiagnostics;
    const bool valid = externalControlGetSetpoint(&currentSetpoint);
    externalControlGetDiagnostics(&currentDiagnostics);

    const bool fresh = valid &&
        cmpTimeUs(currentTimeUs, currentSetpoint.receivedAtUs) <= EXTERNAL_CONTROL_DEFAULT_TIMEOUT_US;
    const int16_t state = externalControlStateFlags(valid, fresh, &currentSetpoint, &currentDiagnostics);

    const timeDelta_t ageUs = valid ? cmpTimeUs(currentTimeUs, currentSetpoint.receivedAtUs) : -1000;
    DEBUG_SET(DEBUG_EXTERNAL_CONTROL_RX, 0, state);
    DEBUG_SET(DEBUG_EXTERNAL_CONTROL_RX, 1, externalControlDebugValue(ageUs * 0.001f));
    DEBUG_SET(DEBUG_EXTERNAL_CONTROL_RX, 2, currentDiagnostics.roundTripTimeValid ?
        externalControlDebugValue(currentDiagnostics.lastRoundTripTimeUs * 0.001f) : -1);
    DEBUG_SET(DEBUG_EXTERNAL_CONTROL_RX, 3, externalControlDebugValue(currentDiagnostics.lastReceiveIntervalUs * 0.01f));
    DEBUG_SET(DEBUG_EXTERNAL_CONTROL_RX, 4, externalControlDebugValue(currentDiagnostics.lastQuaternionNormError * 10000.0f));
    DEBUG_SET(DEBUG_EXTERNAL_CONTROL_RX, 5, valid ? externalControlDebugValue(currentSetpoint.thrust * 10000.0f) : -1);
    DEBUG_SET(DEBUG_EXTERNAL_CONTROL_RX, 6, valid && currentSetpoint.hasYawRate ?
        externalControlDebugValue(RADIANS_TO_DEGREES(currentSetpoint.yawRate) * 10.0f) : 0);
    DEBUG_SET(DEBUG_EXTERNAL_CONTROL_RX, 7, currentDiagnostics.lastRejectReason);

    float attitudeError[XYZ_AXIS_COUNT] = { 0.0f, 0.0f, 0.0f };
    if (valid) {
        quaternion_t currentAttitude;
        getQuaternion(&currentAttitude);
        externalControlCalculateAttitudeError(&currentAttitude, &currentSetpoint.attitude, attitudeError);
    }

    DEBUG_SET(DEBUG_EXTERNAL_CONTROL_ATTITUDE, 0, state);
    DEBUG_SET(DEBUG_EXTERNAL_CONTROL_ATTITUDE, 1, externalControlDebugValue(ageUs * 0.001f));
    DEBUG_SET(DEBUG_EXTERNAL_CONTROL_ATTITUDE, 2, valid ? externalControlDebugValue(currentSetpoint.thrust * 10000.0f) : -1);
    DEBUG_SET(DEBUG_EXTERNAL_CONTROL_ATTITUDE, 3, valid && currentSetpoint.hasYawRate ?
        externalControlDebugValue(RADIANS_TO_DEGREES(currentSetpoint.yawRate) * 10.0f) : 0);
    DEBUG_SET(DEBUG_EXTERNAL_CONTROL_ATTITUDE, 4, valid ?
        externalControlDebugValue(RADIANS_TO_DEGREES(attitudeError[FD_ROLL]) * 10.0f) : 0);
    DEBUG_SET(DEBUG_EXTERNAL_CONTROL_ATTITUDE, 5, valid ?
        externalControlDebugValue(RADIANS_TO_DEGREES(attitudeError[FD_PITCH]) * 10.0f) : 0);
    DEBUG_SET(DEBUG_EXTERNAL_CONTROL_ATTITUDE, 6, valid ?
        externalControlDebugValue(RADIANS_TO_DEGREES(attitudeError[FD_YAW]) * 10.0f) : 0);
    DEBUG_SET(DEBUG_EXTERNAL_CONTROL_ATTITUDE, 7, currentDiagnostics.lastRejectReason);
}

#endif
