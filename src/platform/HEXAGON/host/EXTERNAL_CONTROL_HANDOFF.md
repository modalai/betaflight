# External thrust and attitude control handoff

Last updated: 2026-08-10

## Working rules

- Do not build, link, or compile this repository from an assistant session. The
  owner builds it in a special Docker environment.
- Ask before running any built artifact. Firmware and target-side programs are
  normally run by the owner on the target.
- All active-path tests so far have been performed with the propellers removed.
- Inspect the current branch, working tree, and this document before resuming.

## Goal

ModalAI's VFC implementation produces a scalar thrust command, an attitude
quaternion, and an optional yaw-rate feed-forward. PX4 accepts that contract via
MAVLink `SET_ATTITUDE_TARGET`. This branch adds the equivalent internal control
path to Betaflight while keeping the transport replaceable.

The desired deployed interface is an external UART. On HEXAGON, UART6 is a
virtual UART carried through the SLPI link to the application processor. The
host-side `betaflight` process exposes its byte stream through an MPA pipe, so a
future VFC-compatible application can connect without putting MPA-specific code
in the flight controller.

Reference implementations outside this repository:

- PX4 receiver: `../../px4/voxl-fpv-dev-1.18.0/src/modules/mavlink/mavlink_receiver.cpp`,
  especially `MavlinkReceiver::handle_message_set_attitude_target()`.
- PX4 attitude controller: `../../px4/voxl-fpv-dev-1.18.0/src/modules/mc_att_control/`.
- VFC producer: `../../modalai/voxl-vision-hub/src/offboard_vfc.c`, especially
  `_fill_attitude_target()` and the send near the end of the control loop.

VFC is owned by ModalAI and may be changed. Compatibility with the exact
current MAVLink message is useful but is not a hard architectural constraint.

## Repository state at this handoff

The feature branch is `external-thrust-attitude`. It was corrected to be based
on `origin/voxl3-dev`, not Betaflight master. The merge base at the time of this
handoff is `ee652ee89` (`Debug: report gyro loop overruns`).

The branch was five commits ahead of `origin/voxl3-dev` before this document was
added:

| Commit | Purpose |
|---|---|
| `6425cad65` | Flush the final partial HEXAGON Blackbox buffer. |
| `9647b1c69` | Name the HEXAGON Blackbox file `log.bbl`. |
| `5eb0a3458` | Add the passive external-control mailbox, MAVLink adapter, controller diagnostics, virtual-UART bridge, and loopback helper. |
| `24b46cda9` | Grant external commands PID/mixer authority and add the safety/fallback state machine. |
| `fd285a7b0` | Suppress throttle boost across external/RC authority handoffs. |

The first two Blackbox commits are intentionally separate from each other and
from the external-controller work. They are candidates for early integration
into `voxl3-dev`. Preserve that commit separation when rebasing or
cherry-picking.

At the time this document was created, `origin/external-thrust-attitude` pointed
to `9647b1c69`; the three controller commits had not yet been pushed to that
remote branch. Recheck the remotes rather than assuming this is still true.

## Implemented architecture

```text
VFC replacement or loopback app
        |
        | encoded MAVLink byte stream through betaflight_mavlink MPA pipe
        v
apps-processor betaflight bridge
        |
        | SLPI virtual-UART frames, tag 0x4d
        v
HEXAGON UART6 / Betaflight MAVLink telemetry RX
        |
        | MAVLink adapter and frame conversion
        v
transport-neutral external-control mailbox
        |
        | attitude error -> body-rate command; scalar thrust
        v
normal Betaflight rate PID and mixer/output safety path
```

Important files:

- `src/main/flight/external_control.c` and `.h`: validation, mailbox,
  freshness, state machine, attitude controller, and debug output.
- `src/main/telemetry/mavlink.c`: `SET_ATTITUDE_TARGET` adapter and diagnostic
  `ATTITUDE_TARGET` sender.
- `src/main/flight/pid.c`: calculates the command each PID iteration and
  selects external rate setpoints while authority is active.
- `src/main/flight/mixer.c`: selects external normalized thrust and handles a
  bumpless throttle-boost handoff.
- `src/main/fc/core.c`: mode permission, arming inhibition, Angle fallback,
  PID enablement, and runaway-takeoff integration.
- `src/platform/HEXAGON/serial_uart_hexagon.c`: virtual UART6 transport and
  independent Blackbox changes.
- `src/platform/HEXAGON/host/betaflight`: SLPI-to-MPA/UDP bridge.
- `src/platform/HEXAGON/host/bf_external_control_loopback.py`: deterministic
  bench source and echo tester.
- `src/platform/HEXAGON/host/README.md`: current setup, debug-field reference,
  and operator-facing bench procedure.

`USE_EXTERNAL_CONTROL` is currently enabled by the HEXAGONV66 target. The core
input structure uses Betaflight-native coordinates and does not depend on
MAVLink, so another UART protocol can be added as an adapter later.

## Input contract and frame conversion

The phase-one MAVLink adapter accepts `SET_ATTITUDE_TARGET` when:

- the target system/component is broadcast (`0`) or matches Betaflight;
- attitude and scalar thrust are present;
- roll and pitch body-rate fields are ignored by the type mask; and
- yaw rate is either present or explicitly ignored.

The adapter converts MAVLink/PX4 body-FRD and earth-NED into Betaflight
body-FLU and earth-NWU:

```text
quaternion [w, x, y, z] -> [w, x, -y, -z]
yaw rate                   -> -yaw rate
```

The mailbox rejects non-finite attitude, thrust, or optional yaw rate; a
quaternion norm squared outside `[0.5, 1.5]`; thrust outside `[0, 1]`; an
unsupported type mask; or a wrong target. Accepted quaternions are normalized.
The default freshness timeout is 200 ms.

The outbound `ATTITUDE_TARGET` message exists only as a bench reference for the
loopback tester. It reports measured attitude with zero thrust; it is not the
active Betaflight control target.

## Attitude controller

The controller computes the shortest-path body attitude error from
`current^-1 * desired`, then turns that error into a body-rate setpoint for the
normal Betaflight rate PID. Current provisional gains and limits are:

| Axis | Attitude gain | Rate limit |
|---|---:|---:|
| Roll | 4.0 /s | 220 deg/s |
| Pitch | 4.0 /s | 220 deg/s |
| Yaw | 2.8 /s | 200 deg/s |

These match the initial PX4 multicopter defaults and remain compile-time
constants. VFC's yaw move rate is treated as rotation about earth vertical and
projected into the current Betaflight body frame before being added as
feed-forward.

While active:

- external rates replace RC rate setpoints at the rate PID;
- external thrust replaces normalized RC throttle before configured throttle
  limits and the normal mixer/output safety path;
- Betaflight's own Angle calculation is bypassed for the external setpoint;
- the path is treated as requiring active stabilization/Airmode behavior; and
- throttle boost is disabled for external thrust.

PID I terms reset whenever authority changes. The throttle-boost filter is
reseeded from the newly selected source on both sides of a handoff, preventing
the source change from appearing as a throttle transient.

## Authority and fallback behavior

`EXTERNAL CTRL` is an AUX mode exposed in Configurator.

- The vehicle must be armed with `EXTERNAL CTRL` deselected. Selecting it while
  disarmed reports the arming-disable flag `EXTCTRL_SW`.
- Authority requires a fresh target, an accelerometer, valid RC flight
  channels, a multirotor configuration, no 3D feature, and no active failsafe,
  navigation, crash-flip, launch-control, or chirp mode.
- If a target becomes stale or authority becomes impermissible, external
  authority is removed and stick-controlled Angle mode is selected.
- RC link loss uses Betaflight's normal failsafe instead of pretending stale RC
  inputs are valid fallback commands.
- Fallback is latched. A resumed command stream does not retake authority. The
  pilot must move `EXTERNAL CTRL` out of its AUX range and deliberately select
  it again.
- The RC throttle stick should be near a sustainable value before selecting
  external control, because that value becomes the immediate fallback throttle.

Common `EXT_CTRL_CMD` state values with the timestamped loopback are:

| Value | Meaning |
|---:|---|
| `15` | Valid/fresh command available, mode not requested. |
| `399` | Mode requested with active external authority. |
| `653` | Command stale; latched Angle fallback. |
| `655` | Command fresh again, but fallback remains latched. |

See the host README for the individual flag bits, VFC's timestamp-zero values,
all three debug modes, and rejection codes.

## HEXAGON transport and setup

HEXAGONV66 maps MAVLink to virtual UART6. New default configuration assigns
function mask `512` (`FUNCTION_TELEMETRY_MAVLINK`) to that UART. A target with
an older saved configuration may need the CLI setting restored explicitly:

```text
serial UART6 512 115200 57600 0 115200
save
```

UART7 remains the serial Blackbox endpoint. The target-side bridge exposes:

- MAVLink UDP on `0.0.0.0:14550`;
- MAVLink MPA on `/run/mpa/betaflight_mavlink/`;
- Configurator/MSP WebSocket on `0.0.0.0:8765`; and
- OSD on the `msp_osd` MPA pipe.

The MPA transport carries complete encoded MAVLink bytes, not an in-memory
`mavlink_message_t`, so it has the same framing semantics as a physical UART.

Basic propeller-free loopback invocation:

```sh
python3 bf_external_control_loopback.py --rate-hz 20 --thrust 0.10
```

For a small attitude-path check:

```sh
python3 bf_external_control_loopback.py --rate-hz 20 --thrust 0.10 \
    --roll-offset 2
```

Keep nonzero relative offsets brief on a stationary fixture. The controller
cannot eliminate the error, so its I term will eventually saturate and create
large motor separation even though the signal path is operating correctly.

## Observability

Select a debug mode in the CLI, for example:

```text
set debug_mode = EXT_CTRL_CMD
save
```

Available modes are `EXT_CTRL_RX`, `EXT_CTRL_ATT`, and `EXT_CTRL_CMD`. The last
one records the command that is about to enter the PID loop:

| Debug field | `EXT_CTRL_CMD` meaning |
|---:|---|
| 0 | Transport, limiting, mode-request, authority, and fallback flags. |
| 1 | Normalized thrust x10000. |
| 2-4 | Roll, pitch, and yaw rate commands in deg/s x10. |
| 5-7 | Roll, pitch, and yaw attitude errors in degrees x10. |

Standard Blackbox setpoint and mixer-throttle fields show the values that
actually reached the PID and mixer paths. Current Blackbox Explorer expects the
`.bbl` filename extension. Do not commit large `.bbl` captures; preserve their
important conclusions here instead.

Blackbox's slow field named `flightModeFlags` is populated from
`rcModeActivationMask`. It can prove that the pilot's ordinary ANGLE AUX mode
was deselected, but it cannot independently prove that the internal Angle flag
was automatically enabled by fallback. The live Configurator mode indicator is
the direct check for that behavior.

## Validation completed

All tests below were performed on a HEXAGON target with propellers removed.

### Transport and passive controller

- The host bridge and `betaflight_mavlink` MPA pipe carried bidirectional
  MAVLink successfully at 20 Hz.
- Loopback reported targets received and echoed with no bad CRCs or TX errors.
- `EXT_CTRL_RX` showed valid/fresh data, expected receive age/interval, and
  loopback RTT.
- A `0.25` thrust override appeared as `2500`.
- A MAVLink yaw rate of `+0.5 rad/s` appeared near `-286` in Betaflight's
  deg/s-x10 convention, confirming the FRD-to-FLU sign change.
- Roll, pitch, and yaw relative offsets produced the expected controller axes
  and signs. Saturation and stale-command diagnostics also behaved as expected.
- Configurator initially displayed an incorrect physical orientation. Board
  alignment and accelerometer calibration were corrected before active tests;
  the model then followed physical roll and pitch properly.

### Arming and state machine

- Selecting `EXTERNAL CTRL` while disarmed exposed `CLI MSP EXTCTRL_SW` among
  the arming-disable flags.
- A healthy command and deliberate AUX selection produced state `399`.
- Stopping the stream produced `399 -> 653` after the timeout and returned rate
  and throttle control to the RC sticks.
- Restarting the stream produced `653 -> 655` without authority reacquisition.
- Cycling the AUX switch produced `655 -> 15 -> 399` and deliberately restored
  authority.
- No unexpected Betaflight failsafe occurred in the reviewed captures.

If not already consciously observed during the bench session, explicitly
confirm the Configurator ANGLE indicator at the next stale transition. The
Blackbox limitation described above means the saved logs cannot prove this one
UI-visible detail on their own.

### Armed zero-offset path and handoff

- External thrust held exactly `0.100` while RC throttle and axes moved.
- External zero rate setpoints remained authoritative despite RC stick motion.
- A stale transition reset PID I terms, selected RC/Angle fallback, and retained
  the no-reacquisition latch.
- The first reviewed handoff exposed a throttle-boost artifact: a transition
  from external `0.100` to RC throttle near `0.455` briefly reached about
  `0.595` because `throttle_boost = 5` interpreted the source change as a
  transient.
- Commit `fd285a7b0` fixed this. The repeated mismatch changed directly from
  `0.100` to `0.460`, with no synthetic boost pulse.

### Armed nonzero attitude path

The final reviewed test used thrust `0.10` and a relative roll offset of `2`
degrees:

- `debug[5]` was `20`, representing 2.0 degrees of roll error.
- `debug[2]` was approximately `80`, representing 8 deg/s from the 4.0/s gain.
- Blackbox's actual roll setpoint remained 8-9 deg/s and thrust remained
  exactly `0.100`, even during large RC movements.
- The expected roll motor-pair differentiation appeared.
- On timeout, state changed `399 -> 653`, commands returned to RC fallback, and
  I terms reset from `[400, 9, -4]` to zero.
- Stream restart and switch cycling again proved the fallback latch.

The first active interval was held for about 15.6 seconds, longer than needed.
The roll I term reached its configured limit and two motors approached
1800-1900 because a stationary, propeller-free vehicle could not satisfy the
persistent relative target. This was expected fixture behavior, not a handoff
spike.

The last two reviewed logs each contained one incomplete trailing frame but
zero invalid main frames. Their hundreds of thousands of valid frames decoded
normally, so the partial EOF did not affect any conclusions. The raw logs were
deleted after their conclusions were recorded.

## Known gaps and next decisions

The initial active-path bench phase is complete, subject to the explicit live
ANGLE-indicator confirmation noted above. No test with propellers has occurred.

Before flight, consider the following work:

1. Perform a focused code review of authority transitions and Betaflight safety
   interactions.
2. Exercise an RC-link loss and representative incompatible modes on the
   propeller-free bench to confirm that normal failsafe or the latched fallback
   wins as designed.
3. Decide whether the 200 ms timeout, gains, and rate limits should become a
   configuration group rather than compile-time constants.
4. Define a conservative restrained/tethered first-prop test with low external
   thrust, a neutral attitude target, RC throttle prepared for fallback, and an
   immediate pilot disarm path.
5. Replace the echo helper with an application that consumes VFC output (likely
   through an MPA pipe) and emits the accepted external-control stream to
   `betaflight_mavlink`.
6. Decide whether to keep MAVLink as the physical-UART protocol or add a smaller
   adapter. The internal controller should not need to change either way.
7. Tune controller gains, rate limits, thrust behavior, and transition behavior
   from restrained-flight data before unrestricted flight.
8. Integrate `6425cad65` and `9647b1c69` into `voxl3-dev` independently of this
   feature when ready.

Other areas not yet validated comprehensively include malformed-message fault
injection, long-duration command timing/jitter, system/component targeting with
the eventual application, reconnect/restart behavior of that application, and
non-HEXAGON physical-UART operation.

## Resume checklist

1. Read the repository's `AGENTS.md` instructions and this document.
2. Run read-only `git status`, `git log`, and branch/upstream checks. Preserve
   unrelated user changes.
3. Re-read `src/platform/HEXAGON/host/README.md` for the current operator
   procedure; this handoff records history and intent, while the README is the
   primary usage reference.
4. Confirm whether the two independent Blackbox commits have already been
   integrated into `voxl3-dev` and whether the feature commits were pushed.
5. Ask which item under "Known gaps and next decisions" should be tackled.
6. Do not build or run target code. Make source changes and tell the owner when
   a Docker build or target-side test is required.

A concise prompt for a new assistant session is:

> Read `AGENTS.md` and
> `src/platform/HEXAGON/host/EXTERNAL_CONTROL_HANDOFF.md`. Inspect the current
> branch and working tree before changing anything. Resume the external
> thrust/attitude controller from the documented status. Do not build or run
> target code; I handle that in a special Docker environment and on target.
