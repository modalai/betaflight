# HEXAGON host helpers

Companion scripts for the Betaflight DSP build on VOXL.

The flight controller runs as `betaflight.so` on the Hexagon DSP (SLPI). It can't open sockets directly, so a small Python process on the application processor speaks to it over the SLPI link (`libslpi_link.so.1`) and re-exposes its serial UARTs as network endpoints:

- **MSP** (Configurator) → WebSocket on `0.0.0.0:8765`
- **MAVLink** → UDP on `0.0.0.0:14550` and a bidirectional MPA pipe (`betaflight_mavlink`)
- **OSD** → MPA pipe (`msp_osd`)

That process is the `betaflight` script in this directory. The loopback helper
also runs on VOXL; the remaining files are laptop-side tooling.

## Files

| Path | Runs on | What it does |
|---|---|---|
| `betaflight` | voxl3 device | Loads `libslpi_link.so.1`, multiplexes MSP / MAVLink / OSD over the SLPI link. |
| `voxl-configurator` | laptop | Sets up `adb forward` and opens the Betaflight Configurator PWA in a dedicated Chrome window. |
| `bf_gcs.py` | laptop | Minimal Tkinter MAVLink GCS with joystick → `RC_OVERRIDE`. |
| `bf_external_control_loopback.py` | voxl3 device | Echoes `ATTITUDE_TARGET` to `SET_ATTITUDE_TARGET` through the MPA pipe for passive validation. |
| `voxl-inspect-osd` | laptop | Prints the OSD MPA pipe contents (debug only). |
| `ws2tcp.py`, `ws2udp.py`, `betaflight_udp` | laptop / voxl2 | Legacy bridges from the voxl2 era. Not used on voxl3. |

## One-time setup

### On the voxl3 device

Install the websockets library used by the `betaflight` script:

```sh
adb shell apt install -y python3-websockets
```

(Ubuntu 24.04 noble/universe; v10.4 is sufficient — the script uses the asyncio API.)

Push the script to the device, e.g.:

```sh
adb push betaflight /usr/bin/betaflight
adb shell chmod +x /usr/bin/betaflight
```

### On the laptop

For `bf_gcs.py` (optional), create the venv and install its Python deps:

```sh
python3 -m venv .venv
.venv/bin/pip install pymavlink pygame
```

`voxl-configurator` only needs `adb` and `google-chrome` on PATH.

## Running

Start the bridge on the device:

```sh
adb shell betaflight
```

Expected output:

```
slpi_link_init returned: 0
MAVLink UDP listening on 0.0.0.0:14550
WebSocket server started at ws://0.0.0.0:8765
```

Then on the laptop:

```sh
./voxl-configurator
```

This forwards laptop `localhost:8765` → device `8765` over adb and launches a chromeless Chrome window on `https://app.betaflight.com/#` with an isolated profile in `/tmp/voxl-configurator-profile`. Inside the Configurator, set the connection URL to **`ws://localhost:8765`** and connect.

> The Configurator silently refuses to initiate a WebSocket connection to a non-loopback host, so the `adb forward` + `ws://localhost:…` indirection is required even though the device's WS server binds to `0.0.0.0`.

## MAVLink GCS (optional)

The on-device script also forwards MAVLink between the DSP and UDP 14550. To use the included GCS:

```sh
.venv/bin/python3 bf_gcs.py --target <device-ip>:14550
```

It heartbeats as a GCS so the FC begins streaming telemetry, then shows link status, attitude, battery, and (with a gamepad attached) sends `RC_OVERRIDE`.

Any standard MAVLink GCS (QGroundControl, MAVProxy) will also work against UDP 14550 — see the project-level note that Betaflight's MAVLink is really meant for RC-link consumers (Crossfire / Tango / Yaapu), not full GCS workflows.

The `betaflight_mavlink` MPA data pipe contains encoded MAVLink wire bytes. A
client sends bytes in the other direction through that pipe's control FIFO.
This is intentionally a byte stream rather than the platform-specific
`mavlink_message_t` memory layout, so the virtual UART and a future physical
UART use the same framing.

For the passive loopback test on the device (disarmed, with propellers removed):

```sh
python3 bf_external_control_loopback.py --rate-hz 20
```

The tool requests `ATTITUDE_TARGET` at the selected rate, validates its MAVLink
checksum, and by default preserves timestamp/quaternion/yaw-rate/thrust/type-mask
fields when returning it as `SET_ATTITUDE_TARGET`. It uses only Python's
standard library and `libmodal_pipe.so`.

While this firmware phase remains passive, optional `--thrust 0.25`,
`--yaw-rate 0.5`, and attitude-offset arguments can exercise the shadow
controller without reaching the active PID setpoint or mixer. The offsets are
specified in Betaflight's native body-FLU axes and degrees. For example:

```sh
python3 bf_external_control_loopback.py --rate-hz 20 --thrust 0.25 \
    --roll-offset 10
```

Do not use these overrides after external setpoints gain control authority.

## External-control Blackbox diagnostics

Three debug modes are available for passive validation:

- `EXT_CTRL_RX`: state flags, setpoint age (ms), loopback RTT (ms), receive
  interval (0.1 ms), quaternion norm error (x10000), thrust (x10000), yaw rate
  (deg/s x10), and the last rejection code.
- `EXT_CTRL_ATT`: state flags, setpoint age (ms), thrust (x10000), yaw rate
  (deg/s x10), and X/Y/Z shortest-path attitude error (deg x10), followed by
  the last rejection code.
- `EXT_CTRL_CMD`: state flags plus rate-saturation flags, command thrust
  (x10000), roll/pitch/yaw shadow rate (deg/s x10), and roll/pitch/yaw
  shortest-path attitude error (deg x10).

The state flags are valid=`1`, fresh=`2`, yaw-rate-present=`4`, and
RTT-valid=`8`. A steady passive loopback should normally report state `15`,
zero thrust, small attitude error, and rejection code `0`. VFC's normal
zero-valued source timestamp leaves RTT invalid, so its healthy state is `7`.
Rejection codes are target=`1`, type-mask=`2`, non-finite attitude=`3`, bad
quaternion norm=`4`, non-finite thrust=`5`, out-of-range thrust=`6`, and
non-finite yaw rate=`7`.

The logged yaw rate is Betaflight-native and CCW-positive. MAVLink/PX4 body-FRD
yaw is clockwise-positive, so a loopback override of `--yaw-rate 0.5` should
appear near `-286` in the x10-deg/s debug channel.

The shadow controller currently uses the provisional PX4 multicopter defaults:
attitude gains of 4.0/4.0/2.8 per second and rate limits of 220/220/200 deg/s.
It adds VFC's yaw-rate input as an earth-vertical feed-forward projected into
the current body frame. These constants are deliberately not user parameters
until bench results establish useful ranges.

Before testing that earth-frame projection, verify that Configurator shows the
vehicle upright and that its model follows physical roll and pitch correctly.
An incorrect board alignment can still pass the relative attitude-offset tests
but invalidates the earth-vertical projection. Correct board alignment first,
then calibrate the accelerometer on a level surface.

With `EXT_CTRL_CMD` selected and the vehicle stationary, the example above
should produce approximately:

- debug 0 = `15` (valid, fresh, yaw-rate present, and RTT valid)
- debug 1 = `2500` (thrust 0.25)
- debug 2 = `400` (about 40 deg/s roll command)
- debug 3 and 4 = `0`
- debug 5 = `100` (about 10 degrees roll error)
- debug 6 and 7 = `0`

Use one offset at a time for the first axis/sign checks. `--pitch-offset 10`
should move debug 3 and 6 positive; `--yaw-offset 10` should move debug 4 to
about `280` and debug 7 to about `100`. Exact readings can vary slightly due
to quaternion math and motion of the fixture. With no offsets, all three rate
commands and errors should remain near zero.

A large offset can check limiting without affecting the aircraft. A
`--roll-offset 90` command should clamp debug 2 at `2200` and add the roll
saturation bit (`16`) to debug 0, making the usual loopback state `31`.
Stopping the loopback should make the fresh bit clear after 200 ms; debug 0
returns to `13`, while command thrust, rates, and errors all become zero.

`EXT_CTRL_CMD` is calculated at the start of the normal PID loop so its timing
matches a future integration point. In this phase its result is only copied to
a diagnostic mailbox and debug channels. It does not modify RC input,
`pidSetpoint`, the rate PID, mixer throttle, arming state, or motor output.

Select one mode before a log, for example:

```text
set debug_mode = EXT_CTRL_RX
save
```

The HEXAGON configurations already select serial Blackbox on UART7. The DSP
writer stores it at `/data/betaflight/log.bbl` and now flushes the last partial
buffer during Blackbox shutdown, so a short bench capture is retained. Standard
Blackbox fields already capture Betaflight's measured quaternion and mixer
throttle. Copy the log before rebooting into another debug mode because the
next process starts a new `log.bbl`.

## Stats

The `betaflight` script prints a one-line-per-second traffic summary whenever any counter is non-zero:

```
[1s] MAVLink up=3p/420B dn=1p/14B dropped=0
     MAV MPA up=3p/420B dn=1p/63B errors=0
     MSP     up=12p/156B dn=12p/68B
     OSD     up=0p/0B
```

`up` = DSP → laptop, `dn` = laptop → DSP.
