# HEXAGON host helpers

Companion scripts for the Betaflight DSP build on VOXL.

The flight controller runs as `betaflight.so` on the Hexagon DSP (SLPI). It can't open sockets directly, so a small Python process on the application processor speaks to it over the SLPI link (`libslpi_link.so.1`) and re-exposes its serial UARTs as network endpoints:

- **MSP** (Configurator) → WebSocket on `0.0.0.0:8765`
- **MAVLink** → UDP on `0.0.0.0:14550`
- **OSD** → MPA pipe (`msp_osd`)

That process is the `betaflight` script in this directory. It runs on the VOXL
device, as does `bf-cli` by default. The remaining helpers are laptop-side
tooling.

## Files

| Path | Runs on | What it does |
|---|---|---|
| `betaflight` | voxl3 device | Loads `libslpi_link.so.1`, multiplexes MSP / MAVLink / OSD over the SLPI link. |
| `voxl-configurator` | laptop | Sets up `adb forward` and opens the Betaflight Configurator PWA in a dedicated Chrome window. |
| `bf-cli` | voxl3 device / laptop | Opens an interactive CLI or executes one CLI command over the MSP WebSocket. |
| `bf_gcs.py` | laptop | Minimal Tkinter MAVLink GCS with joystick → `RC_OVERRIDE`. |
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
adb push bf-cli /usr/bin/bf-cli
adb shell chmod +x /usr/bin/bf-cli
```

### On the laptop

For `bf_gcs.py` (optional), create the venv and install its Python deps:

```sh
python3 -m venv .venv
.venv/bin/pip install pymavlink pygame
```

`voxl-configurator` only needs `adb` and `google-chrome` on PATH. The on-device
`python3-websockets` package installed above also supports `bf-cli`. For optional
laptop use, install that package with `sudo apt install python3-websockets` or
run `pip install websockets` in an activated virtual environment.

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

## Command-line interface

`bf-cli` runs on the VOXL device by default and connects to the local `betaflight`
bridge. Disconnect Betaflight Configurator first; the MSP virtual UART supports
only one client at a time. Run `bf-cli --help` for a summary of every supported
use case and its safety behavior. `bf-cli help` is deliberately different: it
sends Betaflight's own `help` command to the flight controller.

With no arguments, it enters the normal interactive Betaflight CLI:

```sh
bf-cli
```

Type `exit` or press Ctrl-D to leave interactive mode and reboot Betaflight.
Any arguments are joined with spaces and executed as one non-interactive CLI
command, so common queries stay short:

```sh
bf-cli version
bf-cli status
bf-cli get gyro_lpf1_static_hz
bf-cli set gyro_lpf1_static_hz=100
bf-cli calibrate acc
bf-cli save
```

`bf-cli version` reports the upstream Betaflight version and a separate fork
version derived from the closest matching fork release tag. Development builds
include their commit distance and abbreviated revision; builds made with local
source changes also end in `-dirty`. The fork version uses Betaflight's existing
`RELEASE_NAME` build field and can be queried directly with
`bf-cli env RELEASE_NAME`. If the Docker build does not have access to the
repository's `.git` directory, pass `FORK_VERSION=<version>` to the build;
otherwise the embedded fork version falls back to
`<betaflight-version>-unknown`. Official releases should be built from a clean
checkout at the corresponding fork tag.

`bf-cli calibrate acc` refuses to start while the flight controller reports
armed, asks Betaflight to calibrate through `MSP_ACC_CALIBRATION`, waits for the
calibration to finish, and prints the stored offsets. Put the drone in its normal
flight orientation on a level, vibration-free surface and do not move it until
the command completes. Betaflight saves the accelerometer calibration
automatically.

The gyro calibrates automatically whenever Betaflight boots, so keep the drone
still during startup. Run accelerometer calibration after applying and saving
the rest of the vehicle configuration: alignment changes take effect after the
reboot caused by `save`. The current `eric-prototype.config` contains a fixed
`acc_calibration` setting; applying that file later will overwrite a calibration
measured for an individual vehicle.

For optional laptop use, set `BF_CLI_ADB=1`; the helper then creates the
`adb forward tcp:8765 tcp:8765` rule before connecting:

```sh
BF_CLI_ADB=1 ./bf-cli status
```

Set `BF_CLI_URL` to use another WebSocket endpoint.

> **Safety:** Entering the interactive CLI prevents subsequent arming, but does
> not disarm an already-armed vehicle. Use it only while disarmed and on the
> bench. If the client disconnects unexpectedly, Betaflight remains in CLI mode
> with arming disabled. `exit noreboot` also leaves arming disabled until the
> flight controller is rebooted. Short non-interactive commands do not set the
> CLI arming-disable flag.

## MAVLink GCS (optional)

The on-device script also forwards MAVLink between the DSP and UDP 14550. To use the included GCS:

```sh
.venv/bin/python3 bf_gcs.py --target <device-ip>:14550
```

It heartbeats as a GCS so the FC begins streaming telemetry, then shows link status, attitude, battery, and (with a gamepad attached) sends `RC_OVERRIDE`.

Any standard MAVLink GCS (QGroundControl, MAVProxy) will also work against UDP 14550 — see the project-level note that Betaflight's MAVLink is really meant for RC-link consumers (Crossfire / Tango / Yaapu), not full GCS workflows.

## Stats

The `betaflight` script prints a one-line-per-second traffic summary whenever any counter is non-zero:

```
[1s] MAVLink up=3p/420B dn=1p/14B dropped=0
     MSP     up=12p/156B dn=12p/68B
     OSD     up=0p/0B
```

`up` = DSP → laptop, `dn` = laptop → DSP.
