# HEXAGON host helpers

Companion scripts for the Betaflight DSP build on VOXL.

The flight controller runs as `betaflight.so` on the Hexagon DSP (SLPI). It can't open sockets directly, so a small Python process on the application processor speaks to it over the SLPI link (`libslpi_link.so.1`) and re-exposes its serial UARTs as network endpoints:

- **MSP** (Configurator) → WebSocket on `0.0.0.0:8765`
- **MAVLink** → UDP on `0.0.0.0:14550`
- **OSD** → MPA pipe (`msp_osd`)

That process is the `betaflight` script in this directory. It runs on the VOXL device. Everything else here is laptop-side tooling.

## Files

| Path | Runs on | What it does |
|---|---|---|
| `betaflight` | voxl3 device | Loads `libslpi_link.so.1`, multiplexes MSP / MAVLink / OSD over the SLPI link. |
| `voxl-configurator` | laptop | Sets up `adb forward` and opens the Betaflight Configurator PWA in a dedicated Chrome window. |
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

## Stats

The `betaflight` script prints a one-line-per-second traffic summary whenever any counter is non-zero:

```
[1s] MAVLink up=3p/420B dn=1p/14B dropped=0
     MSP     up=12p/156B dn=12p/68B
     OSD     up=0p/0B
```

`up` = DSP → laptop, `dn` = laptop → DSP.
