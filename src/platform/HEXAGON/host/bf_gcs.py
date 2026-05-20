#!/usr/bin/env python3
"""
Lightweight MAVLink ground station for Betaflight on VOXL.

Talks to the host-side `betaflight` bridge running on the VOXL board, which
exposes MAVLink on UDP 14550. This app sends GCS HEARTBEATs so the bridge
learns our address and starts forwarding telemetry up to us, then shows a
minimal Tkinter dashboard:

  - link status (HEARTBEAT age, armed flag, custom_mode)
  - ATTITUDE (roll / pitch / yaw)
  - battery voltage (SYS_STATUS or BATTERY_STATUS, whichever arrives)
  - joystick -> RC_OVERRIDE (Mode 2, gated by an Engage checkbox)

Usage:
    .venv/bin/pip install pymavlink pygame
    .venv/bin/python3 bf_gcs.py --target 192.168.1.42:14550
"""

import argparse
import math
import os
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk

from pymavlink import mavutil

try:
    # SDL needs *some* video driver to init even though we only want joystick.
    os.environ.setdefault('SDL_VIDEODRIVER', 'dummy')
    import pygame
    HAVE_PYGAME = True
except ImportError:
    HAVE_PYGAME = False


# We identify on the bus as a GCS so the FC begins streaming telemetry.
GCS_SYSTEM_ID = 255
GCS_COMPONENT_ID = mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER
HEARTBEAT_HZ = 1.0

# How often the Tk loop drains the rx queue (ms).
GUI_POLL_MS = 50

# --- Joystick / RC_OVERRIDE ---
# Mode 2 mapping for an xbox-style controller as exposed by SDL on Linux.
# Override on the command line if your gamepad enumerates axes differently.
AXIS_ROLL     = 3   # right stick X
AXIS_PITCH    = 4   # right stick Y (inverted: up = nose up = high pitch PWM)
AXIS_THROTTLE = 1   # left  stick Y (inverted: up = more throttle)
AXIS_YAW      = 0   # left  stick X
ARM_BUTTON    = 0   # 'A' on xbox / cross on PS — press toggles AUX1 high/low

AXIS_INVERT = {AXIS_PITCH, AXIS_THROTTLE}
AXIS_DEADZONE = 0.05

RC_OVERRIDE_HZ = 50
PWM_MIN, PWM_MID, PWM_MAX = 1000, 1500, 2000


def parse_target(s):
    """'host' or 'host:port' -> pymavlink 'udpout:host:port' connection string."""
    if ':' in s:
        host, port = s.rsplit(':', 1)
    else:
        host, port = s, '14550'
    return f'udpout:{host}:{port}'


class MavlinkLink(threading.Thread):
    """Background rx/tx pump. Pushes parsed messages onto a queue for the GUI."""

    def __init__(self, conn_str, update_q):
        super().__init__(daemon=True)
        self.conn_str = conn_str
        self.update_q = update_q
        self.stop_flag = threading.Event()
        self.master = None

    def run(self):
        try:
            self.master = mavutil.mavlink_connection(
                self.conn_str,
                source_system=GCS_SYSTEM_ID,
                source_component=GCS_COMPONENT_ID,
                dialect='common',
            )
        except Exception as e:
            self.update_q.put(('error', f'connect: {e}'))
            return

        self.update_q.put(('info', f'opened {self.conn_str}'))
        next_hb = 0.0
        hb_interval = 1.0 / HEARTBEAT_HZ

        while not self.stop_flag.is_set():
            now = time.monotonic()
            if now >= next_hb:
                try:
                    self.master.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                        0, 0, 0,
                    )
                except Exception as e:
                    self.update_q.put(('error', f'tx heartbeat: {e}'))
                next_hb = now + hb_interval

            msg = self.master.recv_match(blocking=True, timeout=0.2)
            if msg is None:
                continue
            t = msg.get_type()
            if t == 'BAD_DATA':
                continue
            if t in ('HEARTBEAT', 'ATTITUDE', 'SYS_STATUS', 'BATTERY_STATUS'):
                self.update_q.put((t, msg.to_dict()))

    def send_rc_override(self, channels):
        """Stub for the future joystick path. `channels` is an 8-tuple of uint16 PWMs."""
        if self.master is None:
            return
        ch = list(channels) + [0] * (8 - len(channels))
        self.master.mav.rc_channels_override_send(
            self.master.target_system or 1,
            self.master.target_component or 1,
            *ch[:8],
        )

    def stop(self):
        self.stop_flag.set()


def _axis_to_pwm(v, invert=False):
    if invert:
        v = -v
    if abs(v) < AXIS_DEADZONE:
        v = 0.0
    v = max(-1.0, min(1.0, v))
    return int(round(PWM_MID + v * 500))


class Joystick(threading.Thread):
    """Polls a gamepad and (when engaged) streams RC_OVERRIDE to the FC."""

    def __init__(self, link, update_q):
        super().__init__(daemon=True)
        self.link = link
        self.update_q = update_q
        self.stop_flag = threading.Event()
        self.engaged = False
        self.armed = False
        self._js = None

    def set_engaged(self, on):
        # Always clear armed when crossing the engage boundary in either direction.
        # Engaging therefore never re-arms with a stale latched state — the user
        # has to press the arm button again after engaging.
        self.engaged = bool(on)
        self.armed = False
        self.update_q.put(('joy_arm', False))
        if not self.engaged:
            # NOT zeros: betaflight's MAVLink RX driver treats 0 as "skip this
            # slot" (rx/mavlink.c:65), so an all-zero override is a no-op and the
            # FC keeps the last commanded throttle latched. Send an explicit safe
            # state instead — sticks centered, throttle low, AUX1 low (disarm).
            self.link.send_rc_override((PWM_MID, PWM_MID, PWM_MIN, PWM_MID, PWM_MIN))

    def run(self):
        if not HAVE_PYGAME:
            self.update_q.put(('joy_status', 'pygame not installed (pip install pygame)'))
            return
        try:
            pygame.init()
            pygame.joystick.init()
        except Exception as e:
            self.update_q.put(('joy_status', f'pygame init failed: {e}'))
            return

        period = 1.0 / RC_OVERRIDE_HZ
        last_count = -1
        while not self.stop_flag.is_set():
            t0 = time.monotonic()

            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN and event.button == ARM_BUTTON and self._js is not None:
                    self.armed = not self.armed
                    self.update_q.put(('joy_arm', self.armed))

            count = pygame.joystick.get_count()
            if count != last_count:
                last_count = count
                if count == 0:
                    self._js = None
                    self.update_q.put(('joy_status', 'no joystick'))
                else:
                    self._js = pygame.joystick.Joystick(0)
                    self._js.init()
                    self.update_q.put(('joy_status',
                                       f'{self._js.get_name()} '
                                       f'({self._js.get_numaxes()} axes, '
                                       f'{self._js.get_numbuttons()} buttons)'))

            if self._js is not None:
                roll     = _axis_to_pwm(self._js.get_axis(AXIS_ROLL),     AXIS_ROLL     in AXIS_INVERT)
                pitch    = _axis_to_pwm(self._js.get_axis(AXIS_PITCH),    AXIS_PITCH    in AXIS_INVERT)
                throttle = _axis_to_pwm(self._js.get_axis(AXIS_THROTTLE), AXIS_THROTTLE in AXIS_INVERT)
                yaw      = _axis_to_pwm(self._js.get_axis(AXIS_YAW),      AXIS_YAW      in AXIS_INVERT)
                aux1     = PWM_MAX if self.armed else PWM_MIN
                pwms = (roll, pitch, throttle, yaw, aux1)
                self.update_q.put(('joy_pwm', pwms))
                if self.engaged:
                    self.link.send_rc_override(pwms)

            dt = time.monotonic() - t0
            if dt < period:
                time.sleep(period - dt)

    def stop(self):
        self.stop_flag.set()


class App:
    def __init__(self, root, conn_str):
        self.root = root
        self.q = queue.Queue()
        self.last_hb_time = None
        self.last_attitude_time = None

        root.title(f'Betaflight MAVLink GCS — {conn_str}')
        root.minsize(400, 0)  # width floor; height auto-fits to content

        pad = {'padx': 8, 'pady': 2}

        link_frame = ttk.LabelFrame(root, text='Link')
        link_frame.pack(fill='x', padx=8, pady=6)
        self.link_state = tk.StringVar(value='waiting…')
        self.hb_age = tk.StringVar(value='--')
        ttk.Label(link_frame, text='Status:').grid(row=0, column=0, sticky='w', **pad)
        ttk.Label(link_frame, textvariable=self.link_state).grid(row=0, column=1, sticky='w', **pad)
        ttk.Label(link_frame, text='HB age:').grid(row=1, column=0, sticky='w', **pad)
        ttk.Label(link_frame, textvariable=self.hb_age).grid(row=1, column=1, sticky='w', **pad)

        fc_frame = ttk.LabelFrame(root, text='Flight controller')
        fc_frame.pack(fill='x', padx=8, pady=6)
        self.armed = tk.StringVar(value='--')
        self.mode = tk.StringVar(value='--')
        self.sys_state = tk.StringVar(value='--')
        ttk.Label(fc_frame, text='Armed:').grid(row=0, column=0, sticky='w', **pad)
        ttk.Label(fc_frame, textvariable=self.armed).grid(row=0, column=1, sticky='w', **pad)
        ttk.Label(fc_frame, text='Custom mode:').grid(row=1, column=0, sticky='w', **pad)
        ttk.Label(fc_frame, textvariable=self.mode).grid(row=1, column=1, sticky='w', **pad)
        ttk.Label(fc_frame, text='System:').grid(row=2, column=0, sticky='w', **pad)
        ttk.Label(fc_frame, textvariable=self.sys_state).grid(row=2, column=1, sticky='w', **pad)

        att_frame = ttk.LabelFrame(root, text='Attitude')
        att_frame.pack(fill='x', padx=8, pady=6)
        self.roll = tk.StringVar(value='--')
        self.pitch = tk.StringVar(value='--')
        self.yaw = tk.StringVar(value='--')
        ttk.Label(att_frame, text='Roll:').grid(row=0, column=0, sticky='w', **pad)
        ttk.Label(att_frame, textvariable=self.roll).grid(row=0, column=1, sticky='w', **pad)
        ttk.Label(att_frame, text='Pitch:').grid(row=1, column=0, sticky='w', **pad)
        ttk.Label(att_frame, textvariable=self.pitch).grid(row=1, column=1, sticky='w', **pad)
        ttk.Label(att_frame, text='Yaw:').grid(row=2, column=0, sticky='w', **pad)
        ttk.Label(att_frame, textvariable=self.yaw).grid(row=2, column=1, sticky='w', **pad)

        bat_frame = ttk.LabelFrame(root, text='Battery')
        bat_frame.pack(fill='x', padx=8, pady=6)
        self.voltage = tk.StringVar(value='--')
        ttk.Label(bat_frame, text='Voltage:').grid(row=0, column=0, sticky='w', **pad)
        ttk.Label(bat_frame, textvariable=self.voltage).grid(row=0, column=1, sticky='w', **pad)

        joy_frame = ttk.LabelFrame(root, text='Joystick → RC override')
        joy_frame.pack(fill='x', padx=8, pady=6)
        self.joy_status = tk.StringVar(value='waiting…')
        self.engaged_var = tk.BooleanVar(value=False)
        self.arm_state = tk.StringVar(value='disarmed')
        self.pwm_strs = {k: tk.StringVar(value='--')
                         for k in ('Roll', 'Pitch', 'Throttle', 'Yaw', 'AUX1')}

        ttk.Label(joy_frame, text='Device:').grid(row=0, column=0, sticky='w', **pad)
        ttk.Label(joy_frame, textvariable=self.joy_status).grid(
            row=0, column=1, columnspan=3, sticky='w', **pad)
        ttk.Checkbutton(joy_frame, text='Engage', variable=self.engaged_var,
                        command=self._on_engage).grid(row=1, column=0, columnspan=2, sticky='w', **pad)
        ttk.Label(joy_frame, text='AUX1:').grid(row=1, column=2, sticky='e', **pad)
        ttk.Label(joy_frame, textvariable=self.arm_state).grid(row=1, column=3, sticky='w', **pad)
        for i, k in enumerate(('Roll', 'Pitch', 'Throttle', 'Yaw', 'AUX1')):
            ttk.Label(joy_frame, text=f'{k}:').grid(row=2 + i, column=0, sticky='w', **pad)
            ttk.Label(joy_frame, textvariable=self.pwm_strs[k]).grid(
                row=2 + i, column=1, sticky='w', **pad)

        self.link = MavlinkLink(conn_str, self.q)
        self.link.start()
        self.joy = Joystick(self.link, self.q)
        self.joy.start()

        root.protocol('WM_DELETE_WINDOW', self._on_close)
        root.after(GUI_POLL_MS, self._pump)

    def _on_engage(self):
        self.joy.set_engaged(self.engaged_var.get())

    def _pump(self):
        try:
            while True:
                kind, payload = self.q.get_nowait()
                self._handle(kind, payload)
        except queue.Empty:
            pass

        # Refresh the HB age readout regardless of new messages so it visibly counts up.
        if self.last_hb_time is None:
            self.link_state.set('waiting…')
            self.hb_age.set('--')
        else:
            age = time.monotonic() - self.last_hb_time
            self.hb_age.set(f'{age:4.1f} s')
            self.link_state.set('CONNECTED' if age < 3.0 else 'STALE')

        self.root.after(GUI_POLL_MS, self._pump)

    def _handle(self, kind, m):
        if kind == 'HEARTBEAT':
            self.last_hb_time = time.monotonic()
            base = m.get('base_mode', 0)
            armed = bool(base & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            self.armed.set('ARMED' if armed else 'disarmed')
            self.mode.set(str(m.get('custom_mode', 0)))
            self.sys_state.set(_system_status_name(m.get('system_status', 0)))
        elif kind == 'ATTITUDE':
            self.last_attitude_time = time.monotonic()
            self.roll.set(f"{math.degrees(m.get('roll', 0.0)):+7.2f}°")
            self.pitch.set(f"{math.degrees(m.get('pitch', 0.0)):+7.2f}°")
            self.yaw.set(f"{math.degrees(m.get('yaw', 0.0)):+7.2f}°")
        elif kind == 'SYS_STATUS':
            mv = m.get('voltage_battery', 0)  # millivolts; 0xFFFF means unknown
            if mv and mv != 0xFFFF:
                self.voltage.set(f'{mv / 1000.0:5.2f} V')
        elif kind == 'BATTERY_STATUS':
            cells = m.get('voltages', []) or []
            usable = [c for c in cells if c and c != 0xFFFF]
            if usable:
                self.voltage.set(f'{sum(usable) / 1000.0:5.2f} V')
        elif kind == 'joy_status':
            self.joy_status.set(m)
        elif kind == 'joy_pwm':
            for name, v in zip(('Roll', 'Pitch', 'Throttle', 'Yaw', 'AUX1'), m):
                self.pwm_strs[name].set(f'{v:4d}')
        elif kind == 'joy_arm':
            self.arm_state.set('ARMED' if m else 'disarmed')
        elif kind == 'error':
            print(f'[link] error: {m}')
        elif kind == 'info':
            print(f'[link] {m}')

    def _on_close(self):
        self.joy.stop()
        self.link.stop()
        self.root.destroy()


def _system_status_name(v):
    names = {
        0: 'UNINIT', 1: 'BOOT', 2: 'CALIB', 3: 'STANDBY',
        4: 'ACTIVE', 5: 'CRITICAL', 6: 'EMERGENCY', 7: 'POWEROFF',
        8: 'TERMINATE',
    }
    return names.get(v, str(v))


def main():
    ap = argparse.ArgumentParser(description='Minimal Tkinter MAVLink GCS for Betaflight/VOXL.')
    ap.add_argument('--target', default='127.0.0.1:14550',
                    help='VOXL bridge UDP endpoint (default: 127.0.0.1:14550)')
    args = ap.parse_args()

    conn = parse_target(args.target)
    root = tk.Tk()
    App(root, conn)
    root.mainloop()


if __name__ == '__main__':
    main()
