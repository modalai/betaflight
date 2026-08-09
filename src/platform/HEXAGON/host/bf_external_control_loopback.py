#!/usr/bin/env python3
"""SET_ATTITUDE_TARGET bench loopback for Betaflight on VOXL.

Subscribe to encoded MAVLink bytes from the ``betaflight_mavlink`` MPA pipe,
turn each ATTITUDE_TARGET into a SET_ATTITUDE_TARGET, and send it back through
the pipe's control FIFO. This version has no pymavlink dependency.

This is a bench validation tool for external control. Remove propellers before
selecting EXTERNAL CTRL; its overrides become live whenever the vehicle is armed.
"""

import argparse
import ctypes
import math
import struct
import threading
import time


PIPE_CHANNEL = 0
CLIENT_FLAG_EN_SIMPLE_HELPER = 1 << 0

MAVLINK_V1_STX = 0xFE
MAVLINK_V2_STX = 0xFD
MAVLINK_V2_SIGNED = 1 << 0

MAVLINK_MSG_ID_COMMAND_LONG = 76
MAVLINK_MSG_ID_SET_ATTITUDE_TARGET = 82
MAVLINK_MSG_ID_ATTITUDE_TARGET = 83
MAV_CMD_SET_MESSAGE_INTERVAL = 511

CRC_EXTRA_COMMAND_LONG = 152
CRC_EXTRA_SET_ATTITUDE_TARGET = 49
CRC_EXTRA_ATTITUDE_TARGET = 22

ATTITUDE_TARGET_PAYLOAD_LEN = 37


def quaternion_multiply(lhs, rhs):
    """Hamilton product for [w, x, y, z] quaternions."""
    lw, lx, ly, lz = lhs
    rw, rx, ry, rz = rhs
    return (
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    )


def quaternion_from_rpy_degrees(roll, pitch, yaw):
    """Return a body-FLU quaternion from intrinsic roll/pitch/yaw offsets."""
    half_roll = math.radians(roll) * 0.5
    half_pitch = math.radians(pitch) * 0.5
    half_yaw = math.radians(yaw) * 0.5
    cr, sr = math.cos(half_roll), math.sin(half_roll)
    cp, sp = math.cos(half_pitch), math.sin(half_pitch)
    cy, sy = math.cos(half_yaw), math.sin(half_yaw)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def quaternion_normalize(quaternion):
    norm = math.sqrt(sum(value * value for value in quaternion))
    return tuple(value / norm for value in quaternion)


def x25_crc(data, extra):
    """Return the MAVLink X.25 checksum for header/payload bytes plus CRC extra."""
    crc = 0xFFFF
    for byte in data + bytes([extra]):
        tmp = byte ^ (crc & 0xFF)
        tmp ^= (tmp << 4) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


class MavlinkFrame:
    def __init__(self, msgid, payload, checksum_data, checksum):
        self.msgid = msgid
        self.payload = payload
        self.checksum_data = checksum_data
        self.checksum = checksum


class MavlinkStreamParser:
    """Small MAVLink 1/2 stream framer; message-specific CRC is checked later."""

    def __init__(self):
        self.buffer = bytearray()
        self.discarded_bytes = 0

    def feed(self, data):
        self.buffer.extend(data)
        frames = []

        while self.buffer:
            start = next(
                (i for i, value in enumerate(self.buffer)
                 if value in (MAVLINK_V1_STX, MAVLINK_V2_STX)),
                None,
            )
            if start is None:
                self.discarded_bytes += len(self.buffer)
                self.buffer.clear()
                break
            if start:
                self.discarded_bytes += start
                del self.buffer[:start]

            stx = self.buffer[0]
            header_len = 10 if stx == MAVLINK_V2_STX else 6
            if len(self.buffer) < header_len:
                break

            payload_len = self.buffer[1]
            signature_len = 13 if (
                stx == MAVLINK_V2_STX and self.buffer[2] & MAVLINK_V2_SIGNED
            ) else 0
            frame_len = header_len + payload_len + 2 + signature_len
            if len(self.buffer) < frame_len:
                break

            frame = bytes(self.buffer[:frame_len])
            del self.buffer[:frame_len]

            if stx == MAVLINK_V2_STX:
                msgid = int.from_bytes(frame[7:10], "little")
                payload_offset = 10
            else:
                msgid = frame[5]
                payload_offset = 6

            checksum_offset = payload_offset + payload_len
            frames.append(MavlinkFrame(
                msgid,
                frame[payload_offset:checksum_offset],
                frame[1:checksum_offset],
                int.from_bytes(frame[checksum_offset:checksum_offset + 2], "little"),
            ))

        return frames


class Loopback:
    def __init__(self, args):
        self.args = args
        self.pipe = ctypes.CDLL("libmodal_pipe.so")
        self.parser = MavlinkStreamParser()
        self.sequence = 0
        self.lock = threading.Lock()
        if args.roll_offset or args.pitch_offset or args.yaw_offset:
            offset_bf = quaternion_from_rpy_degrees(
                args.roll_offset, args.pitch_offset, args.yaw_offset,
            )
            # Betaflight's body-FLU/earth-NWU to MAVLink body-FRD/earth-NED
            # conversion is [w, x, y, z] -> [w, x, -y, -z]. Applying the
            # converted offset on the right makes these CLI offsets appear in
            # Betaflight's native axes after the firmware converts them back.
            self.attitude_offset_mavlink = (
                offset_bf[0], offset_bf[1], -offset_bf[2], -offset_bf[3],
            )
        else:
            self.attitude_offset_mavlink = None
        self.stats = {
            "rx_bytes": 0,
            "rx_frames": 0,
            "attitude_targets": 0,
            "echoed": 0,
            "bad_crc": 0,
            "tx_errors": 0,
            "interval_requests": 0,
        }

        callback_type = ctypes.CFUNCTYPE(
            None, ctypes.c_int, ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p,
        )
        self.pipe.pipe_client_set_simple_helper_cb.argtypes = [
            ctypes.c_int, callback_type, ctypes.c_void_p,
        ]
        self.pipe.pipe_client_set_simple_helper_cb.restype = ctypes.c_int
        self.pipe.pipe_client_open.argtypes = [
            ctypes.c_int, ctypes.c_char_p, ctypes.c_char_p,
            ctypes.c_int, ctypes.c_int,
        ]
        self.pipe.pipe_client_open.restype = ctypes.c_int
        self.pipe.pipe_client_send_control_cmd_bytes.argtypes = [
            ctypes.c_int, ctypes.c_void_p, ctypes.c_int,
        ]
        self.pipe.pipe_client_send_control_cmd_bytes.restype = ctypes.c_int
        self.pipe.pipe_client_close.argtypes = [ctypes.c_int]
        self.pipe.pipe_client_close.restype = None

        self.data_callback = callback_type(self._on_data)

    def _next_sequence(self):
        sequence = self.sequence
        self.sequence = (self.sequence + 1) & 0xFF
        return sequence

    def _pack_v2(self, msgid, payload, crc_extra):
        header = bytes([
            len(payload),
            0,  # incompatibility flags
            0,  # compatibility flags
            self._next_sequence(),
            self.args.source_system,
            self.args.source_component,
        ]) + msgid.to_bytes(3, "little")
        checksum = x25_crc(header + payload, crc_extra)
        return bytes([MAVLINK_V2_STX]) + header + payload + struct.pack("<H", checksum)

    def _send(self, frame):
        buffer = (ctypes.c_ubyte * len(frame)).from_buffer_copy(frame)
        result = self.pipe.pipe_client_send_control_cmd_bytes(
            PIPE_CHANNEL, buffer, len(frame),
        )
        return result == 0

    def request_output_rate(self):
        interval_us = 1_000_000.0 / self.args.rate_hz
        payload = struct.pack(
            "<7fHBBB",
            float(MAVLINK_MSG_ID_ATTITUDE_TARGET),
            interval_us,
            0.0, 0.0, 0.0, 0.0, 0.0,
            MAV_CMD_SET_MESSAGE_INTERVAL,
            self.args.target_system,
            self.args.target_component,
            0,
        )
        frame = self._pack_v2(MAVLINK_MSG_ID_COMMAND_LONG, payload, CRC_EXTRA_COMMAND_LONG)
        with self.lock:
            if self._send(frame):
                self.stats["interval_requests"] += 1
            else:
                self.stats["tx_errors"] += 1

    def _echo_attitude_target(self, mavlink_frame):
        if len(mavlink_frame.payload) < ATTITUDE_TARGET_PAYLOAD_LEN:
            return
        expected_crc = x25_crc(
            mavlink_frame.checksum_data,
            CRC_EXTRA_ATTITUDE_TARGET,
        )
        if mavlink_frame.checksum != expected_crc:
            self.stats["bad_crc"] += 1
            return

        # ATTITUDE_TARGET and SET_ATTITUDE_TARGET share their first 36 payload
        # bytes. Optional bench-only overrides exercise the controller path.
        type_mask = mavlink_frame.payload[36]
        common_payload = bytearray(mavlink_frame.payload[:36])
        if self.attitude_offset_mavlink is not None:
            measured_attitude = struct.unpack_from("<4f", common_payload, 4)
            desired_attitude = quaternion_normalize(quaternion_multiply(
                measured_attitude, self.attitude_offset_mavlink,
            ))
            struct.pack_into("<4f", common_payload, 4, *desired_attitude)
        if self.args.yaw_rate is not None:
            struct.pack_into("<f", common_payload, 28, self.args.yaw_rate)
        if self.args.thrust is not None:
            struct.pack_into("<f", common_payload, 32, self.args.thrust)

        # Replace ATTITUDE_TARGET's type-mask byte with target IDs followed by
        # that same mask. The zero-valued thrust_body MAVLink 2 extension is
        # omitted, yielding SET_ATTITUDE_TARGET's valid 39-byte minimum length.
        payload = bytes(common_payload) + bytes([
            self.args.target_system,
            self.args.target_component,
            type_mask,
        ])
        frame = self._pack_v2(
            MAVLINK_MSG_ID_SET_ATTITUDE_TARGET,
            payload,
            CRC_EXTRA_SET_ATTITUDE_TARGET,
        )
        if self._send(frame):
            self.stats["echoed"] += 1
        else:
            self.stats["tx_errors"] += 1

    def _on_data(self, channel, data, length, context):
        if channel != PIPE_CHANNEL or length <= 0:
            return
        chunk = ctypes.string_at(data, length)
        with self.lock:
            self.stats["rx_bytes"] += length
            for frame in self.parser.feed(chunk):
                self.stats["rx_frames"] += 1
                if frame.msgid == MAVLINK_MSG_ID_ATTITUDE_TARGET:
                    self.stats["attitude_targets"] += 1
                    self._echo_attitude_target(frame)

    def open(self):
        if self.pipe.pipe_client_set_simple_helper_cb(
            PIPE_CHANNEL, self.data_callback, None,
        ) != 0:
            raise RuntimeError("could not install MPA data callback")

        result = self.pipe.pipe_client_open(
            PIPE_CHANNEL,
            self.args.pipe.encode(),
            b"bf-ext-loopback",
            CLIENT_FLAG_EN_SIMPLE_HELPER,
            64 * 1024,
        )
        if result != 0:
            raise RuntimeError(f"could not open MPA pipe {self.args.pipe!r}: {result}")

    def close(self):
        self.pipe.pipe_client_close(PIPE_CHANNEL)

    def snapshot_stats(self):
        with self.lock:
            snapshot = dict(self.stats)
            snapshot["discarded_bytes"] = self.parser.discarded_bytes
            return snapshot


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pipe", default="betaflight_mavlink")
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--target-system", type=int, default=1)
    parser.add_argument("--target-component", type=int, default=1)
    parser.add_argument("--source-system", type=int, default=245)
    parser.add_argument("--source-component", type=int, default=191)
    parser.add_argument("--thrust", type=float, default=None,
                        help="bench-only scalar thrust override in [0, 1]")
    parser.add_argument("--yaw-rate", type=float, default=None,
                        help="bench-only MAVLink FRD yaw-rate override in rad/s")
    parser.add_argument("--roll-offset", type=float, default=0.0,
                        help="bench-only Betaflight body-FLU roll offset in degrees")
    parser.add_argument("--pitch-offset", type=float, default=0.0,
                        help="bench-only Betaflight body-FLU pitch offset in degrees")
    parser.add_argument("--yaw-offset", type=float, default=0.0,
                        help="bench-only Betaflight body-FLU yaw offset in degrees")
    args = parser.parse_args()

    if not 0.1 <= args.rate_hz <= 50.0:
        parser.error("--rate-hz must be between 0.1 and 50")
    for name in ("target_system", "target_component", "source_system", "source_component"):
        if not 0 <= getattr(args, name) <= 255:
            parser.error(f"--{name.replace('_', '-')} must be between 0 and 255")
    if args.thrust is not None and not 0.0 <= args.thrust <= 1.0:
        parser.error("--thrust must be between 0 and 1")
    for name in ("roll_offset", "pitch_offset", "yaw_offset"):
        if not -180.0 <= getattr(args, name) <= 180.0:
            parser.error(f"--{name.replace('_', '-')} must be between -180 and 180 degrees")
    return args


def main():
    args = parse_args()
    loopback = Loopback(args)
    loopback.open()
    print(f"Connected to {args.pipe}; requesting ATTITUDE_TARGET at {args.rate_hz:g} Hz")
    print("BENCH ONLY: remove propellers before selecting EXTERNAL CTRL")
    if args.thrust is not None or args.yaw_rate is not None:
        print(f"Overrides: thrust={args.thrust!r}, MAVLink yaw_rate={args.yaw_rate!r} rad/s")
    if args.roll_offset or args.pitch_offset or args.yaw_offset:
        print(
            "Betaflight attitude offsets: "
            f"roll={args.roll_offset:g} pitch={args.pitch_offset:g} "
            f"yaw={args.yaw_offset:g} deg"
        )

    last = None
    next_rate_request = 0.0
    try:
        while True:
            now = time.monotonic()
            snapshot = loopback.snapshot_stats()
            if snapshot["attitude_targets"] == 0 and now >= next_rate_request:
                loopback.request_output_rate()
                next_rate_request = now + 2.0

            if snapshot != last:
                print(
                    f"rx={snapshot['rx_frames']}f/{snapshot['rx_bytes']}B "
                    f"targets={snapshot['attitude_targets']} echoed={snapshot['echoed']} "
                    f"bad_crc={snapshot['bad_crc']} tx_errors={snapshot['tx_errors']} "
                    f"discarded={snapshot['discarded_bytes']}"
                )
                last = snapshot
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        loopback.close()


if __name__ == "__main__":
    main()
