#!/usr/bin/env python

"""
Bridges between websocket connection from betaflight configurator to
the betaflight tcp socket to exchange MSP data over adb.
1. betaflight on target uses port 8765
2. On host, use 'adb forward tcp:8766 tcp:8765' to forward local port 8766
   to port 8765 on target.
3. This script will attempt to connect to local port 8766 and present port 8767
   for the websocket to be connected to by betaflight configurator
4. In betaflight configurator, attempt connection to ws://localhost:8767
"""

import sys
import socket
import threading
import asyncio
from websockets.asyncio.server import serve
import websockets
import logging

_reader = None
_writer = None
_websoc = None
_target_addr = None
_target_port = None

async def receive_data():
    global _reader
    global _websoc
    while True:
        try:
            # data, addr = _sock.recvfrom(1024)  # Buffer size is 1024 bytes
            data = await _reader.read(2048)

            print(f"Received {len(data)} bytes: {data}")
            await _websoc.send(data)
        except Exception as e:
            print(f"Error receiving data: {e}")
            break


async def handle_client(websocket):
    global _websoc
    global _writer

    _websoc = websocket

    try:
        async for message in websocket:
            if isinstance(message, bytes):
                # Process binary data
                print(f"Received binary message: {message}")

                # _sock.sendall(message, _target)
                _writer.write(message)
                await _writer.drain()
            else:
                print(f"Received (non-binary) message: {message}")

    except websockets.exceptions.ConnectionClosed as e:
        print(f"Connection closed: {e}")



async def main():
    global _target_addr
    global _target_port
    global _reader
    global _writer

    # Create a TCP socket
    _reader, _writer = await asyncio.open_connection(_target_addr, _target_port)

    asyncio.create_task(receive_data())

    async with websockets.serve(handle_client, "localhost", 8767, subprotocols = ["binary", "wsSerial"]):
        print("WebSocket server started at ws://localhost:8767")
        await asyncio.Future()  # Run forever


if __name__ == "__main__":
    # global _target_addr
    # global _target_port

    if len(sys.argv) != 3:
        print(f"Usage: python {sys.argv[0]} <IP> <PORT>")
        sys.exit(1)

    _target_addr = sys.argv[1]
    try:
        _target_port = int(sys.argv[2])
    except ValueError:
        print("Port must be an integer.")
        sys.exit(1)

    asyncio.run(main())
