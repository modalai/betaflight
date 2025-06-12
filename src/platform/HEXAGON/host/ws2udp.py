#!/usr/bin/env python

"""Echo server using the asyncio API."""

import sys
import socket
import threading
import asyncio
from websockets.asyncio.server import serve
import websockets
import logging

_sock = ''
_websoc = ''
_target = ''

async def receive_data():
    global _sock
    global _websoc
    _sock.setblocking(False)
    loop = asyncio.get_event_loop()
    while True:
        try:
            # data, addr = _sock.recvfrom(1024)  # Buffer size is 1024 bytes
            data = await loop.sock_recv(_sock, 2048)

            print(f"Received {len(data)} bytes: {data}")
            await _websoc.send(data)
        except Exception as e:
            print(f"Error receiving data: {e}")
            break


async def handle_client(websocket):
    global _sock
    global _websoc
    global _target

    _websoc = websocket

    try:
        async for message in websocket:
            if isinstance(message, bytes):
                # Process binary data
                print(f"Received binary message: {message}")

                _sock.sendto(message, _target)
            else:
                print(f"Received (non-binary) message: {message}")

    except websockets.exceptions.ConnectionClosed as e:
        print(f"Connection closed: {e}")


async def main():
    asyncio.create_task(receive_data())

    async with websockets.serve(handle_client, "localhost", 8765, subprotocols = ["binary", "wsSerial"]):
        print("WebSocket server started at ws://localhost:8765")
        await asyncio.Future()  # Run forever


if __name__ == "__main__":

    if len(sys.argv) != 3:
        print(f"Usage: python {sys.argv[0]} <IP> <PORT>")
        sys.exit(1)

    ip_address = sys.argv[1]
    try:
        port_number = int(sys.argv[2])
    except ValueError:
        print("Port must be an integer.")
        sys.exit(1)

    _target = (ip_address, port_number)

    # Create a UDP socket
    _sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Start the listener thread
    # listener_thread = threading.Thread(target=receive_data, args=(_sock,), daemon=True)
    # listener_thread.start()

    # Setup websocket logging
    # logging.basicConfig(
    #     format="%(asctime)s %(message)s",
    #     level=logging.DEBUG,
    # )
    # logger = logging.getLogger("websockets")
    # logger.setLevel(logging.DEBUG)
    # logger.addHandler(logging.StreamHandler())

    asyncio.run(main())







# import asyncio
# import socket
# import websockets
# 
# # Configuration
# UDP_LISTEN_IP = "0.0.0.0"
# UDP_LISTEN_PORT = 9999
# 
# WEBSOCKET_PORT = 8765
# 
# # Where to send data received from WebSocket clients (optional)
# UDP_FORWARD_IP = "127.0.0.1"
# UDP_FORWARD_PORT = 10000
# 
# connected_clients = set()
# 
# async def udp_listener():
#     loop = asyncio.get_event_loop()
#     udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#     udp_sock.bind((UDP_LISTEN_IP, UDP_LISTEN_PORT))
#     udp_sock.setblocking(False)
#     print(f"UDP listener running on {UDP_LISTEN_IP}:{UDP_LISTEN_PORT}")
# 
#     while True:
#         data, addr = await loop.sock_recvfrom(udp_sock, 2048)
#         print(f"UDP packet from {addr}: {data}")
#         await broadcast_to_websockets(data)
# 
# async def broadcast_to_websockets(data):
#     if connected_clients:
#         disconnected = set()
#         for ws in connected_clients:
#             try:
#                 await ws.send(data.decode(errors="ignore"))
#             except Exception as e:
#                 print(f"WebSocket send error: {e}")
#                 disconnected.add(ws)
#         connected_clients.difference_update(disconnected)
# 
# async def websocket_handler(websocket, path):
#     print("WebSocket client connected")
#     connected_clients.add(websocket)
#     try:
#         async for message in websocket:
#             print(f"Received from WebSocket: {message}")
#             send_to_udp(message.encode())
#     except websockets.exceptions.ConnectionClosed:
#         pass
#     finally:
#         connected_clients.remove(websocket)
#         print("WebSocket client disconnected")
# 
# def send_to_udp(data):
#     try:
#         udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#         udp_sock.sendto(data, (UDP_FORWARD_IP, UDP_FORWARD_PORT))
#         udp_sock.close()
#         print(f"Forwarded to UDP {UDP_FORWARD_IP}:{UDP_FORWARD_PORT}: {data}")
#     except Exception as e:
#         print(f"Error sending to UDP: {e}")
# 
# async def main():
#     # Start the UDP listener
#     asyncio.create_task(udp_listener())
# 
#     # Start WebSocket server
#     async with websockets.serve(websocket_handler, "localhost", WEBSOCKET_PORT):
#         print(f"WebSocket server running on ws://localhost:{WEBSOCKET_PORT}")
#         await asyncio.Future()  # Keep running
# 
# if __name__ == "__main__":
#     asyncio.run(main())












# # UDP settings
# UDP_IP = "127.0.0.1"
# UDP_PORT = 5005
# # WebSocket settings
# WEBSOCKET_PORT = 8765
# 
# async def udp_to_websocket(websocket):
#     """Receives UDP packets and sends them to the WebSocket."""
#     with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
#         sock.bind((UDP_IP, UDP_PORT))
#         print(f"Listening for UDP on {UDP_IP}:{UDP_PORT}")
#         try:
#             while True:
#                 data, addr = sock.recvfrom(65507) # Maximum possible UDP datagram size
#                 print(f"Received UDP data from {addr}: {data}")
#                 try:
#                   await websocket.send(data)
#                 except websockets.exceptions.ConnectionClosedError:
#                   print("Websocket connection closed")
#                   break
# 
#         except Exception as e:
#             print(f"Error in UDP handling: {e}")
# 
# async def websocket_to_udp(websocket, path):
#     """Receives WebSocket messages and sends them as UDP packets."""
#     print("Client connected")
#     try:
#         async for message in websocket:
#             print(f"Received WebSocket message: {message}")
#             with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
#                 sock.sendto(message, (UDP_IP, UDP_PORT))
#                 print(f"Sent to UDP: {message}")
#     except Exception as e:
#         print(f"Error in WebSocket handling: {e}")
#     finally:
#         print("Client disconnected")
# 
# async def main():
#     """Sets up and runs both UDP and WebSocket tasks concurrently."""
#     async with websockets.serve(websocket_to_udp, "localhost", WEBSOCKET_PORT):
#         print(f"WebSocket server started on port {WEBSOCKET_PORT}")
#         await asyncio.Future()  # Run forever
# 
# if __name__ == "__main__":
#     asyncio.run(main())