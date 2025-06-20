#!/usr/bin/env python3

import socket
import struct

UDP_IP = "0.0.0.0"
UDP_PORT = 49152

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for UDP packets on {UDP_IP}:{UDP_PORT}...")

def parse_packet(data):
    if len(data) != 31:
        print(f"Invalid packet length: {len(data)} bytes")
        return

    try:
        header, length, seq, stamp, lat, lon, endcode = struct.unpack('<BBIdddB', data)

        if length != 28:
            print(f"Unexpected payload length: {length}")
            return

        print(f"Received packet -Seq: {seq}, Time: {stamp:.3f}, Lat: {lat:.6f}, Lon: {lon:.6f}")
        
    except struct.error as e:
        print(f"Error unpacking packet: {e}")

try:
    while True:
        data, addr = sock.recvfrom(1024)
        parse_packet(data)
except KeyboardInterrupt:
    print("\n Interrupted by user. Exiting gracefully...")
finally:
    sock.close()
    print("Socket closed.")
