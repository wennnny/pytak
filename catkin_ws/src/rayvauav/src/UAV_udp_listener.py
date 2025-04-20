#!/usr/bin/env python3

import socket
import struct

UDP_IP = "0.0.0.0"
UDP_PORT = 49152

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for UDP packets on {UDP_IP}:{UDP_PORT}...")

def parse_packet(data):
    if len(data) != 19:
        print(f"Invalid packet length: {len(data)}")
        return

    header = data[0]
    length = data[1]
    if length != 16:
        print(f"Unexpected payload length: {length}")
        return
        
    lat = struct.unpack('<d', data[2:10])[0]
    lon = struct.unpack('<d', data[10:18])[0]
    endcode = data[18]

    print(f"Received packet - Header: {header}, Length: {length}, Lat: {lat:.6f}, Lon: {lon:.6f}, End: {endcode}")

while True:
    data, addr = sock.recvfrom(1024)
    parse_packet(data)

