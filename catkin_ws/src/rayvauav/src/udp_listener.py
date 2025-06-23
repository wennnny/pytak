#!/usr/bin/env python3
import socket
import struct

UDP_IP = "0.0.0.0"
UDP_PORT = 49152  # 注意：若 CAN 用不同 port，請開另一個 socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for UDP packets on {UDP_IP}:{UDP_PORT}...")

def parse_gps_packet(data):
    try:
        header, length, seq, stamp, lat, lon, endcode = struct.unpack('<BBIdddB', data)
        print(f"[GPS] Seq: {seq}, Time: {stamp:.3f}, Lat: {lat:.6f}, Lon: {lon:.6f}")
    except struct.error as e:
        print(f"[GPS] Unpack error: {e}")

def parse_vel_packet(data):
    try:
        header, length, seq, linear_x, endcode = struct.unpack('<BBIdB', data)
        print(f"[VEL] Seq: {seq}, Linear X: {linear_x:.3f}")
    except struct.error as e:
        print(f"[VEL] Unpack error: {e}")

def parse_can_packet(data):
    try:
        header, length, b6, b7, endcode = struct.unpack('<BB2B B', data)
        print(f"[CAN] Throttle (b6): {b6}, Steering (b7): {b7}")
    except struct.error as e:
        print(f"[CAN] Unpack error: {e}")

def parse_hdg_packet(data):
    try:
        header, length, heading, endcode = struct.unpack('<BBdB', data)
        print(f"[HDG] Compass Heading: {heading:.2f}°")
    except struct.error as e:
        print(f"[HDG] Unpack error: {e}")

def parse_packet(data):
    if len(data) == 31 and data[0] == 0xAA:
        parse_gps_packet(data)
    elif len(data) == 15 and data[0] == 0xAB:
        parse_vel_packet(data)
    elif len(data) == 5 and data[0] == 0xAC:
        parse_can_packet(data)
    elif len(data) == 11 and data[0] == 0xAD:
        parse_hdg_packet(data)
    else:
        print(f"Unknown or invalid packet: {data.hex()}")

try:
    while True:
        data, addr = sock.recvfrom(1024)
        parse_packet(data)
except KeyboardInterrupt:
    print("\n Interrupted by user. Exiting gracefully...")
finally:
    sock.close()
    print("Socket closed.")


# 資料	    Header	 Port	 Payload
# GPS	    0xAA	49152	seq + timestamp + lat/lon
# Velocity	0xAB	49152	seq + linear.x
# CAN	    0xAC	49152	b6 + b7
# Compass	0xAD	49152	float64 (heading)
