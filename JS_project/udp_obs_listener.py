#!/usr/bin/env python3
import socket
import struct

UDP_IP = "0.0.0.0"
UDP_PORT = 49153

def parse_obs_packet(data):
    if len(data) == 16 and data[0] == 0xAE:
        try:
            _, _, idx, x, y, z, _ = struct.unpack('=BBBfffB', data)
            return idx, x, y, z
        except Exception as e:
            print(f"[ERROR] Failed to parse OBS packet: {e}")
    return None

def parse_goal_packet(data):
    if len(data) == 16 and data[0] == 0xAF:
        try:
            _, _, idx, x, y, z, _ = struct.unpack('=BBBfffB', data)
            return idx, x, y, z
        except Exception as e:
            print(f"[ERROR] Failed to parse GOAL packet: {e}")
    return None

def parse_start_packet(data):
    if len(data) == 16 and data[0] == 0xAC:
        try:
            _, _, idx, x, y, z, _ = struct.unpack('=BBBfffB', data)
            return idx, x, y, z
        except Exception as e:
            print(f"[ERROR] Failed to parse START packet: {e}")
    return None

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    print(f"[OBS Client] Listening on {UDP_IP}:{UDP_PORT}")

    while True:
        data, addr = sock.recvfrom(1024)
        obs = parse_obs_packet(data)
        goal = parse_goal_packet(data)
        start = parse_start_packet(data)
        if obs:
            idx, x, y, z = obs
            print(f"[OBS] From {addr} → idx={idx}, x={x:.6f}, y={y:.6f}, z={z:.6f}")
        elif goal:
            idx, x, y, z = goal
            print(f"[GOAL] From {addr} → idx={idx}, x={x:.6f}, y={y:.6f}, z={z:.6f}")
        elif start:
            idx, x, y, z = start
            print(f"[START] From {addr} → idx={idx}, x={x:.6f}, y={y:.6f}, z={z:.6f}")
        else:
            print(f"[IGNORED] {len(data)} bytes from {addr}: {data.hex()}")

if __name__ == "__main__":
    main()
