#!/usr/bin/env python3
import socket, struct
from multiprocessing import Process, Queue
import time

UDP_IP = "0.0.0.0"
UDP_PORT = 49152

def parse_packet(data):
    try:
        if len(data) == 31 and data[0] == 0xAA:
            _, _, seq, stamp, lat, lon, _ = struct.unpack('<BBIdddB', data)
            return "gps", {"seq": seq, "stamp": stamp, "lat": lat, "lon": lon}
        elif len(data) == 15 and data[0] == 0xAB:
            _, _, seq, linear_x, _ = struct.unpack('<BBIdB', data)
            return "vel", {"seq": seq, "linear_x": linear_x}
        elif len(data) == 5 and data[0] == 0xAC:
            _, _, b6, b7, _ = struct.unpack('<BB2BB', data)
            return "can", {"throttle": b6, "steering": b7}
        elif len(data) == 11 and data[0] == 0xAD:
            _, _, heading, _ = struct.unpack('<BBdB', data)
            return "hdg", {"heading": heading}
        elif len(data) == 16 and data[0] == 0xAE:
            _, _, idx, x, y, z, _ = struct.unpack('<BBBfffB', data)
            return "obspos", {"idx": idx, "x": x, "y": y, "z": z}
    except Exception as e:
        return "error", {"error": str(e), "raw": data.hex()}
    return "unknown", {"raw": data.hex()}

def listener_worker(q: Queue):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    print(f"[UDP Listener] Listening on {UDP_IP}:{UDP_PORT}")

    while True:
        data, _ = sock.recvfrom(1024)
        msg_type, parsed = parse_packet(data)
        q.put((msg_type, parsed))  # 非同步寫入共享 Queue




# 資料	    Header	 Port	 Payload
# GPS	    0xAA	49152	seq + timestamp + lat/lon
# Velocity	0xAB	49152	seq + linear.x
# CAN	    0xAC	49152	b6 + b7
# Compass	0xAD	49152	float64 (heading)
# PoseArray 0xAE	49152	index + x + y + z