#!/usr/bin/env python3
import socket, struct
from multiprocessing import Process, Queue
import time

UDP_IP = "0.0.0.0"
UDP_PORT = 49152

def to_seconds(stamp):
    """
    將時間戳推斷為秒：
    - ~1e9 量級：秒 (Unix epoch seconds)
    - ~1e12：毫秒
    - ~1e15：微秒
    - ~1e18：奈秒
    """
    if stamp > 1e16:   # ns
        return stamp / 1e9
    elif stamp > 1e13: # us
        return stamp / 1e6
    elif stamp > 1e11: # ms
        return stamp / 1e3
    else:              # s
        return stamp

def parse_packet(data):
    try:
        if len(data) == 31 and data[0] == 0xAA:
            # header, len, seq(uint32), stamp(double), lat(double), lon(double), end
            _, _, seq, stamp, lat, lon, _ = struct.unpack('<BBIdddB', data)
            return "gps", {"seq": seq, "stamp": stamp, "lat": lat, "lon": lon}

        elif len(data) == 15 and data[0] == 0xAB:
            _, _, seq, linear_x, _ = struct.unpack('<BBIdB', data)
            return "vel", {"seq": seq, "linear_x": linear_x}

        elif len(data) == 9 and data[0] == 0xAC:
            _, _, b2, b3, b4, b5, b6, b7, _ = struct.unpack('<BBBBBBBBB', data)
            driveLine = (b3 & 0xF0) >> 4
            externalControl = b3 & 0x0F
            engineState = (b4 & 0x0C) >> 2
            gear = b4 & 0x03
            return "can", {
                "b2": b2, "driveLine": driveLine, "externalControl": externalControl,
                "engineState": engineState, "gear": gear,
                "dps": b5, "throttle": b6, "steering": b7
            }

        elif len(data) == 5 and data[0] == 0xAC:
            # header, len, throttle, steering, end
            h, l, b6, b7, e = struct.unpack('<BBBBB', data)
            return "can", {"throttle": b6, "steering": b7}

        elif len(data) == 11 and data[0] == 0xAD:
            _, _, heading, _ = struct.unpack('<BBdB', data)
            return "hdg", {"heading": heading}

        elif len(data) == 16 and data[0] == 0xAE:
            _, _, idx = struct.unpack('<BBB', data[:3])
            x = struct.unpack('<f', data[3:7])[0]
            y = struct.unpack('<f', data[7:11])[0]
            z = struct.unpack('<f', data[11:15])[0]
            return "pose", {"index": idx, "x": x, "y": y, "z": z}

    except Exception as e:
        return "error", {"error": str(e), "raw": data.hex()}

    return "unknown", {"raw": data.hex()}

def listener_worker(q: Queue):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    print(f"[UDP Listener] Listening on {UDP_IP}:{UDP_PORT}")

    while True:
        data, addr = sock.recvfrom(1024)
        msg_type, parsed = parse_packet(data)

        # === 在接收端計算 latency（只對帶有 stamp 的 GPS 做） ===
        if msg_type == "gps" and "stamp" in parsed:
            recv_time = time.time()
            sent_s   = to_seconds(parsed["stamp"])
            latency  = recv_time - sent_s  # 可能為負，代表兩端時鐘未完全對齊
            parsed["recv_time"]  = recv_time
            parsed["latency_s"]  = latency
            parsed["latency_ms"] = latency * 1000.0

        q.put((msg_type, parsed))
