#!/usr/bin/env python3

import asyncio
import socket
import struct
import xml.etree.ElementTree as ET
from configparser import ConfigParser
import pytak

# === UDP 傳送目標設定 ===
UDP_TARGET_IP = "140.113.148.99"
UDP_TARGET_PORT = 49157
udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# === waypoint 封包格式參數 ===
HEADER = 0xAF
END = 0xC0

waypoint_buffer = []

def parse_cot(xml_data):
    root = ET.fromstring(xml_data)

    event_type = root.get("type", "Unknown")

    point = root.find("point")
    lat = float(point.get("lat", 0)) if point is not None else 0
    lon = float(point.get("lon", 0)) if point is not None else 0
    hae = float(point.get("hae", 0)) if point is not None else 0

    contact = root.find("./detail/contact")
    callsign = contact.get("callsign", "Unknown") if contact is not None else "Unknown"

    return callsign, event_type, lat, lon, hae

def send_waypoint_udp(index, lat, lon, hae):
    LENGTH = 25
    packet = struct.pack('<BBBdddB', HEADER, LENGTH, index, lat, lon, hae, END)
    udp_sock.sendto(packet, (UDP_TARGET_IP, UDP_TARGET_PORT))
    print(f"[UDP] Sent waypoint{index}: lat={lat:.6f}, lon={lon:.6f}, hae={hae:.2f}")

class WaypointReceiver(pytak.QueueWorker):
    """接收 TAK 傳來的 CoT 並解析/轉送成 UDP"""

    async def handle_data(self, data):
        try:
            xml_str = data.decode()
            callsign, event_type, lat, lon, hae = parse_cot(xml_str)

            if event_type != "b-m-p-s-m":
                return

            if callsign.lower().startswith("waypoint"):
                waypoint_buffer.append((lat, lon, hae))

                if callsign.endswith("!"):
                    for idx, (lat, lon, hae) in enumerate(waypoint_buffer):
                        send_waypoint_udp(idx, lat, lon, hae)
                    print(f"[UDP] Published {len(waypoint_buffer)} waypoints via UDP\n")
                    waypoint_buffer.clear()

        except Exception as e:
            print(f"[ERROR] Failed to parse/send CoT: {e}")

    async def run(self):
        while True:
            data = await self.queue.get()
            await self.handle_data(data)

async def main():
    config = ConfigParser()
    config["mycottool"] = {
        "COT_URL": "tls://140.113.148.80:8089",
        "PYTAK_TLS_CLIENT_CERT": "/home/moos-dawg/Downloads/pytak/catkin_ws/files_WAN/argtest2_cert.pem",
        "PYTAK_TLS_CLIENT_KEY": "/home/moos-dawg/Downloads/pytak/catkin_ws/files_WAN/argtest2_key.pem",
        "PYTAK_TLS_CA_CERT": "/home/moos-dawg/Downloads/pytak/catkin_ws/files_WAN/argtest2-trusted.pem",
        "PYTAK_TLS_DONT_CHECK_HOSTNAME": "1",
        "PYTAK_TLS_DONT_VERIFY": "1"
    }
    config = config["mycottool"]

    clitool = pytak.CLITool(config)
    await clitool.setup()

    clitool.add_tasks(set([WaypointReceiver(clitool.rx_queue, config)]))
    await clitool.run()

if __name__ == "__main__":
    asyncio.run(main())
