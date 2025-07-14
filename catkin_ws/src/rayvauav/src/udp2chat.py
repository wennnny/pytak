#!/usr/bin/env python3
import asyncio, uuid, socket, struct, platform
import xml.etree.ElementTree as ET
import pytak
from configparser import ConfigParser
from collections import defaultdict

# === Device identity ===
DEVICE_CALLSIGN = socket.gethostname()
DEVICE_UID = str(uuid.uuid4())

# === Store the latest parsed messages ===
latest_msgs = defaultdict(str)

# === Generate a CoT event formatted as a chat message ===
def generate_chat_cot(message: str):
    root = ET.Element("event")
    root.set("version", "2.0")
    root.set("type", "b-t-f")
    root.set("uid", f"GeoChat.{DEVICE_UID}.{uuid.uuid4()}")
    root.set("how", "m-g-i-g-o")
    root.set("time", pytak.cot_time())
    root.set("start", pytak.cot_time())
    root.set("stale", pytak.cot_time(300))

    gps_data = {
        "lat": "0.0", "lon": "0.0", "hae": "999999.0", "ce": "999999.0", "le": "999999.0"
    }
    ET.SubElement(root, "point", attrib=gps_data)

    detail = ET.SubElement(root, "detail")
    chat = ET.SubElement(detail, "__chat", {
        "chatroom": "All Chat Rooms",
        "senderCallsign": DEVICE_CALLSIGN,
        "parent": "RootContactGroup",
        "groupOwner": "false",
        "id": "All Chat Rooms"
    })
    ET.SubElement(chat, "chatgrp", {
        "id": "All Chat Rooms",
        "uid0": DEVICE_UID,
        "uid1": "All Chat Rooms"
    })
    ET.SubElement(detail, "remarks", {
        "time": pytak.cot_time(),
        "source": DEVICE_UID,
        "to": "All Chat Rooms"
    }).text = message

    return ET.tostring(root)

# === Parse raw UDP packet and store last values ===
def parse_packet(data):
    try:
        if len(data) == 31 and data[0] == 0xAA:
            header, length, seq, stamp, lat, lon, endcode = struct.unpack('<BBIdddB', data)
            latest_msgs['gps'] = f"[GPS] Seq={seq}, Time={stamp:.3f}, Lat={lat:.6f}, Lon={lon:.6f}"
        elif len(data) == 15 and data[0] == 0xAB:
            header, length, seq, linear_x, endcode = struct.unpack('<BBIdB', data)
            latest_msgs['vel'] = f"[VEL] Seq={seq}, LinearX={linear_x:.3f}"
        elif len(data) == 5 and data[0] == 0xAC:
            header, length, b6, b7, endcode = struct.unpack('<BB2BB', data)
            latest_msgs['can'] = f"[CAN] Throttle={b6}, Steering={b7}"
        elif len(data) == 11 and data[0] == 0xAD:
            header, length, heading, endcode = struct.unpack('<BBdB', data)
            latest_msgs['hdg'] = f"[HDG] Heading={heading:.2f}°"
        elif len(data) == 16 and data[0] == 0xAE:
            header, length, idx, x, y, z, endcode = struct.unpack('<BBBfffB', data)
            latest_msgs['pose'] = f"[POSE] Index={idx}, x={x:.2f}, y={y:.2f}, z={z:.2f}"
        else:
            latest_msgs['unknown'] = f"[UNKNOWN] {data.hex()}"
    except struct.error as e:
        latest_msgs['error'] = f"[UNPACK ERROR] {e}"

# === Background task to receive UDP packets ===
class UDPListener(pytak.QueueWorker):
    def __init__(self, tx_queue, config, latest_msgs):
        super().__init__(tx_queue, config)
        self.latest_msgs = latest_msgs

    async def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("0.0.0.0", 49152))
        print("[UDP] Listening on 0.0.0.0:49152")
        loop = asyncio.get_event_loop()

        while True:
            data, _ = await loop.run_in_executor(None, sock.recvfrom, 1024)
            parse_packet(data)

# === Bundles and sends one chat message per second ===
class BundlerWorker(pytak.QueueWorker):
    def __init__(self, tx_queue, config, latest_msgs):
        super().__init__(tx_queue, config)
        self.latest_msgs = latest_msgs

    async def run(self):
        while True:
            combined_msg = "\n".join([
                self.latest_msgs.get('gps', ''),
                self.latest_msgs.get('can', ''),
                self.latest_msgs.get('hdg', ''),
                self.latest_msgs.get('vel', ''),
                self.latest_msgs.get('pose', '')
            ]).strip()

            if combined_msg:
                print(f"[Bundled] \n{combined_msg}\n")
                cot_msg = generate_chat_cot(combined_msg)
                await self.put_queue(cot_msg)

            await asyncio.sleep(1)

# === Receiver for any incoming CoT messages (optional) ===
class MyReceiver(pytak.QueueWorker):
    async def handle_data(self, data):
        print("[RX CoT]", data.decode())

    async def run(self):
        while True:
            data = await self.queue.get()
            await self.handle_data(data)

# === Main asyncio entry point ===
async def main():
    config = ConfigParser()
    config["mycottool"] = {
        "COT_URL": "tls://140.113.148.80:8089",
        "PYTAK_TLS_CLIENT_CERT": "argtest2_cert.pem",
        "PYTAK_TLS_CLIENT_KEY": "argtest2_key.pem",
        "PYTAK_TLS_CA_CERT": "argtest2-trusted.pem",
        "PYTAK_TLS_DONT_CHECK_HOSTNAME": "1",
        "PYTAK_TLS_DONT_VERIFY": "1"
    }
    config = config["mycottool"]

    clitool = pytak.CLITool(config)
    await clitool.setup()

    clitool.add_tasks({
        UDPListener(clitool.tx_queue, config, latest_msgs),
        BundlerWorker(clitool.tx_queue, config, latest_msgs),
        MyReceiver(clitool.rx_queue, config)
    })

    await clitool.run()

# === Launch the script ===
if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user. Shutting down gracefully.")
