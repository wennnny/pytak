#!/usr/bin/env python3

import asyncio, uuid, socket, platform
import xml.etree.ElementTree as ET
import pytak
from configparser import ConfigParser
from collections import defaultdict
from multiprocessing import Queue, Process
from udp_listener_async_49152 import listener_worker
import json, os

config_path = os.path.expanduser("~/Downloads/pytak/catkin_ws/src/rayvauav/src/cot_config.json")
with open(config_path, "r") as f:
    cot_config = json.load(f)

DEVICE_CALLSIGN = socket.gethostname()
DEVICE_UID = str(uuid.uuid4())
DEVICE_OS = platform.system()
latest_msgs = defaultdict(str)
latest_gps = {"lat": None, "lon": None, "hae": 999999.0}


async def handle_queue_data(q, latest_msgs, tx_queue):
    while True:
        if not q.empty():
            msg_type, data = q.get()
            if msg_type == "gps":
                latest_msgs['gps'] = f"[GPS] Seq={data['seq']}, Time={data['stamp']:.3f}, Lat={data['lat']:.6f}, Lon={data['lon']:.6f}"
                latest_gps['lat'] = data['lat']
                latest_gps['lon'] = data['lon']
                latest_gps['hae'] = 999999.0  # Placeholder HAE
            elif msg_type == "vel":
                latest_msgs['vel'] = f"[VEL] Seq={data['seq']}, LinearX={data['linear_x']:.3f}"
            elif msg_type == "can":
                latest_msgs['can'] = f"[CAN] Throttle={data['throttle']}, Steering={data['steering']}"
            elif msg_type == "hdg":
                latest_msgs['hdg'] = f"[HDG] Heading={data['heading']:.2f}°"
            elif msg_type == "obspos":
                idx, x, y, z = data["idx"], data["x"], data["y"], data["z"]
                latest_msgs[f'obstacle{idx}'] = f"[OBS] idx={idx} x={x:.2f}, y={y:.2f}, z={z:.2f}"
                cot_msg = generate_obstacle_cot(idx, x, y, z)
                await tx_queue.put(cot_msg)
            elif msg_type == "error":
                latest_msgs['error'] = f"[UNPACK ERROR] {data['error']}"
            else:
                latest_msgs['unknown'] = f"[UNKNOWN] {data}"
        await asyncio.sleep(0.01)

# === Generate chat CoT ===
def generate_chat_cot(message: str):
    cfg = cot_config["chat"]
    root = ET.Element("event")
    root.set("version", cfg["event"]["version"])
    root.set("type", cfg["event"]["type"])
    root.set("uid", f"GeoChat.{DEVICE_UID}.{uuid.uuid4()}")
    root.set("how", cfg["event"]["how"])
    root.set("time", pytak.cot_time())
    root.set("start", pytak.cot_time())
    root.set("stale", pytak.cot_time(cfg["event"]["stale_seconds"]))

    ET.SubElement(root, "point", cfg["point"])

    detail = ET.SubElement(root, "detail")
    chat = ET.SubElement(detail, "__chat", {
        "chatroom": cfg["chatroom"]["name"],
        "senderCallsign": DEVICE_CALLSIGN,
        "parent": cfg["chatroom"]["parent"],
        "groupOwner": cfg["chatroom"]["groupOwner"],
        "id": cfg["chatroom"]["name"]
    })
    ET.SubElement(chat, "chatgrp", {
        "id": cfg["chatroom"]["name"],
        "uid0": DEVICE_UID,
        "uid1": cfg["chatroom"]["name"]
    })
    ET.SubElement(detail, "remarks", {
        "time": pytak.cot_time(),
        "source": DEVICE_UID,
        "to": cfg["chatroom"]["name"]
    }).text = message

    return ET.tostring(root)


# === Generate boat GPS member CoT ===
def generate_gps_cot():
    cfg = cot_config["gps"]
    root = ET.Element("event")
    root.set("version", cfg["event"]["version"])
    root.set("type", cfg["event"]["type"])
    root.set("uid", DEVICE_UID)
    root.set("how", cfg["event"]["how"])
    root.set("time", pytak.cot_time())
    root.set("start", pytak.cot_time())
    root.set("stale", pytak.cot_time(cfg["event"]["stale_seconds"]))

    ET.SubElement(root, "point", {
        "lat": str(latest_gps["lat"]),
        "lon": str(latest_gps["lon"]),
        "hae": str(latest_gps["hae"]),
        "ce": "999999",
        "le": "999999"
    })

    detail = ET.SubElement(root, "detail")
    ET.SubElement(detail, "takv", {
        "device": "JS_05",
        "platform": "Python",
        "os": DEVICE_OS
    })
    ET.SubElement(detail, "contact", {"callsign": "JS_05"})
    ET.SubElement(detail, "__group", cfg["detail"]["group"])
    ET.SubElement(detail, "uid", {"Droid": cfg["detail"]["uid"]})
    ET.SubElement(detail, "usericon", {"iconsetpath": cfg["detail"]["iconsetpath"]})

    return ET.tostring(root)

# === Generate obstacle GPS member CoT ===
def generate_obstacle_cot(idx, x, y, z):
    cfg = cot_config["obstacle"]
    root = ET.Element("event")
    root.set("version", cfg["event"]["version"])
    root.set("type", cfg["event"]["type"])
    root.set("uid", f"obstacle-{idx}")
    root.set("how", cfg["event"]["how"])
    root.set("time", pytak.cot_time())
    root.set("start", pytak.cot_time())
    root.set("stale", pytak.cot_time(cfg["event"]["stale_seconds"]))

    ET.SubElement(root, "point", {
        "lat": str(y),
        "lon": str(x),
        "hae": str(z),
        "ce": "999999",
        "le": "999999"
    })

    detail = ET.SubElement(root, "detail")
    ET.SubElement(detail, "takv", {
        "device": DEVICE_CALLSIGN,
        "platform": "Python",
        "os": DEVICE_OS
    })
    ET.SubElement(detail, "contact", {"callsign": f"obstacle{idx}"})
    ET.SubElement(detail, "__group", cfg["detail"]["group"])
    ET.SubElement(detail, "uid", {"Droid": cfg["detail"]["uid"]})
    ET.SubElement(detail, "usericon", {"iconsetpath": cfg["detail"]["iconsetpath"]})
    ET.SubElement(detail, "color", {"argb": cfg["detail"]["color"]})

    return ET.tostring(root)


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
                self.latest_msgs.get('vel', '')
            ]).strip()

            if combined_msg:
                print(f"[Bundled] \n{combined_msg}\n")
                cot_msg = generate_chat_cot(combined_msg)
                await self.put_queue(cot_msg)

            await asyncio.sleep(1)

class GPSMemberWorker(pytak.QueueWorker):
    async def run(self):
        while True:
            if latest_gps["lat"] is not None and latest_gps["lon"] is not None:
                event = generate_gps_cot()
                await self.put_queue(event)
            await asyncio.sleep(1)

class MyReceiver(pytak.QueueWorker):
    async def handle_data(self, data):
        print("[RX CoT]", data.decode())

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

    queue = Queue()
    listener_process = Process(target=listener_worker, args=(queue,))
    listener_process.start()

    parser_task = asyncio.create_task(handle_queue_data(queue, latest_msgs, clitool.tx_queue))

    clitool.add_tasks({
        BundlerWorker(clitool.tx_queue, config, latest_msgs),
        GPSMemberWorker(clitool.tx_queue, config),
        MyReceiver(clitool.rx_queue, config)
    })

    try:
        await asyncio.gather(clitool.run(), parser_task)
    finally:
        listener_process.terminate()
        listener_process.join()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user. Shutting down.")
