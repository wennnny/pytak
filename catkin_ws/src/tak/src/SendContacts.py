#!/usr/bin/env python3

import asyncio, uuid, socket, platform
import xml.etree.ElementTree as ET
import pytak
import sys
import rospy
from sensor_msgs.msg import NavSatFix
from configparser import ConfigParser

DEVICE_CALLSIGN = socket.gethostname()
DEVICE_UID = str(uuid.uuid4())
DEVICE_OS = platform.system()
SYNC_IP = "127.0.0.1"
SYNC_PORT = 5005
sync_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

latest_gps = {"lat": None, "lon": None, "hae": None}

for arg in sys.argv:
    if arg.startswith("callsign:="):
        DEVICE_CALLSIGN = arg.split(":=")[1]
    if arg.startswith("uid:="):
        DEVICE_UID = arg.split(":=")[1]

def gps_callback(data):
    global latest_gps
    latest_gps["lat"] = data.latitude
    latest_gps["lon"] = data.longitude
    latest_gps["hae"] = data.altitude

rospy.init_node('cot_sender', anonymous=True)
rospy.Subscriber("/mavros/global_position/global", NavSatFix, gps_callback)

def generate_gps_cot():
    """Generate a simple takPong CoT Event."""
    root = ET.Element("event")
    root.set("version", "2.0")
    root.set("type", "b-t-f")
    root.set("uid",  DEVICE_UID)
    root.set("how", "m-g-i-g-o")
    root.set("time", pytak.cot_time())
    root.set("start", pytak.cot_time())
    root.set("stale", pytak.cot_time(300))

    # lat, lon = get_gps()
    gps_data = {
        "lat": "0.0",
        "lon": "0.0",
        "hae": "999999.0",
        "ce": "999999.0",
        "le": "999999.0",
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
    ET.SubElement(detail, "remarks", attrib={
        "time": pytak.cot_time(),
        "source": DEVICE_UID,
        "to": "All Chat Rooms"
    }).text = "!"

    return ET.tostring(root)


class MySerializer(pytak.QueueWorker):
    """
    Defines how you process or generate your Cursor on Target Events.
    From there it adds the CoT Events to a queue for TX to a COT_URL.
    """

    async def handle_data(self, data):
        """Handle pre-CoT data, serialize to CoT Event, then puts on queue."""
        event = data
        await self.put_queue(event)

    async def run(self):
        """Run the loop for processing or generating pre-CoT data."""
        while not rospy.is_shutdown():
            data = generate_gps_cot()
            self._logger.info("Sending:\n%s\n", data.decode())
            await self.handle_data(data)
            await asyncio.sleep(1)

class MyReceiver(pytak.QueueWorker):
    """Handle incoming CoT events."""

    async def handle_data(self, data):
        self._logger.info("[Received] CoT Event:\n%s\n", data.decode())
        sync_socket.sendto(data, (SYNC_IP, SYNC_PORT))

    async def run(self):
        while True:
            data = await self.queue.get()
            await self.handle_data(data)


async def main():
    """Main definition of your program, sets config params and
    adds your serializer to the asyncio task list.
    """
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

    # Initializes worker queues and tasks.
    clitool = pytak.CLITool(config)
    await clitool.setup()

    # Add your serializer to the asyncio task list.
    clitool.add_tasks(
        set([MySerializer(clitool.tx_queue, config), MyReceiver(clitool.rx_queue, config)])
    )

    await clitool.run()

if __name__ == "__main__":
    asyncio.run(main())


# 2025-07-10 14:44:04,456 pytak INFO - [Received] CoT Event:
# <?xml version="1.0" encoding="UTF-8"?>
# <event version="2.0" uid="GeoChat.eefebd72-8739-7dd4-3e95-b1168a127fda.2fe9ea7f-6795-829a-f02f-e4fb06789042" type="b-t-f" how="h-g-i-g-o" time="2025-07-10T06:44:04Z" start="2025-07-10T06:44:04Z" stale="2025-11-03T00:30:44Z"><point lat="0.0" lon="0.0" hae="999999.0" ce="999999.0" le="999999.0"/>
# <detail><__chat chatroom="All Chat Rooms" senderCallsign="OVERLORD680" parent="RootContactGroup" groupOwner="false" id="All Chat Rooms"><chatgrp id="All Chat Rooms" uid0="eefebd72-8739-7dd4-3e95-b1168a127fda" uid1="All Chat Rooms"/></__chat><remarks time="2025-07-10T06:44:04Z" source="eefebd72-8739-7dd4-3e95-b1168a127fda" to="All Chat Rooms">!</remarks><link relation="p-p" type="a-f-G-U-C-I" uid="eefebd72-8739-7dd4-3e95-b1168a127fda"/><_flow-tags_ TAK-Server-abb9190590884a1d9a606bcfdb67648c="2025-07-10T06:44:04Z"/></detail></event>

