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
    if latest_gps["lat"] is None or latest_gps["lon"] is None:
        return None

    root = ET.Element("event")
    root.set("version", "2.0")
    root.set("type", "a-f-G-U-C-I")
    root.set("uid",  DEVICE_UID)
    root.set("how", "m-g")
    root.set("time", pytak.cot_time())
    root.set("start", pytak.cot_time())
    root.set("stale", pytak.cot_time(10))

    gps_data = {
        "lat": str(latest_gps["lat"]),
        "lon": str(latest_gps["lon"]),
        "hae": str(latest_gps["hae"]),
        "ce": "999999",
        "le": "999999",
    }
    ET.SubElement(root, "point", attrib=gps_data)
    detail = ET.SubElement(root, "detail")
    ET.SubElement(detail, "takv", {"device": DEVICE_CALLSIGN, "platform": "Python", "os": DEVICE_OS})
    ET.SubElement(detail, "contact", {"callsign": DEVICE_CALLSIGN})
    ET.SubElement(detail, "__group", {"name": "Purple", "role": "Team Member"})
    ET.SubElement(detail, "uid", {"Droid": "my-tak-device"})

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
        while not rospy.is_shutdown():
            data = generate_gps_cot()
            if data:
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
        "PYTAK_TLS_CLIENT_CERT": "admin_cert.pem",
        "PYTAK_TLS_CLIENT_KEY": "admin_key.pem",
        "PYTAK_TLS_CA_CERT": "admin-trusted.pem",
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
