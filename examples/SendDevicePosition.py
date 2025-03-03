#!/usr/bin/env python3

import asyncio, uuid, socket, platform
import xml.etree.ElementTree as ET
import pytak
import geocoder, sys
from configparser import ConfigParser

DEVICE_CALLSIGN = socket.gethostname()
DEVICE_UID = str(uuid.uuid4())
DEVICE_OS = platform.system()

for arg in sys.argv:
    if arg.startswith("callsign=:"):
        DEVICE_CALLSIGN = arg.split("=:")[1]
    if arg.startswith("uid=:"):
        DEVICE_UID = arg.split("=:")[1]

# def get_gps():
#     """Get GPS coordinates from the device."""
#     g = geocoder.ip('me')
#     if g.ok:
#         lat, lon = g.latlng
#         return str(lat), str(lon)
#     else:
#         print("Failed to get GPS coordinates.")
#         return None, None

def generate_gps_cot():
    """Generate a simple takPong CoT Event."""
    root = ET.Element("event")
    root.set("version", "2.0")
    root.set("type", "a-f-G-U-C-I")
    root.set("uid",  DEVICE_UID)
    root.set("how", "m-g")
    root.set("time", pytak.cot_time())
    root.set("start", pytak.cot_time())
    root.set("stale", pytak.cot_time(300))

    # lat, lon = get_gps()
    gps_data = {
        "lat": "24.784074",
        "lon": "120.998384",
        "hae": "999999",
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
        """Run the loop for processing or generating pre-CoT data."""
        while True:
            data = generate_gps_cot()
            self._logger.info("Sending:\n%s\n", data.decode())
            await self.handle_data(data)
            await asyncio.sleep(30)

class MyReceiver(pytak.QueueWorker):
    """Handle incoming CoT events."""

    async def handle_data(self, data):
        self._logger.info("[Received] CoT Event:\n%s\n", data.decode())

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
        "PYTAK_TLS_CLIENT_CERT": "/home/wen/Downloads/user1_cert.pem",
        "PYTAK_TLS_CLIENT_KEY": "/home/wen/Downloads/user1_key.pem",
        "PYTAK_TLS_CA_CERT": "/home/wen/Downloads/user1-trusted.pem",
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
