#!/usr/bin/env python3

import asyncio, uuid, socket, platform
import xml.etree.ElementTree as ET
import pytak
import sys
import rospy
from sensor_msgs.msg import NavSatFix
from configparser import ConfigParser

rospy.init_node('cot_sender', anonymous=True)

DEVICE_CALLSIGN = rospy.get_param("~callsign", socket.gethostname())
DEVICE_UID = DEVICE_CALLSIGN + "-vessel"
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
    latest_gps["hae"] = 99

rospy.init_node('cot_sender', anonymous=True)
# rospy.Subscriber("/mavros/global_position/global", NavSatFix, gps_callback)
rospy.Subscriber("/garmin/fix", NavSatFix, gps_callback)

def generate_gps_cot():
    """Generate a GPS CoT Event with a custom ship icon."""
    if latest_gps["lat"] is None or latest_gps["lon"] is None:
        return None

    root = ET.Element("event")
    root.set("version", "2.0")
    root.set("type", "a-u-S")
    root.set("uid",  DEVICE_UID)           # 確保 UID 不變，覆蓋同一個目標
    root.set("how", "h-g-i-g-o")
    root.set("time", pytak.cot_time())
    root.set("start", pytak.cot_time())
    root.set("stale", pytak.cot_time(300))

    ET.SubElement(root, "point", attrib={
        "lat": str(latest_gps["lat"]),
        "lon": str(latest_gps["lon"]),
        "hae": str(latest_gps.get("hae", 99)),
        "ce": "999999",
        "le": "999999",
    })

    detail = ET.SubElement(root, "detail")
    ET.SubElement(detail, "takv", {"device": DEVICE_CALLSIGN, "platform": "Python", "os": DEVICE_OS})
    ET.SubElement(detail, "contact", {"callsign": DEVICE_CALLSIGN})
    ET.SubElement(detail, "__group", {"name": "Purple", "role": "Team Member"})
    ET.SubElement(detail, "uid", {"Droid": "my-tak-device-2"})
    ET.SubElement(detail, "usericon", {"iconsetpath": "f7f71666-8b28-4b57-9fbb-e38e61d33b79/Google/ferry.png"})

    return ET.tostring(root)


class MySerializer(pytak.QueueWorker):
    async def handle_data(self, data):
        await self.put_queue(data)

    async def run(self):
        """Run the loop for processing or generating pre-CoT data."""
        while not rospy.is_shutdown():
            data = generate_gps_cot()
            if data is None:
                # 還沒拿到第一筆 GPS，避免 .decode() 當掉
                self._logger.info("Waiting for first GPS fix...")
                await asyncio.sleep(0.5)
                continue
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
        "PYTAK_TLS_CLIENT_CERT": "/home/moos-dawg/Downloads/pytak/catkin_ws/files_WAN/argtest2_cert.pem",
        "PYTAK_TLS_CLIENT_KEY": "/home/moos-dawg/Downloads/pytak/catkin_ws/files_WAN/argtest2_key.pem",
        "PYTAK_TLS_CA_CERT": "/home/moos-dawg/Downloads/pytak/catkin_ws/files_WAN/argtest2-trusted.pem",
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

