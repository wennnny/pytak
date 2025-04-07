#!/usr/bin/env python3

import asyncio, uuid, socket, platform
import xml.etree.ElementTree as ET
import pytak
import sys
import rospy
from std_msgs.msg import String
from configparser import ConfigParser

DEVICE_CALLSIGN = socket.gethostname()
DEVICE_UID = str(uuid.uuid4())
DEVICE_OS = platform.system()
SYNC_IP = "127.0.0.1"
SYNC_PORT = 5005
sync_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

for arg in sys.argv:
    if arg.startswith("callsign:="):
        DEVICE_CALLSIGN = arg.split(":=")[1]
    if arg.startswith("uid:="):
        DEVICE_UID = arg.split(":=")[1]

def generate_gps_cot():
    """生成一個 CoT 事件"""
    root = ET.Element("event")
    root.set("version", "2.0")
    root.set("type", "a-f-G-U-C-I")
    root.set("uid",  DEVICE_UID)
    root.set("how", "m-g")
    root.set("time", pytak.cot_time())
    root.set("start", pytak.cot_time())
    root.set("stale", pytak.cot_time(300))

    gps_data = {
        "lat": "22.604910",
        "lon": "120.295591",
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

def parse_cot(xml_data):
    """解析 CoT 事件，提取 `lat, lon, hae, callsign`"""
    root = ET.fromstring(xml_data)

    point = root.find("point")
    if point is not None:
        lat = point.get("lat", "0")
        lon = point.get("lon", "0")
        hae = point.get("hae", "0")
    else:
        lat, lon, hae = "0", "0", "0"

    contact = root.find("./detail/contact")
    callsign = contact.get("callsign", "Unknown") if contact is not None else "Unknown"

    return f"{lat},{lon},{hae},{callsign}"

class MySerializer(pytak.QueueWorker):
    """
    發送 CoT 事件，並同時將資訊發布到 ROS topic `/cot_message`
    """

    def __init__(self, queue, config):
        super().__init__(queue, config)
        rospy.init_node('cot_publisher', anonymous=True)
        self.pub = rospy.Publisher('/cot_message', String, queue_size=10)

    async def handle_data(self, data):
        """處理 CoT 事件，並放入佇列"""
        event = data
        await self.put_queue(event)

    async def run(self):
        """定期產生 CoT 事件，並發送到 CoT 伺服器與 ROS"""
        while not rospy.is_shutdown():
            data = generate_gps_cot()
            self._logger.info("Sending CoT Event:\n%s\n", data.decode())

            await self.handle_data(data)

            cot_string = parse_cot(data)  
            
            msg = String()
            msg.data = cot_string
            
            self.pub.publish(msg)
            rospy.loginfo(f"[ROS] Published CoT Message: {msg.data}")

            await asyncio.sleep(5)

async def main():
    """初始化 CoT 連線與 ROS"""
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

    clitool = pytak.CLITool(config)
    await clitool.setup()

    clitool.add_tasks(set([MySerializer(clitool.tx_queue, config)]))

    await clitool.run()

if __name__ == "__main__":
    asyncio.run(main())
