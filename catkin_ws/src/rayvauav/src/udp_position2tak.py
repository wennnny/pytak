#!/usr/bin/env python3

import asyncio, uuid, socket, platform, threading, struct, time
import xml.etree.ElementTree as ET
import pytak
import rospy
from std_msgs.msg import String
from configparser import ConfigParser

DEVICE_UID = str(uuid.uuid4())
DEVICE_OS = platform.system()
SYNC_IP = "127.0.0.1"
SYNC_PORT = 5005
sync_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

UDP_LISTEN_IP = "0.0.0.0"
UDP_LISTEN_PORT = 49152

latest_gps = {"lat": None, "lon": None, "hae": 999999.0}
latest_gps_time = 0
ros_pub = None


def udp_listener():
    global latest_gps, latest_gps_time
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_LISTEN_IP, UDP_LISTEN_PORT))
    print(f"[UDP] Listening for binary packets on {UDP_LISTEN_IP}:{UDP_LISTEN_PORT}...")

    while True:
        data, _ = sock.recvfrom(1024)
        if len(data) != 19:
            print(f"[UDP] Invalid packet length: {len(data)}")
            continue

        header = data[0]
        length = data[1]
        if length != 16:
            print(f"[UDP] Unexpected payload length: {length}")
            continue

        lat = struct.unpack('<d', data[2:10])[0]
        lon = struct.unpack('<d', data[10:18])[0]
        endcode = data[18]

        latest_gps["lat"] = lat
        latest_gps["lon"] = lon
        latest_gps_time = time.time()

        print(f"[UDP] Packet - Header: {header}, Length: {length}, Lat: {lat:.6f}, Lon: {lon:.6f}, End: {endcode}")


def generate_gps_cot():
    if latest_gps["lat"] is None or latest_gps["lon"] is None or (time.time() - latest_gps_time) > 5:
        return None

    callsign = "UAV"

    if ros_pub:
        ros_msg = String()
        ros_msg.data = f"{callsign},a-f-G-U-C-I,{latest_gps['lat']},{latest_gps['lon']},{latest_gps['hae']}"
        ros_pub.publish(ros_msg)
        rospy.loginfo(f"[ROS] Published to /uav/global_position: {ros_msg.data}")

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
    ET.SubElement(detail, "takv", {"device": "UAV", "platform": "Python", "os": DEVICE_OS})
    ET.SubElement(detail, "contact", {"callsign": callsign})
    ET.SubElement(detail, "__group", {"name": "Purple", "role": "Team Member"})
    ET.SubElement(detail, "uid", {"Droid": "my-tak-device"})

    return ET.tostring(root)


class MySerializer(pytak.QueueWorker):
    async def handle_data(self, data):
        await self.put_queue(data)

    async def run(self):
        while not rospy.is_shutdown():
            data = generate_gps_cot()
            if data:
                self._logger.info("Sending:\n%s\n", data.decode())
                await self.handle_data(data)
            await asyncio.sleep(1)


class MyReceiver(pytak.QueueWorker):
    async def handle_data(self, data):
        self._logger.info("[Received] CoT Event:\n%s\n", data.decode())
        sync_socket.sendto(data, (SYNC_IP, SYNC_PORT))

    async def run(self):
        while True:
            data = await self.queue.get()
            await self.handle_data(data)


async def main():
    global ros_pub

    rospy.init_node("uav_gps_node", anonymous=True)
    ros_pub = rospy.Publisher('/uav/global_position', String, queue_size=10)

    threading.Thread(target=udp_listener, daemon=True).start()

    config = ConfigParser()
    config["mycottool"] = {
        "COT_URL": "tls://192.168.2.159:8089",
        "PYTAK_TLS_CLIENT_CERT": "UAV_cert.pem",
        "PYTAK_TLS_CLIENT_KEY": "UAV_key.pem",
        "PYTAK_TLS_CA_CERT": "UAV-trusted.pem",
        "PYTAK_TLS_DONT_CHECK_HOSTNAME": "1",
        "PYTAK_TLS_DONT_VERIFY": "1"
    }

    clitool = pytak.CLITool(config["mycottool"])
    await clitool.setup()

    clitool.add_tasks({
        MySerializer(clitool.tx_queue, config["mycottool"]),
        MyReceiver(clitool.rx_queue, config["mycottool"]),
    })

    await clitool.run()


if __name__ == "__main__":
    asyncio.run(main())
