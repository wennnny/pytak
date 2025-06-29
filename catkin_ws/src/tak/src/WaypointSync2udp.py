#!/usr/bin/env python3

import rospy
import socket
import struct
from std_msgs.msg import String, Header
from geometry_msgs.msg import Pose, PoseArray
import xml.etree.ElementTree as ET

SYNC_IP = "0.0.0.0"
SYNC_PORT = 5005

UDP_TARGET_IP = rospy.get_param("/cot_udp_target_ip", "140.113.148.99")
UDP_TARGET_PORT = rospy.get_param("/cot_udp_target_port", 49157)

HEADER = 0xAF
END = 0xC0

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

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
    LENGTH = 25  # index + lat + lon + hae
    packet = struct.pack('<BBBdddB', HEADER, LENGTH, index, lat, lon, hae, END)
    sock.sendto(packet, (UDP_TARGET_IP, UDP_TARGET_PORT))
    rospy.loginfo(f"[UDP] Sent waypoint{index}: lat={lat:.6f}, lon={lon:.6f}, hae={hae:.2f}")

def main():
    rospy.init_node('cot_sync_udp_node')
    pub_debug = rospy.Publisher('/cot_message', String, queue_size=10)

    rospy.loginfo(f"[SYNC] Listening on {SYNC_IP}:{SYNC_PORT}...")
    rospy.loginfo(f"[UDP] Target: {UDP_TARGET_IP}:{UDP_TARGET_PORT}")

    sock_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_recv.bind((SYNC_IP, SYNC_PORT))

    waypoint_list = []

    while not rospy.is_shutdown():
        data, addr = sock_recv.recvfrom(65535)
        xml_str = data.decode()

        try:
            callsign, event_type, lat, lon, hae = parse_cot(xml_str)

            if event_type != "b-m-p-s-m":
                continue

            msg = String()
            msg.data = f"{callsign},{event_type},{lon},{lat},{hae}"
            pub_debug.publish(msg)

            if callsign.startswith("waypoint"):
                waypoint_list.append((lat, lon, hae))

                if callsign.endswith("!"):
                    for idx, (lat, lon, hae) in enumerate(waypoint_list):
                        send_waypoint_udp(idx, lat, lon, hae)
                    rospy.loginfo(f"[UDP] Published {len(waypoint_list)} waypoints via UDP")
                    waypoint_list = []

        except Exception as e:
            rospy.logerr(f"[SYNC] Error parsing CoT: {e}")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("[SYNC] Shutting down.")
