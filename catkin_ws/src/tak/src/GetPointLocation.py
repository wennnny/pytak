#!/usr/bin/env python3

import socket
import xml.etree.ElementTree as ET
import rospy
from std_msgs.msg import String

SYNC_IP = "127.0.0.1"
SYNC_PORT = 5005

def argb_to_hex(argb_str):
    """ Convert ARGB string to hex color format. """
    try:
        value = int(argb_str)
        unsigned = value & 0xFFFFFFFF
        hex_str = f"{unsigned:08X}"
        r = hex_str[2:4]
        g = hex_str[4:6]
        b = hex_str[6:8]
        return f"#{r}{g}{b}"
    except:
        return "#000000"

def parse_cot(xml_data):
    """ Parse CoT XML data and return lat, lon, hae, callsign, color_hex. """
    root = ET.fromstring(xml_data)

    # Get location
    point = root.find("point")
    lat = float(point.get("lat", 0)) if point is not None else 0
    lon = float(point.get("lon", 0)) if point is not None else 0
    hae = float(point.get("hae", 0)) if point is not None else 0

    # Get callsign
    contact = root.find("./detail/contact")
    callsign = contact.get("callsign", "Unknown") if contact is not None else "Unknown"

    # Get color in ARGB
    color_elem = root.find("./detail/color")
    argb = color_elem.get("argb", "0") if color_elem is not None else "0"
    color_hex = argb_to_hex(argb)

    return lat, lon, hae, callsign, color_hex

def main():
    rospy.init_node('cot_sync_node')
    pub = rospy.Publisher('/cot_message', String, queue_size=10)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((SYNC_IP, SYNC_PORT))
    
    rospy.loginfo(f"[SYNC] Listening on {SYNC_IP}:{SYNC_PORT}...")

    while not rospy.is_shutdown():
        data, addr = sock.recvfrom(65535)
        xml_str = data.decode()

        try:
            lat, lon, hae, callsign, color_hex = parse_cot(xml_str)

            msg = String()
            msg.data = f"{lat},{lon},{hae},{callsign},{color_hex}"

            pub.publish(msg)

            rospy.loginfo(f"[SYNC] Published: {msg.data}")
        except Exception as e:
            rospy.logerr(f"[SYNC] Error parsing CoT: {e}")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
