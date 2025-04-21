#!/usr/bin/env python3

import socket
import xml.etree.ElementTree as ET
import rospy
from std_msgs.msg import String

SYNC_IP = "0.0.0.0"
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

def decimal_to_dms(decimal, is_latitude=True):
    """ Convert decimal degrees to DMS format string. """
    direction = ''
    if is_latitude:
        direction = 'N' if decimal >= 0 else 'S'
    else:
        direction = 'E' if decimal >= 0 else 'W'

    decimal = abs(decimal)
    degrees = int(decimal)
    minutes_float = (decimal - degrees) * 60
    minutes = int(minutes_float)
    seconds = round((minutes_float - minutes) * 60, 2)

    return f"{degrees}°{minutes}'{seconds}\"{direction}"

def parse_cot(xml_data):
    """ Parse CoT XML data and return all needed fields. """
    root = ET.fromstring(xml_data)

    # Get type from event element
    event_type = root.get("type", "Unknown")

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

    return callsign, event_type, lat, lon, hae, color_hex

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
            callsign, event_type, lat, lon, hae, color_hex = parse_cot(xml_str)

            if event_type != "b-m-p-s-m":
                rospy.loginfo(f"[SYNC] Skipped CoT with type: {event_type}")
                continue

            lat_dms = decimal_to_dms(lat, is_latitude=True)
            lon_dms = decimal_to_dms(lon, is_latitude=False)

            msg = String()
            # Format: callsign, type, lat, lon, hae, color, lat_dms, lon_dms
            msg.data = f"{callsign},{event_type},{lat},{lon},{hae},{color_hex}"

            pub.publish(msg)
            rospy.loginfo(f"[SYNC] Published: {msg.data}")
        except Exception as e:
            rospy.logerr(f"[SYNC] Error parsing CoT: {e}")



if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException or KeyboardInterrupt:
        rospy.loginfo("[SYNC] Shutting down.")
        pass
