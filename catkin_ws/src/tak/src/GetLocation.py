#!/usr/bin/env python3

import socket
import xml.etree.ElementTree as ET
import rospy
from tak.msg import CotMessage
from geometry_msgs.msg import Point

SYNC_IP = "127.0.0.1"
SYNC_PORT = 5005

def parse_cot(xml_data):
    """ Parse CoT XML data and return the event attributes. """
    root = ET.fromstring(xml_data)

    # Get CoT attributes
    uid = root.get("uid", "Unknown")
    event_type = root.get("type", "Unknown")
    how = root.get("how", "Unknown")
    time = root.get("time", "Unknown")
    start = root.get("start", "Unknown")
    stale = root.get("stale", "Unknown")

    # Get location data
    point = root.find("point")
    if point is not None:
        lat = float(point.get("lat", 0))
        lon = float(point.get("lon", 0))
        hae = float(point.get("hae", 0))
    else:
        lat, lon, hae = 0, 0, 0

    # Get callsign
    contact = root.find("./detail/contact")
    callsign = contact.get("callsign", "Unknown") if contact is not None else "Unknown"

    # Get device info
    takv = root.find("./detail/takv")
    device = takv.get("device", "Unknown") if takv is not None else "Unknown"
    platform = takv.get("platform", "Unknown") if takv is not None else "Unknown"
    os = takv.get("os", "Unknown") if takv is not None else "Unknown"
    version = takv.get("version", "Unknown") if takv is not None else "Unknown"

    return uid, event_type, how, time, start, stale, lat, lon, hae, callsign, device, platform, os, version

def main():
    rospy.init_node('cot_sync_node')
    pub = rospy.Publisher('/cot_message', CotMessage, queue_size=10)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((SYNC_IP, SYNC_PORT))
    
    rospy.loginfo(f"[SYNC] Listening on {SYNC_IP}:{SYNC_PORT}...")

    while not rospy.is_shutdown():
        data, addr = sock.recvfrom(65535)
        xml_str = data.decode()

        try:
            uid, event_type, how, time, start, stale, lat, lon, hae, callsign, device, platform, os, version = parse_cot(xml_str)
            
            msg = CotMessage()
            msg.uid = uid
            msg.type = event_type
            msg.how = how
            msg.time = time
            msg.start = start
            msg.stale = stale
            msg.lat = lat
            msg.lon = lon
            msg.hae = hae
            msg.callsign = callsign
            msg.device = device
            msg.platform = platform
            msg.os = os
            msg.version = version
            
            pub.publish(msg)
            
            rospy.loginfo(f"[SYNC] Published CoT Message: uid={uid}, callsign={callsign}, lat={lat}, lon={lon}, hae={hae}")
        except Exception as e:
            rospy.logerr(f"[SYNC] Error parsing CoT: {e}")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
