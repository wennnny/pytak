#!/usr/bin/env python3

import rospy
import socket
from std_msgs.msg import String, Header
from geometry_msgs.msg import Pose, PoseArray
import xml.etree.ElementTree as ET

SYNC_IP = "0.0.0.0"
SYNC_PORT = 5005

def parse_cot(xml_data):
    root = ET.fromstring(xml_data)

    event_type = root.get("type", "Unknown")

    point = root.find("point")
    lat = float(point.get("lat", 0)) if point is not None else 0
    lon = float(point.get("lon", 0)) if point is not None else 0
    hae = float(point.get("hae", 0)) if point is not None else 0

    contact = root.find("./detail/contact")
    callsign = contact.get("callsign", "Unknown") if contact is not None else "Unknown"

    color_elem = root.find("./detail/color")
    argb = color_elem.get("argb", "0") if color_elem is not None else "0"
    color_hex = f"#{int(argb) & 0xFFFFFF:06X}"

    return callsign, event_type, lat, lon, hae, color_hex

def latlonhae_to_pose(lat, lon, hae):
    pose = Pose()
    pose.position.x = lon
    pose.position.y = lat
    pose.position.z = hae
    pose.orientation.w = 1.0
    return pose

def main():
    rospy.init_node('cot_sync_node')
    pub_debug = rospy.Publisher('/cot_message', String, queue_size=10)
    pub_waypoint = rospy.Publisher('/waypoint/global_position', PoseArray, queue_size=10)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((SYNC_IP, SYNC_PORT))

    rospy.loginfo(f"[SYNC] Listening on {SYNC_IP}:{SYNC_PORT}...")

    waypoint_list = []

    while not rospy.is_shutdown():
        data, addr = sock.recvfrom(65535)
        xml_str = data.decode()

        try:
            callsign, event_type, lat, lon, hae, color_hex = parse_cot(xml_str)

            if event_type != "b-m-p-s-m":
                rospy.loginfo(f"[SYNC] Skipped CoT with type: {event_type}")
                continue

            msg = String()
            msg.data = f"{callsign},{event_type},{lat},{lon},{hae},{color_hex}"
            pub_debug.publish(msg)

            if callsign.startswith("waypoint"):
                pose = latlonhae_to_pose(lat, lon, hae)
                waypoint_list.append(pose)

                if callsign.endswith("!"):
                    pose_array = PoseArray()
                    pose_array.header = Header()
                    pose_array.header.stamp = rospy.Time.now()
                    pose_array.header.frame_id = "map"
                    pose_array.poses = waypoint_list

                    pub_waypoint.publish(pose_array)
                    rospy.loginfo(f"[SYNC] Published {len(waypoint_list)} waypoints to /waypoint/global_position")
                    waypoint_list = []
        except Exception as e:
            rospy.logerr(f"[SYNC] Error parsing CoT: {e}")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("[SYNC] Shutting down.")
