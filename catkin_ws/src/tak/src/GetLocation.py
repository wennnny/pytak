#!/usr/bin/env python3

import socket
import xml.etree.ElementTree as ET
import rospy
from geometry_msgs.msg import Point

# UDP 設定
SYNC_IP = "127.0.0.1"
SYNC_PORT = 5005

def parse_cot(xml_data):
    """解析 CoT XML 並提取位置"""
    root = ET.fromstring(xml_data)
    point = root.find("point")
    
    if point is not None:
        lat = float(point.get("lat", 0))
        lon = float(point.get("lon", 0))
        hae = float(point.get("hae", 0))
        return lat, lon, hae
    else:
        return None, None, None

def main():
    # 初始化 ROS node
    rospy.init_node('cot_sync_node')
    pub = rospy.Publisher('/cot_position', Point, queue_size=10)

    # 設定 UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((SYNC_IP, SYNC_PORT))
    
    rospy.loginfo(f"[SYNC] Listening on {SYNC_IP}:{SYNC_PORT}...")

    while not rospy.is_shutdown():
        data, addr = sock.recvfrom(65535)
        xml_str = data.decode()

        try:
            lat, lon, hae = parse_cot(xml_str)
            if lat is not None:
                point_msg = Point()
                point_msg.x = lat
                point_msg.y = lon
                point_msg.z = hae
                pub.publish(point_msg)
                rospy.loginfo(f"[SYNC] Published position: lat={lat}, lon={lon}, hae={hae}")
            else:
                rospy.logwarn("[SYNC] No point data in CoT event.")
        except Exception as e:
            rospy.logerr(f"[SYNC] Error parsing CoT: {e}")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

