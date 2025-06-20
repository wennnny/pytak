#!/usr/bin/env python3
import rospy
import socket
import struct
from sensor_msgs.msg import NavSatFix
UDP_TARGET_IP = "192.168.0.109"
UDP_TARGET_PORT = 49153
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
HEADER = 0xAA
LENGTH = 16
END = 0xBB
def gps_callback(msg: NavSatFix):
    try:
        lat = msg.latitude
        lon = msg.longitude
        packet = struct.pack('<BBddB', HEADER, LENGTH, lat, lon, END)
        sock.sendto(packet, (UDP_TARGET_IP, UDP_TARGET_PORT))
        local_ip, local_port = sock.getsockname()
        rospy.loginfo(f"[UDP] Sent from {local_ip}:{local_port} -> {UDP_TARGET_IP}:{UDP_TARGET_PORT} | Lat: {lat:.6f}, Lon: {lon:.6f}")
    except Exception as e:
        rospy.logerr(f"[UDP] Error sending packet: {e}")
def main():
    rospy.init_node('gps_udp_sender', anonymous=True)
    rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, gps_callback)
    rospy.loginfo(":逆時針箭頭: Listening to /mavros/global_position/raw/fix and sending UDP packets...")
    rospy.spin()
if __name__ == "__main__":
    main()