#!/usr/bin/env python3

import rospy
import socket
import struct
from std_msgs.msg import String

UDP_TARGET_IP = "192.168.168.177"
UDP_TARGET_PORT = 49153
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

HEADER = 0xAA
LENGTH = 16
END = 0xBB

def cot_callback(msg):
    try:
        parts = msg.data.strip().split(",")
        if len(parts) < 2:
            rospy.logwarn("Received malformed message.")
            return

        lat = float(parts[0])
        lon = float(parts[1])

        packet = struct.pack('<BBddB', HEADER, LENGTH, lat, lon, END)
        sock.sendto(packet, (UDP_TARGET_IP, UDP_TARGET_PORT))

        local_ip, local_port = sock.getsockname()
        rospy.loginfo(f"[UDP] Sent from {local_ip}:{local_port} -> {UDP_TARGET_IP}:{UDP_TARGET_PORT} | Lat: {lat:.6f}, Lon: {lon:.6f}")
    except Exception as e:
        rospy.logerr(f"Error parsing or sending packet: {e}")

def main():
    rospy.init_node('cot_udp_sender', anonymous=True)
    rospy.Subscriber("/cot_message", String, cot_callback)
    rospy.loginfo("Listening to /cot_message and sending 19-byte UDP packets...")
    rospy.spin()

if __name__ == "__main__":
    main()
