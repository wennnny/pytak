#!/usr/bin/env python3
import rospy
import socket
import struct
from std_msgs.msg import String

UDP_TARGET_IP = "140.113.148.80"
UDP_TARGET_PORT = 49152

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

HEADER = 0xAC
LENGTH = 2  # 1 byte for b6 (throttle), 1 byte for b7 (steering)
END = 0xBD

def parse_can_rx(data_str):
    try:
        if "ID: 0xCFF4500" in data_str:
            parts = data_str.split("Data:")[1].strip().split(" ")
            b6 = int(parts[6], 16)  # Current Throttle Position
            b7 = int(parts[7], 16)  # Current Steering Angle
            return b6, b7
    except Exception as e:
        rospy.logwarn(f"Failed to parse CAN RX: {e}")
    return None, None

def can_callback(msg: String):
    try:
        b6, b7 = parse_can_rx(msg.data)
        if b6 is not None and b7 is not None:
            packet = struct.pack('<BB2B B', HEADER, LENGTH, b6, b7, END)
            sock.sendto(packet, (UDP_TARGET_IP, UDP_TARGET_PORT))
            rospy.loginfo(f"[UDP] Sent CAN data - Throttle(b6): {b6}, Steering(b7): {b7}")
    except Exception as e:
        rospy.logerr(f"[UDP] Error sending CAN data: {e}")

def main():
    rospy.init_node('can_udp_sender', anonymous=True)
    rospy.Subscriber("/can_rx", String, can_callback)
    rospy.loginfo("Listening to /can_rx and sending b6/b7 via UDP...")
    rospy.spin()

if __name__ == "__main__":
    main()
