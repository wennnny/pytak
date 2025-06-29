#!/usr/bin/env python3
import rospy
import socket
import struct
from std_msgs.msg import Float64

UDP_TARGET_IP = rospy.get_param("~target_ip", "140.113.148.80")
UDP_TARGET_PORT = rospy.get_param("~udp_port", 49152)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

HEADER = 0xAD
LENGTH = 8
END = 0xBE

def compass_callback(msg: Float64):
    try:
        compass_value = msg.data  # float64
        packet = struct.pack('<BBdB', HEADER, LENGTH, compass_value, END)
        sock.sendto(packet, (UDP_TARGET_IP, UDP_TARGET_PORT))
        rospy.loginfo(f"[UDP] Sent compass heading: {compass_value:.2f}")
    except Exception as e:
        rospy.logerr(f"[UDP] Error sending compass heading: {e}")

def main():
    rospy.init_node('compass_udp_sender', anonymous=True)
    rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, compass_callback)
    rospy.loginfo("Listening to /mavros/global_position/compass_hdg and sending via UDP...")
    rospy.spin()

if __name__ == "__main__":
    main()
