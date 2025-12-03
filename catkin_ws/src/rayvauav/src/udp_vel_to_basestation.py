#!/usr/bin/env python3
import rospy
import socket
import struct
import time
from geometry_msgs.msg import TwistStamped

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

HEADER = 0xAB
LENGTH = 12  # 4 (seq) + 8 (linear_x)
END = 0xBC

def velocity_callback(msg: TwistStamped):
    try:
        seq = msg.header.seq
        linear_x = msg.twist.linear.x

        packet = struct.pack('<BBIdB', HEADER, LENGTH, seq, linear_x, END)
        sock.sendto(packet, (UDP_TARGET_IP, UDP_TARGET_PORT))

        rospy.loginfo(f"[UDP] Sent LinearX={linear_x:.3f} with Seq={seq}")

    except Exception as e:
        rospy.logerr(f"[UDP] Error: {e}")

def main():
    global UDP_TARGET_IP, UDP_TARGET_PORT

    rospy.init_node('velocity_udp_sender', anonymous=True)
    UDP_TARGET_IP = rospy.get_param("~target_ip", "140.113.148.80")
    UDP_TARGET_PORT = rospy.get_param("~udp_port", 49152)

    rospy.Subscriber("/vel", TwistStamped, velocity_callback)
    rospy.loginfo("Subscribing to /vel and sending linear.x via UDP...")
    rospy.spin()

if __name__ == '__main__':
    main()
