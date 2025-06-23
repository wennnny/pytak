#!/usr/bin/env python3
import rospy
import socket
import struct
import time
from geometry_msgs.msg import TwistStamped

UDP_IP = "140.113.148.80"
UDP_PORT = 49152

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

HEADER = 0xAB
LENGTH = 12  # 4 (seq) + 8 (linear_x)
END = 0xBC

def velocity_callback(msg: TwistStamped):
    try:
        seq = msg.header.seq
        linear_x = msg.twist.linear.x

        # 封包格式：Header(1) + Length(1) + Seq(4) + linear_x(8) + End(1) = 15 bytes
        packet = struct.pack('<BBIdB', HEADER, LENGTH, seq, linear_x, END)
        sock.sendto(packet, (UDP_IP, UDP_PORT))

        rospy.loginfo(f"[UDP] Sent LinearX={linear_x:.3f} with Seq={seq}")

    except Exception as e:
        rospy.logerr(f"[UDP] Error: {e}")

def main():
    rospy.init_node('velocity_udp_sender', anonymous=True)
    rospy.Subscriber("/mavros/local_position/velocity_body", TwistStamped, velocity_callback)
    rospy.loginfo("Subscribing to /mavros/local_position/velocity_body and sending linear.x via UDP...")
    rospy.spin()

if __name__ == '__main__':
    main()
