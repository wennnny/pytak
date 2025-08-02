#!/usr/bin/env python3
import rospy
import socket
import struct
from std_msgs.msg import String

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

HEADER = 0xAC
LENGTH = 6  # 我們現在傳送 6 個 CAN bytes (b2 ~ b7)
END = 0xBD

def parse_can_rx(data_str):
    try:
        if "ID: 0xCFF4500" in data_str:
            parts = data_str.split("Data:")[1].strip().split(" ")
            b2 = int(parts[2], 16)
            b3 = int(parts[3], 16)
            b4 = int(parts[4], 16)
            b5 = int(parts[5], 16)
            b6 = int(parts[6], 16)
            b7 = int(parts[7], 16)

            return {
                "b2": b2,
                "b3": b3,
                "b4": b4,
                "b5": b5,
                "b6": b6,
                "b7": b7
            }
    except Exception as e:
        rospy.logwarn(f"Failed to parse CAN RX: {e}")
    return None

def can_callback(msg: String):
    try:
        info = parse_can_rx(msg.data)
        if info:
            b2 = info["b2"]
            b3 = info["b3"]
            b4 = info["b4"]
            b5 = info["b5"]
            b6 = info["b6"]
            b7 = info["b7"]

            # 封包格式：Header(1) + Length(1) + b2~b7(6) + End(1) = 9 bytes
            packet = struct.pack('<BB6B B', HEADER, LENGTH, b2, b3, b4, b5, b6, b7, END)
            sock.sendto(packet, (UDP_TARGET_IP, UDP_TARGET_PORT))

            rospy.loginfo(f"[UDP] Sent CAN data: b2={b2}, b3={b3}, b4={b4}, b5={b5}, b6={b6}, b7={b7}")
    except Exception as e:
        rospy.logerr(f"[UDP] Error sending CAN data: {e}")

def main():
    global UDP_TARGET_IP, UDP_TARGET_PORT
    rospy.init_node('can_udp_sender', anonymous=True)
    UDP_TARGET_IP = rospy.get_param("~target_ip", "140.113.148.80")
    UDP_TARGET_PORT = rospy.get_param("~udp_port", 49152)

    rospy.Subscriber("/can_rx", String, can_callback)
    rospy.loginfo("Listening to /can_rx and sending CAN (b2~b7) via UDP...")
    rospy.spin()

if __name__ == "__main__":
    main()
