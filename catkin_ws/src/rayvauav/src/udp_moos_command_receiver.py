#!/usr/bin/env python3
import rospy
import socket
from std_msgs.msg import String

# === ROS 初始化 ===
rospy.init_node('udp_message_receiver', anonymous=True)

UDP_PORT = rospy.get_param("~udp_port", 49158)  # 預設 port 可改
UDP_IP = rospy.get_param("~udp_ip", "0.0.0.0")  # Listen on all interfaces

# === UDP Socket 設定 ===
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(1.0)

rospy.loginfo(f"Listening for UDP messages on {UDP_IP}:{UDP_PORT}")

# === ROS Publisher ===
pub_msg = rospy.Publisher('/cot_chat/udp_message', String, queue_size=10)

rate = rospy.Rate(50)

try:
    while not rospy.is_shutdown():
        try:
            data, addr = sock.recvfrom(1024)
            message = data.decode('utf-8').strip()
            if message:
                rospy.loginfo(f"[RECV] From {addr}: {message}")
                ros_msg = String(data=message)
                pub_msg.publish(ros_msg)
        except socket.timeout:
            continue
        except UnicodeDecodeError:
            rospy.logwarn("Received non-UTF-8 data")
        except Exception as e:
            rospy.logerr(f"UDP error: {e}")

        rate.sleep()

finally:
    sock.close()
    rospy.loginfo("Socket closed. Node exiting.")
