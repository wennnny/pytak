#!/usr/bin/env python3
import rospy
import socket
import struct
from geometry_msgs.msg import PoseArray

UDP_TARGET_IP = rospy.get_param("~target_ip", "140.113.148.80")
UDP_TARGET_PORT = rospy.get_param("~udp_port", 49152)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

HEADER = 0xAE
END = 0xBF

def posearray_callback(msg: PoseArray):
    try:
        for idx, pose in enumerate(msg.poses):
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z
            LENGTH = 13  # 1 (index) + 3*4 (float32 xyz)

            packet = struct.pack('<BBBfffB', HEADER, LENGTH, idx, x, y, z, END)
            sock.sendto(packet, (UDP_TARGET_IP, UDP_TARGET_PORT))
            rospy.loginfo(f"[UDP] Sent Pose[{idx}] -> x: {x:.2f}, y: {y:.2f}, z: {z:.2f}")
    except Exception as e:
        rospy.logerr(f"[UDP] Error sending PoseArray: {e}")

def main():
    rospy.init_node('posearray_udp_sender', anonymous=True)
    rospy.Subscriber("/detected_obstacles/gps/pose_array", PoseArray, posearray_callback)
    rospy.loginfo("Listening to /detected_obstacles/gps/pose_array and sending each Pose via UDP...")
    rospy.spin()

if __name__ == "__main__":
    main()
