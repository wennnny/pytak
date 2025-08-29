#!/usr/bin/env python3
import rospy
import socket
import struct
from geometry_msgs.msg import PoseArray

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

HEADER = 0xAF
END = 0xBF
LENGTH = 16

def posearray_callback(msg: PoseArray):
    try:
        for idx, pose in enumerate(msg.poses):
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z

            packet = struct.pack('<BBBfffB', HEADER, LENGTH, idx, x, y, z, END)
            sock.sendto(packet, (UDP_TARGET_IP, UDP_TARGET_PORT))

            rospy.loginfo(f"[UDP] Sent Goal[{idx}] -> x: {x:.6f}, y: {y:.6f}, z: {z:.6f} [len={len(packet)}]")
    except Exception as e:
        rospy.logerr(f"[UDP] Error sending PoseArray: {e}")

def main():
    global UDP_TARGET_IP, UDP_TARGET_PORT

    rospy.init_node('posearray_udp_sender', anonymous=True)
    UDP_TARGET_IP = rospy.get_param("~target_ip", "10.0.0.80")
    UDP_TARGET_PORT = rospy.get_param("~udp_port", 49153)

    rospy.Subscriber("/goal_list/gps/pose_array", PoseArray, posearray_callback)
    rospy.loginfo(f"Listening to /goal_list/gps/pose_array and sending each Pose via UDP to {UDP_TARGET_IP}:{UDP_TARGET_PORT}...")
    rospy.spin()

if __name__ == "__main__":
    main()
