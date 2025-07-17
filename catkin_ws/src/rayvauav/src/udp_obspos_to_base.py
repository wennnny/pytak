#!/usr/bin/env python3
import rospy
import socket
import struct
from geometry_msgs.msg import PoseArray

UDP_TARGET_IP = rospy.get_param("~target_ip", "192.168.0.113")
UDP_TARGET_PORT = rospy.get_param("~udp_port", 49153)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

HEADER = 0xAE
END = 0xBF

def posearray_callback(msg: PoseArray):
    try:
        for idx, pose in enumerate(msg.poses):
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z

            packet = struct.pack('=BBBfffB', HEADER, 0, idx, x, y, z, END)
            sock.sendto(packet, (UDP_TARGET_IP, UDP_TARGET_PORT))

            rospy.loginfo(f"[UDP] Sent Pose[{idx}] -> x: {x:.6f}, y: {y:.6f}, z: {z:.6f} [len={len(packet)}]")
    except Exception as e:
        rospy.logerr(f"[UDP] Error sending PoseArray: {e}")

def main():
    rospy.init_node('posearray_udp_sender', anonymous=True)
    rospy.Subscriber("/detected_obstacles/gps/pose_array", PoseArray, posearray_callback)
    rospy.loginfo(f"Listening to /detected_obstacles/gps/pose_array and sending each Pose via UDP to {UDP_TARGET_IP}:{UDP_TARGET_PORT}...")
    rospy.spin()

if __name__ == "__main__":
    main()
