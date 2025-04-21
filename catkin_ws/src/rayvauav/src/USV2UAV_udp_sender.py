#!/usr/bin/env python3

import rospy
import socket
import struct
from sensor_msgs.msg import NavSatFix

UDP_TARGET_IP = "192.168.168.121"
UDP_TARGET_PORT = 49152
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

HEADER = 2
LENGTH = 16
END = 255

def gps_callback(data):
    lat = data.latitude
    lon = data.longitude

    packet = struct.pack('<BBddB', HEADER, LENGTH, lat, lon, END)
    sock.sendto(packet, (UDP_TARGET_IP, UDP_TARGET_PORT))

    local_ip, local_port = sock.getsockname()
    rospy.loginfo(f"Sent from {local_ip}:{local_port} -> {UDP_TARGET_IP}:{UDP_TARGET_PORT} | Lat: {lat:.6f}, Lon: {lon:.6f}")

def main():
    rospy.init_node('gps_udp_sender', anonymous=True)
    rospy.Subscriber("/mavros/global_position/global", NavSatFix, gps_callback)
    rospy.loginfo("Listening to /mavros/global_position/global and sending UDP packets...")
    rospy.spin()

if __name__ == "__main__":
    main()
