#!/usr/bin/env python3
import rospy
import socket, time
import struct
from sensor_msgs.msg import NavSatFix
from datetime import datetime, timedelta

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

HEADER = 0xAA
LENGTH = 28  # 4 (seq) + 8 (timestamp) + 8 (lat) + 8 (lon)
END = 0xBB

def gps_callback(msg: NavSatFix):
    try:
        seq = msg.header.seq
        stamp = time.time()
        print("Taiwan time :", datetime.utcfromtimestamp(stamp) + timedelta(hours=8))
        lat = msg.latitude
        lon = msg.longitude

        # <BBIdd dB: uint8, uint8, uint32, float64 (stamp), float64 (lat), float64 (lon), uint8
        packet = struct.pack('<BBIdddB', HEADER, LENGTH, seq, stamp, lat, lon, END)
        sock.sendto(packet, (UDP_TARGET_IP, UDP_TARGET_PORT))

        local_ip, local_port = sock.getsockname()
        rospy.loginfo(f"[UDP] Sent from {local_ip}:{local_port} -> {UDP_TARGET_IP}:{UDP_TARGET_PORT} | Seq: {seq} | Time: {stamp:.3f} | Lat: {lat:.6f}, Lon: {lon:.6f}")

    except Exception as e:
        rospy.logerr(f"[UDP] Error sending packet: {e}")

def main():
    global UDP_TARGET_IP, UDP_TARGET_PORT

    rospy.init_node('gps_udp_sender', anonymous=True)
    UDP_TARGET_IP = rospy.get_param("~target_ip", "140.113.148.80")
    UDP_TARGET_PORT = rospy.get_param("~udp_port", 49152)

    rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, gps_callback)
    rospy.loginfo("Listening to /mavros/global_position/raw/fix and sending UDP packets...")
    rospy.spin()

if __name__ == "__main__":
    main()
