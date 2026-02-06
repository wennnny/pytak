#!/usr/bin/env python3

import rospy
import socket
import json

from std_msgs.msg import String, Float64
from sensor_msgs.msg import NavSatFix


class UDPTopicReceiver:

    def __init__(self):
        rospy.init_node("udp_topic_receiver")

        # ========= PARAMETERS =========
        bind_ip   = rospy.get_param("~ip", "0.0.0.0")
        bind_port = rospy.get_param("~port", 15000)

        # ========= ROS Publishers =========
        self.status_pub  = rospy.Publisher("/vessel_status_text", String, queue_size=5)
        self.speed_pub   = rospy.Publisher("/kc1400/speed_knots", Float64, queue_size=5)
        self.heading_pub = rospy.Publisher("/kc1400/heading", Float64, queue_size=5)
        self.gps_pub     = rospy.Publisher("/kc1400/navsat_fix", NavSatFix, queue_size=5)

        # ========= UDP Socket =========
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((bind_ip, bind_port))
        self.sock.setblocking(False)

        rospy.loginfo("Listening UDP on %s:%d", bind_ip, bind_port)

    # =====================================
    # Main receive loop
    # =====================================

    def spin(self):
        rate = rospy.Rate(500)  # fast polling, low latency

        while not rospy.is_shutdown():

            try:
                data, _ = self.sock.recvfrom(65535)
                self.process_packet(data)

            except BlockingIOError:
                pass

            rate.sleep()

    # =====================================
    # Packet Handler
    # =====================================

    def process_packet(self, data):

        try:
            pkt = json.loads(data.decode("utf-8"))
        except Exception:
            rospy.logwarn_throttle(2, "Invalid UDP JSON")
            return

        # -------- Status Text --------
        if "status" in pkt:
            self.status_pub.publish(String(pkt["status"]))

        # -------- Speed --------
        if "speed_knots" in pkt:
            self.speed_pub.publish(Float64(pkt["speed_knots"]))

        # -------- Heading --------
        if "heading" in pkt:
            self.heading_pub.publish(Float64(pkt["heading"]))

        # -------- GPS --------
        if "gps" in pkt:
            g = pkt["gps"]

            msg = NavSatFix()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "gps"

            msg.latitude  = g.get("lat", 0.0)
            msg.longitude = g.get("lon", 0.0)
            msg.altitude  = g.get("alt", 0.0)

            cov = g.get("cov", [0]*9)
            if len(cov) == 9:
                msg.position_covariance = cov

            self.gps_pub.publish(msg)


# =========================================

if __name__ == "__main__":
    receiver = UDPTopicReceiver()
    receiver.spin()
