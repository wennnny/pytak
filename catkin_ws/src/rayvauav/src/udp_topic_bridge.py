#!/usr/bin/env python3
import rospy
import socket
import json
from std_msgs.msg import String, Float64
from sensor_msgs.msg import NavSatFix

class UDPBridge:
    def __init__(self):

        rospy.init_node("udp_topic_bridge")

        # ===== PARAMETERS =====
        self.target_ip   = rospy.get_param("~ip",   "10.24.1.22")
        self.target_port = rospy.get_param("~port", 15000)

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Latest state cache
        self.state = {
            "status": "",
            "speed_knots": 0.0,
            "heading": 0.0,
            "gps": {}
        }

        # Subscribers
        rospy.Subscriber("/vessel_status_text", String, self.status_cb)
        rospy.Subscriber("/kc1400/speed_knots", Float64, self.speed_cb)
        rospy.Subscriber("/kc1400/heading", Float64, self.heading_cb)
        rospy.Subscriber("/kc1400/navsat_fix", NavSatFix, self.gps_cb)

        rospy.loginfo("UDP bridge sending to %s:%d",
                      self.target_ip, self.target_port)

    # ========================
    # Callbacks
    # ========================

    def send(self):
        pkt = json.dumps(self.state).encode("utf-8")
        self.sock.sendto(pkt, (self.target_ip, self.target_port))

    def status_cb(self, msg):
        self.state["status"] = msg.data
        self.send()

    def speed_cb(self, msg):
        self.state["speed_knots"] = msg.data
        self.send()

    def heading_cb(self, msg):
        self.state["heading"] = msg.data
        self.send()

    def gps_cb(self, msg):
        self.state["gps"] = {
            "lat": msg.latitude,
            "lon": msg.longitude,
            "alt": msg.altitude,
            "cov": list(msg.position_covariance)
        }
        self.send()


if __name__ == "__main__":
    UDPBridge()
    rospy.spin()
