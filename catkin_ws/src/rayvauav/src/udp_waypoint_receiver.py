#!/usr/bin/env python3
import rospy
import socket
import struct
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Header

UDP_PORT = rospy.get_param("/waypoint_udp_port", 49157)
UDP_IP = rospy.get_param("/waypoint_udp_ip", "0.0.0.0")

HEADER_EXPECTED = 0xAF
END_CODE = 0xC0
PACKET_SIZE = 28

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

rospy.init_node('udp_waypoint_receiver', anonymous=True)
pub_posearray = rospy.Publisher('/waypoint/udp_global_position', PoseArray, queue_size=10)
rospy.loginfo(f"Listening for UDP waypoints on {UDP_IP}:{UDP_PORT}...")

waypoint_buffer = []

def pose_from_latlonhae(lat, lon, hae):
    pose = Pose()
    pose.position.x = lat
    pose.position.y = lon
    pose.position.z = hae
    pose.orientation.w = 1.0  # assume no rotation
    return pose

rate = rospy.Rate(100)

while not rospy.is_shutdown():
    try:
        data, addr = sock.recvfrom(1024)
        if len(data) != PACKET_SIZE:
            rospy.logwarn(f"Invalid packet size: {len(data)}")
            continue

        header, length, index, lat, lon, hae, endcode = struct.unpack('<BBBdddB', data)

        if header != HEADER_EXPECTED or length != 25:
            rospy.logwarn(f"Unexpected packet header/length: {header}, {length}")
            continue

        pose = pose_from_latlonhae(lat, lon, hae)
        waypoint_buffer.append((index, pose))
        rospy.loginfo(f"[WAYPOINT] Received idx={index} lat={lat:.6f}, lon={lon:.6f}, hae={hae:.2f}")

        if endcode == END_CODE:
            sorted_poses = [p for _, p in sorted(waypoint_buffer, key=lambda x: x[0])]
            pose_array = PoseArray()
            pose_array.header = Header()
            pose_array.header.stamp = rospy.Time.now()
            pose_array.header.frame_id = "map"
            pose_array.poses = sorted_poses

            pub_posearray.publish(pose_array)
            rospy.loginfo(f"Published {len(sorted_poses)} waypoints to /waypoint/udp_global_position")
            waypoint_buffer = []

    except Exception as e:
        rospy.logerr(f"Exception in UDP waypoint receiver: {e}")

    rate.sleep()
