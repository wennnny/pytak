#!/usr/bin/env python3
import rospy
import socket
import struct
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Header

# === Initialization ===
rospy.init_node('udp_pose_receiver', anonymous=True)
UDP_PORT = rospy.get_param("~udp_port", 49157)
UDP_IP = rospy.get_param("~udp_ip", "0.0.0.0")

WAYPOINT_HEADER = 0xAF
OBSTACLE_HEADER = 0xB0
END_CODE = 0xC0
PACKET_SIZE = 28

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(1.0)

rospy.loginfo(f"Listening for UDP pose data on {UDP_IP}:{UDP_PORT}...")

pub_waypoint = rospy.Publisher('/waypoint/udp_global_position', PoseArray, queue_size=10)
pub_obstacle = rospy.Publisher('/obstacle/udp_global_position', PoseArray, queue_size=10)

waypoint_buffer = []
obstacle_buffer = []

def pose_from_latlonhae(lat, lon, hae):
    pose = Pose()
    pose.position.x = lat
    pose.position.y = lon
    pose.position.z = hae
    pose.orientation.w = 1.0
    return pose

rate = rospy.Rate(100)

try:
    while not rospy.is_shutdown():
        try:
            data, addr = sock.recvfrom(1024)
        except socket.timeout:
            continue
        except Exception as e:
            rospy.logerr(f"Socket error: {e}")
            continue

        try:
            if len(data) != PACKET_SIZE:
                rospy.logwarn(f"Invalid packet size: {len(data)}")
                continue

            header, length, index, lat, lon, hae, endcode = struct.unpack('<BBBdddB', data)

            if length != 25:
                rospy.logwarn(f"Unexpected length: {length}")
                continue

            pose = pose_from_latlonhae(lat, lon, hae)

            if header == WAYPOINT_HEADER:
                waypoint_buffer.append((index, pose))
                rospy.loginfo(f"[WAYPOINT] idx={index} lat={lat:.6f}, lon={lon:.6f}, hae={hae:.2f}")

                if endcode == END_CODE:
                    sorted_poses = [p for _, p in sorted(waypoint_buffer, key=lambda x: x[0])]
                    pose_array = PoseArray()
                    pose_array.header = Header()
                    pose_array.header.stamp = rospy.Time.now()
                    pose_array.header.frame_id = "map"
                    pose_array.poses = sorted_poses

                    pub_waypoint.publish(pose_array)
                    rospy.loginfo(f"Published {len(sorted_poses)} waypoints to /waypoint/udp_global_position")
                    waypoint_buffer = []

            elif header == OBSTACLE_HEADER:
                obstacle_buffer.append((index, pose))
                rospy.loginfo(f"[OBSTACLE] idx={index} lat={lat:.6f}, lon={lon:.6f}, hae={hae:.2f}")

                if endcode == END_CODE:
                    sorted_poses = [p for _, p in sorted(obstacle_buffer, key=lambda x: x[0])]
                    pose_array = PoseArray()
                    pose_array.header = Header()
                    pose_array.header.stamp = rospy.Time.now()
                    pose_array.header.frame_id = "map"
                    pose_array.poses = sorted_poses

                    pub_obstacle.publish(pose_array)
                    rospy.loginfo(f"Published {len(sorted_poses)} obstacles to /obstacle/udp_global_position")
                    obstacle_buffer = []

            else:
                rospy.logwarn(f"Unknown header: {header}")

        except Exception as e:
            rospy.logerr(f"Exception while parsing packet: {e}")

        rate.sleep()

finally:
    sock.close()
    rospy.loginfo("Socket closed. Node exiting.")
