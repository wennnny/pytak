#!/usr/bin/env python3
import rospy
import numpy as np
from visualization_msgs.msg import MarkerArray
from mavros_msgs.msg import HomePosition
from geometry_msgs.msg import PoseStamped, PoseArray, Pose

class TakMsgConverter:
    def __init__(self):
        rospy.init_node("tak_msg_converter", anonymous=True)
        rospy.loginfo("TAK Message Converter Node Initialized")
        
        ##### Subscriber #####
        # Subscribe Home Position
        rospy.Subscriber("/mavros/home_position/home", HomePosition, self.home_callback)
        self.origin = None 

        # Subscribe Detected Obstacles pose in marker array
        rospy.Subscriber("/js/real/future/markers", MarkerArray, self.marker_callback) # /js/detr/future/markers
        self.obs_marker_pose = None
        
        # Subscribe Goal Point Pose
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)
        self.goal_pose = None

        # Subscribe Local Goal Pose List Array
        rospy.Subscriber("/pinned_pose_array", PoseArray, self.goal_list_callback)
        self.goal_list_array = None

        # Subscribe Start Point Pose Array
        rospy.Subscriber("/start_point_arr", PoseArray, self.start_point_callback)
        self.start_point_pose_array = None

        # Subscribe Goal GPS List from TAK
        rospy.Subscriber("/waypoint/udp_global_position", PoseArray, self.goal_gps_callback)

        # Subscribe Virtual Obstacles GPS List from TAK
        rospy.Subscriber("/obstacle/udp_global_position", PoseArray, self.virtual_obstacles_callback)

        ##### Publisher #####
        # Publish Detected Obstacles GPS List store in PoseArray (2TAK)
        self.obstacles_gps_pub = rospy.Publisher("/detected_obstacles/gps/pose_array", PoseArray, queue_size=10)

        # Publish Goal GPS Pose (2TAK)
        self.goal_gps_pub = rospy.Publisher("/goal/gps/pose_array", PoseArray, queue_size=10)   

        # Publish Goal GPS Pose (2TAK)
        self.goal_list_gps_pub = rospy.Publisher("/goal_list/gps/pose_array", PoseArray, queue_size=10)  

        # Publish Start Point Pose Array (2TAK)
        self.start_point_pub = rospy.Publisher("/start_point/gps/pose_array", PoseArray, queue_size=10)

        # Publish Goal pose array store in pinned_pose_array
        self.pinned_array_pub = rospy.Publisher("/pinned_pose_array", PoseArray, queue_size=10)

        # Publish Virtual Obstacles PoseArray store in buoy_pose_array
        self.buoy_pose_array_pub = rospy.Publisher("/buoy_pose_array", PoseArray, queue_size=10)

        # Timer
        self.timer = rospy.Timer(rospy.Duration(1.0), self.timer_callback)


    def home_callback(self, msg):
        """ Set MAVROS home_position to origin"""
        self.origin = (msg.geo.latitude, msg.geo.longitude)
        rospy.loginfo(f"Set origin: lat={msg.geo.latitude}, lon={msg.geo.longitude}")

    def convert_local_to_gps(self, x, y):
        """
        Convert Local Pose (x, y) to GPS (lat, lon)
        using MAVROS home_position as origin
        """
        if self.origin is None:
            rospy.logwarn("No Home Position received yet!")
            return None

        R = 6378137.0  # Earth radius in meters
        lat0, lon0 = self.origin

        dLat = y / R
        dLon = x / (R * np.cos(np.radians(lat0)))

        lat = lat0 + np.degrees(dLat)
        lon = lon0 + np.degrees(dLon)

        return lat, lon
    
    def convert_gps_to_local(self, lat, lon):
        """ 
        Convert GPS (lat, lon) to Local Pose (x, y)
        with MAVROS home_position as origin
        """
        if self.origin is None:
            rospy.logwarn("No Home Position received yet!")
            return None
        
        R = 6378137.0  # Earth radius in meters
        lat0, lon0 = self.origin
        dLat = np.radians(lat - lat0)
        dLon = np.radians(lon - lon0)
        avgLat = np.radians(lat0)
        x = R * dLon * np.cos(avgLat)
        y = R * dLat
        return x, y
    
    def marker_callback(self, msg):
        """ Callback for Detected Obstacles MarkerArray """
        if not self.origin:
            rospy.logwarn("No Home Position received yet!")
            return
        
        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = "map"

        for marker in msg.markers:            
            lat, lon = self.convert_local_to_gps(marker.pose.position.x, marker.pose.position.y)
            if lat is None or lon is None:
                continue
            
            pose = Pose()
            pose.position.x = lat
            pose.position.y = lon
            pose_array.poses.append(pose)

        self.obs_marker_pose = pose_array
        # rospy.loginfo(f"Published {len(pose_array.poses)} detected obstacles in GPS coordinates")

    def goal_callback(self, data):
        """ Convert Goal Pose to GPS coordinates and publish Float64MultiArray """
        if self.origin is None:
            rospy.logwarn("No Home Position received yet!")
            return

        gps_pos = self.convert_local_to_gps(data.pose.position.x, data.pose.position.y)
        if gps_pos:
            lat, lon = gps_pos
            self.goal_pose = [lat, lon]
            rospy.loginfo(f"Goal GPS coordinates: {self.goal_pose}")
    
    def goal_list_callback(self, data):
        """ Convert Goal Pose Array to GPS coordinates and publish PoseArray """
        if self.origin is None:
            rospy.logwarn("No Home Position received yet!")
            return

        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = "map"

        for pose in data.poses:
            gps_pos = self.convert_local_to_gps(pose.position.x, pose.position.y)
            if gps_pos:
                lat, lon = gps_pos
                new_pose = Pose()
                new_pose.position.x = lat
                new_pose.position.y = lon
                new_pose.position.z = 0.0
                new_pose.orientation.w = 1.0
                pose_array.poses.append(new_pose)
        
        self.goal_list_array = pose_array
        self.goal_list_gps_pub.publish(pose_array)
        rospy.loginfo(f"Published goal list poses in GPS coordinates: {len(pose_array.poses)}")

    def start_point_callback(self, data):
        """ Convert Start Point Pose Array to GPS coordinates and publish PoseArray """
        if self.origin is None:
            rospy.logwarn("No Home Position received yet!")
            return

        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = "map"

        for pose in data.poses:
            local_pos = self.convert_gps_to_local(pose.position.x, pose.position.y)
            if local_pos:
                new_pose = Pose()
                new_pose.position.x, new_pose.position.y = local_pos
                new_pose.position.z = 0.0
                new_pose.orientation.w = 1.0
                pose_array.poses.append(new_pose)

        self.start_point_pose_array = pose_array
        rospy.loginfo(f"Published start point poses in GPS coordinates: {len(pose_array.poses)}")

    def timer_callback(self, event):
        """ Timer callback to publish goal GPS coordinates """
        if self.goal_pose is not None:
            goal_gps_array = PoseArray()
            goal_gps_array.header.stamp = rospy.Time.now()
            goal_gps_array.header.frame_id = "map"

            pose = Pose()
            pose.position.x = self.goal_pose[0]
            pose.position.y = self.goal_pose[1]
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            goal_gps_array.poses.append(pose)

            self.goal_gps_pub.publish(goal_gps_array)
            rospy.loginfo(f"Published goal GPS coordinates: {self.goal_pose}")
        else:
            rospy.logwarn("No goal pose set yet!")
        
        if self.obs_marker_pose is not None:
            self.obstacles_gps_pub.publish(self.obs_marker_pose)
            rospy.loginfo(f"Published detected obs GPS coordinates")
        else:
            rospy.logwarn("No detected obs pose set yet!")

        if self.start_point_pose_array is not None:
            self.start_point_pub.publish(self.start_point_pose_array)
            rospy.loginfo(f"Published start point GPS coordinates")
        else:
            rospy.logwarn("No start point pose set yet!")

        # if self.goal_list_array is not None:
        #     self.goal_list_gps_pub.publish(self.goal_list_array)
        #     rospy.loginfo(f"Published goal list GPS coordinates")
        # else:
        #     rospy.logwarn("No goal list pose set yet!")

    def goal_gps_callback(self, data):
        """ Convert GPS coordinates to Local Pose and publish PoseArray """
        if self.origin is None:
            rospy.logwarn("No Home Position received yet!")
            return

        if len(data.poses) == 0:
            rospy.logwarn("No GPS data received!")
            return

        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = "map"

        for pose in data.poses:
            local_pos = self.convert_gps_to_local(pose.position.x, pose.position.y)
            if local_pos:
                new_pose = Pose()
                new_pose.position.x, new_pose.position.y = local_pos
                new_pose.position.z = 0.0
                new_pose.orientation.w = 1.0
                pose_array.poses.append(new_pose)

        # Publish PoseArray
        self.pinned_array_pub.publish(pose_array)
        rospy.loginfo(f"Published goal GPS poses in pinned_array: {len(pose_array.poses)}")

    def virtual_obstacles_callback(self, data):
        """ Convert Virtual Obstacles to Local Pose and publish PoseArray """
        if self.origin is None:
            rospy.logwarn("No Home Position received yet!")
            return

        if len(data.poses) == 0:
            rospy.logwarn("No Virtual Obstacles data received!")
            return

        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = "map"

        for pose in data.poses:
            local_pos = self.convert_gps_to_local(pose.position.x, pose.position.y)
            if local_pos:
                new_pose = Pose()
                new_pose.position.x, new_pose.position.y = local_pos
                new_pose.position.z = 0.0
                new_pose.orientation.w = 1.0
                pose_array.poses.append(new_pose)

        # Publish PoseArray
        self.buoy_pose_array_pub.publish(pose_array)
        rospy.loginfo(f"Published virtual obstacles poses in buoy_pose_array: {len(pose_array.poses)}")

if __name__ == "__main__":
    try:
        TakMsgConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("TAK Message Converter Node interrupted.")