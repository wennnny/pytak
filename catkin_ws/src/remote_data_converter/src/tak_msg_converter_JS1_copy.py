#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray, Pose

# ===== 固定 Home Position 原點設定 =====
LAT_ORIGIN = 22.989664
LON_ORIGIN = 120.15567


class TakMsgConverter:
    def __init__(self):
        rospy.init_node("tak_msg_converter", anonymous=True)
        rospy.loginfo("TAK Message Converter JS1 Node Initialized")

        ##### Subscriber #####
        # 1) Detected Obstacles，相對於船的相對位置 (local frame)
        #    來源：/obstacle_poses_array
        rospy.Subscriber("/obstacle_poses_array", PoseArray, self.obs_callback)
        self.obs_marker_pose = None

        # 2) 單點 Goal Pose（local）
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)
        self.goal_pose = None

        # 3) Local Goal Pose List Array（local）
        rospy.Subscriber("/pinned_pose_array", PoseArray, self.goal_list_callback)
        self.goal_list_array = None

        # 4) Start Point Pose Array（看起來是 GPS → local）
        rospy.Subscriber("/start_point_arr", PoseArray, self.start_point_callback)
        self.start_point_pose_array = None

        # 5) 從 TAK 來的 Goal GPS List（已經是 global GPS PoseArray）
        rospy.Subscriber("/waypoint/global_position", PoseArray, self.goal_gps_callback)

        # 6) 從 TAK 來的 Virtual Obstacles GPS List
        rospy.Subscriber("/obstacle/global_position", PoseArray, self.virtual_obstacles_callback)

        ##### Publisher #####
        # A) Publish Detected Obstacles GPS List（給 TAK，用 PoseArray）
        self.obstacles_gps_pub = rospy.Publisher(
            "/detected_obstacles/gps/pose_array", PoseArray, queue_size=10
        )

        # B) Publish 單一 Goal GPS Pose（給 TAK）
        self.goal_gps_pub = rospy.Publisher("/goal/gps/pose_array", PoseArray, queue_size=10)

        # C) Publish Goal List GPS PoseArray（給 TAK）
        self.goal_list_gps_pub = rospy.Publisher("/goal_list/gps/pose_array", PoseArray, queue_size=10)

        # D) Publish Start Point PoseArray（local frame）
        self.start_point_pub = rospy.Publisher("/start_point/gps/pose_array", PoseArray, queue_size=10)

        # E) Publish Goal pose array（pinned_pose_array 本身）
        self.pinned_array_pub = rospy.Publisher("/pinned_pose_array", PoseArray, queue_size=10)

        # F) Publish Virtual Obstacles PoseArray（local frame）
        self.buoy_pose_array_pub = rospy.Publisher("/buoy_pose_array", PoseArray, queue_size=10)

        # Timer
        self.timer = rospy.Timer(rospy.Duration(1.0), self.timer_callback)

        # 使用固定 origin
        self.origin = (LAT_ORIGIN, LON_ORIGIN)
        rospy.loginfo(f"Using fixed origin: lat={LAT_ORIGIN}, lon={LON_ORIGIN}")

    # ====== 座標轉換函式 ======
    def convert_local_to_gps(self, x, y):
        """
        Convert Local Pose (x, y) to GPS (lat, lon)
        using fixed origin (LAT_ORIGIN, LON_ORIGIN)
        """
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
        with fixed origin (LAT_ORIGIN, LON_ORIGIN)
        """
        R = 6378137.0
        lat0, lon0 = self.origin

        dLat = np.radians(lat - lat0)
        dLon = np.radians(lon - lon0)
        avgLat = np.radians(lat0)

        x = R * dLon * np.cos(avgLat)
        y = R * dLat
        return x, y

    # ====== Callbacks ======
    def obs_callback(self, msg: PoseArray):
        """
        Detected Obstacles PoseArray (local，相對於船) → GPS PoseArray (/detected_obstacles/gps/pose_array)
        來源 topic: /obstacle_poses_array
        """
        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = "map"

        for pose in msg.poses:
            x = pose.position.x
            y = pose.position.y
            lat, lon = self.convert_local_to_gps(x, y)

            new_pose = Pose()
            new_pose.position.x = lat
            new_pose.position.y = lon
            new_pose.position.z = 0.0
            new_pose.orientation.w = 1.0
            pose_array.poses.append(new_pose)

        self.obs_marker_pose = pose_array
        rospy.loginfo(f"Converted {len(pose_array.poses)} detected obstacles from local to GPS")

    def goal_callback(self, data: PoseStamped):
        """單一 Goal Pose（local）→ GPS，存起來給 timer 定期發"""
        gps_pos = self.convert_local_to_gps(data.pose.position.x, data.pose.position.y)
        lat, lon = gps_pos
        self.goal_pose = [lat, lon]
        rospy.loginfo(f"Goal GPS coordinates: {self.goal_pose}")

    def goal_list_callback(self, data: PoseArray):
        """Local Goal PoseArray → GPS PoseArray（發到 /goal_list/gps/pose_array）"""
        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = "map"

        for pose in data.poses:
            lat, lon = self.convert_local_to_gps(pose.position.x, pose.position.y)
            new_pose = Pose()
            new_pose.position.x = lat
            new_pose.position.y = lon
            new_pose.position.z = 0.0
            new_pose.orientation.w = 1.0
            pose_array.poses.append(new_pose)

        self.goal_list_array = pose_array
        self.goal_list_gps_pub.publish(pose_array)
        rospy.loginfo(f"Published goal list poses in GPS coordinates: {len(pose_array.poses)}")

    def start_point_callback(self, data: PoseArray):
        """
        Start Point PoseArray（看起來 data 是 GPS）
        轉為 local (x, y) 後存起來再由 timer 發到 /start_point/gps/pose_array
        """
        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = "map"

        for pose in data.poses:
            x, y = self.convert_gps_to_local(pose.position.x, pose.position.y)
            new_pose = Pose()
            new_pose.position.x = x
            new_pose.position.y = y
            new_pose.position.z = 0.0
            new_pose.orientation.w = 1.0
            pose_array.poses.append(new_pose)

        self.start_point_pose_array = pose_array
        rospy.loginfo(f"Converted start point poses to local frame: {len(pose_array.poses)}")

    def timer_callback(self, event):
        """Timer 定期發佈累積的目標 / 障礙物等資料"""

        # 單一 Goal GPS
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

        # Detected obstacles GPS PoseArray
        if self.obs_marker_pose is not None:
            self.obstacles_gps_pub.publish(self.obs_marker_pose)
            rospy.loginfo("Published detected obs GPS coordinates")
        else:
            rospy.logwarn("No detected obs pose set yet!")

        # Start point PoseArray（local frame）
        if self.start_point_pose_array is not None:
            self.start_point_pub.publish(self.start_point_pose_array)
            rospy.loginfo("Published start point local poses")
        else:
            rospy.logwarn("No start point pose set yet!")

        # 如果要定期重發 goal_list_array，可以打開這段
        # if self.goal_list_array is not None:
        #     self.goal_list_gps_pub.publish(self.goal_list_array)
        #     rospy.loginfo(f"Published goal list GPS coordinates")
        # else:
        #     rospy.logwarn("No goal list pose set yet!")

    def goal_gps_callback(self, data: PoseArray):
        """
        從 TAK 來的 Goal GPS PoseArray (/waypoint/global_position)
        → 轉成 local PoseArray，發到 /pinned_pose_array（原本用來規劃路徑）
        """
        if len(data.poses) == 0:
            rospy.logwarn("No GPS data received in goal_gps_callback!")
            return

        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = "map"

        for pose in data.poses:
            x, y = self.convert_gps_to_local(pose.position.x, pose.position.y)
            new_pose = Pose()
            new_pose.position.x = x
            new_pose.position.y = y
            new_pose.position.z = 0.0
            new_pose.orientation.w = 1.0
            pose_array.poses.append(new_pose)

        self.pinned_array_pub.publish(pose_array)
        rospy.loginfo(f"Published goal GPS poses in pinned_array (local): {len(pose_array.poses)}")

    def virtual_obstacles_callback(self, data: PoseArray):
        """
        從 TAK 來的 Virtual Obstacles GPS PoseArray (/obstacle/global_position)
        → 轉成 local PoseArray，發到 /buoy_pose_array
        """
        if len(data.poses) == 0:
            rospy.logwarn("No Virtual Obstacles data received!")
            return

        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = "map"

        for pose in data.poses:
            x, y = self.convert_gps_to_local(pose.position.x, pose.position.y)
            new_pose = Pose()
            new_pose.position.x = x
            new_pose.position.y = y
            new_pose.position.z = 0.0
            new_pose.orientation.w = 1.0
            pose_array.poses.append(new_pose)

        self.buoy_pose_array_pub.publish(pose_array)
        rospy.loginfo(f"Published virtual obstacles poses in buoy_pose_array (local): {len(pose_array.poses)}")


if __name__ == "__main__":
    try:
        TakMsgConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("TAK Message Converter Node interrupted.")
