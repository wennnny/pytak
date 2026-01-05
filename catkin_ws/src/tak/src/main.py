#!/usr/bin/env python3

import asyncio
from configparser import ConfigParser
import xml.etree.ElementTree as ET
import math

import pytak
import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from std_msgs.msg import Header

from cot_utils import generate_gps_cot, generate_goal_cot, generate_obstacle_cot

print("[tak_ros_main] file loaded")

# === 原點設定（用來計算 local position） ===
LAT_ORIGIN = 22.989664
LON_ORIGIN = 120.15567
R_EARTH = 6378137.0  # m，地球半徑
LAT0_RAD = math.radians(LAT_ORIGIN)

# === ROS Topics ===
GPS_TOPIC       = "/gazebo/js/gps_pose"
GOAL_TOPIC      = "/goal/gps/pose_array"                 # ROS -> TAK
OBSTACLE_TOPIC  = "/detected_obstacles/gps/pose_array"   # ROS -> TAK

# TAK 來的 waypoint / obstacle 轉成 PoseArray 後，要發到哪裡
WAYPOINT_POSEARRAY_TOPIC       = "/waypoint/global_position"
WAYPOINT_LOCAL_POSEARRAY_TOPIC = "/waypoint/local_position"   # ⭐ 新增：相對位置
OBSTACLE_POSEARRAY_TOPIC       = "/obstacle/global_position"
0
# === TAK 連線設定 ===
TAK_CFG = {
    "COT_URL": "tls://140.113.148.80:8089",
    "PYTAK_TLS_CLIENT_CERT": "/home/moos-dawg/Downloads/pytak/catkin_ws/files_WAN/argtest2_cert.pem",
    "PYTAK_TLS_CLIENT_KEY": "/home/moos-dawg/Downloads/pytak/catkin_ws/files_WAN/argtest2_key.pem",
    "PYTAK_TLS_CA_CERT": "/home/moos-dawg/Downloads/pytak/catkin_ws/files_WAN/argtest2-trusted.pem",
    "PYTAK_TLS_DONT_CHECK_HOSTNAME": "1",
    "PYTAK_TLS_DONT_VERIFY": "1",
}

# === 全域暫存 ===
latest_gps      = {"lat": None, "lon": None, "hae": None}
latest_goal     = {"msg": None}
latest_obstacle = {"msg": None}

# TAK → PoseArray buffer
cot_waypoint_buffer = []   # list of (lat, lon, hae)
cot_obstacle_buffer = []   # list of (lat, lon, hae)

# ROS publishers（TAK → ROS）
pub_waypoint_posearray       = None
pub_waypoint_local_posearray = None   # ⭐ 新增：發 local position 用
pub_obstacle_posearray       = None


# ============================================================
# 小工具：經緯度 → local XY（m）
# ============================================================
def latlon_to_local(lat: float, lon: float):
    """
    以 LatOrigin, LonOrigin 為原點，計算相對位置（公尺）
    x: 東向位移（+x 往東）
    y: 北向位移（+y 往北）
    """
    dlat = math.radians(lat - LAT_ORIGIN)
    dlon = math.radians(lon - LON_ORIGIN)
    x = dlon * math.cos(LAT0_RAD) * R_EARTH  # East
    y = dlat * R_EARTH                       # North
    return x, y


# ============================================================
# ROS Callbacks（ROS → TAK）
# ============================================================
# def gps_callback(msg: NavSatFix):
#     latest_gps["lat"] = msg.latitude
#     latest_gps["lon"] = msg.longitude
#     latest_gps["hae"] = msg.altitude
#     print(
#         f"[gps_callback] GPS lat={msg.latitude:.7f}, "
#         f"lon={msg.longitude:.7f}, hae={msg.altitude:.2f}"
#     )

def gps_callback(msg: PoseStamped):
    lat = msg.pose.position.x
    lon = msg.pose.position.y
    hae = msg.pose.position.z  # 目前是 0，如果之後要高度可以改

    latest_gps["lat"] = lat
    latest_gps["lon"] = lon
    latest_gps["hae"] = hae

    print(
        f"[gps_callback] JS GPS lat={lat:.7f}, "
        f"lon={lon:.7f}, hae={hae:.2f}"
    )


def goal_callback(msg: PoseArray):
    latest_goal["msg"] = msg
    print(f"[goal_callback] got PoseArray: {len(msg.poses)} poses, stamp={msg.header.stamp}")


def obstacle_callback(msg: PoseArray):
    latest_obstacle["msg"] = msg
    print(f"[obstacle_callback] got PoseArray: {len(msg.poses)} poses, stamp={msg.header.stamp}")


# ============================================================
# ROS → TAK Workers
# ============================================================
class GPSWorker(pytak.QueueWorker):
    async def run(self):
        print(f"[GPSWorker] Started, topic={GPS_TOPIC}")
        while not rospy.is_shutdown():
            if latest_gps["lat"] is None:
                print("[GPSWorker] waiting for first GPS...")
                await asyncio.sleep(0.5)
                continue

            try:
                event_bytes = generate_gps_cot(latest_gps)
                print(
                    f"[GPSWorker] Send GPS CoT: "
                    f"lat={latest_gps['lat']:.7f}, "
                    f"lon={latest_gps['lon']:.7f}, "
                    f"hae={latest_gps['hae']:.2f}, "
                    f"len={len(event_bytes)}"
                )
                await self.put_queue(event_bytes)
            except Exception as exc:
                print(f"[GPSWorker] ERROR while generating/sending CoT: {exc}")

            await asyncio.sleep(1.0)  # 1 秒發一次


class GoalPoseArrayWorker(pytak.QueueWorker):
    def __init__(self, queue, config):
        super().__init__(queue, config)
        self._last_stamp = None

    async def run(self):
        print(f"[GoalPoseArrayWorker] Started, topic={GOAL_TOPIC}")
        while not rospy.is_shutdown():
            msg = latest_goal["msg"]
            if msg is None:
                print("[GoalPoseArrayWorker] waiting for first goal PoseArray...")
                await asyncio.sleep(0.5)
                continue

            stamp = msg.header.stamp
            if self._last_stamp is not None and stamp == self._last_stamp:
                await asyncio.sleep(0.2)
                continue

            self._last_stamp = stamp

            try:
                count = 0
                for idx, pose in enumerate(msg.poses, start=1):
                    lat = pose.position.x
                    lon = pose.position.y
                    hae = pose.position.z
                    event_bytes = generate_goal_cot(idx, lat, lon, hae)
                    await self.put_queue(event_bytes)
                    count += 1
                print(
                    f"[GoalPoseArrayWorker] Sent {count} goal CoT(s) "
                    f"for stamp={stamp}"
                )
            except Exception as exc:
                print(f"[GoalPoseArrayWorker] ERROR while sending goal CoTs: {exc}")

            await asyncio.sleep(0.1)


class ObstaclePoseArrayWorker(pytak.QueueWorker):
    def __init__(self, queue, config):
        super().__init__(queue, config)
        self._last_stamp = None

    async def run(self):
        print(f"[ObstaclePoseArrayWorker] Started, topic={OBSTACLE_TOPIC}")
        while not rospy.is_shutdown():
            msg = latest_obstacle["msg"]
            if msg is None:
                print("[ObstaclePoseArrayWorker] waiting for first obstacle PoseArray...")
                await asyncio.sleep(0.5)
                continue

            stamp = msg.header.stamp
            if self._last_stamp is not None and stamp == self._last_stamp:
                await asyncio.sleep(0.2)
                continue

            self._last_stamp = stamp

            try:
                count = 0
                for idx, pose in enumerate(msg.poses, start=1):
                    x = pose.position.x
                    y = pose.position.y
                    z = pose.position.z
                    event_bytes = generate_obstacle_cot(idx, x, y, z)
                    await self.put_queue(event_bytes)
                    count += 1
                print(
                    f"[ObstaclePoseArrayWorker] Sent {count} obstacle CoT(s) "
                    f"for stamp={stamp}"
                )
            except Exception as exc:
                print(f"[ObstaclePoseArrayWorker] ERROR while sending obstacle CoTs: {exc}")

            await asyncio.sleep(0.1)


# ============================================================
# TAK RX → 直接轉 PoseArray（global + local）
# ============================================================
def parse_cot(xml_data: str):
    """
    從 CoT XML 中取出：
    - type
    - callsign
    - lat / lon / hae
    """
    root = ET.fromstring(xml_data)

    event_type = root.get("type", "Unknown")

    point = root.find("point")
    lat = float(point.get("lat", 0)) if point is not None else 0.0
    lon = float(point.get("lon", 0)) if point is not None else 0.0
    hae = float(point.get("hae", 0)) if point is not None else 0.0

    contact = root.find("./detail/contact")
    callsign = contact.get("callsign", "Unknown") if contact is not None else "Unknown"

    return callsign, event_type, lat, lon, hae


def make_pose(lat: float, lon: float, hae: float) -> Pose:
    pose = Pose()
    pose.position.x = lat
    pose.position.y = lon
    pose.position.z = hae
    pose.orientation.w = 1.0
    return pose


def make_local_pose(lat: float, lon: float, hae: float) -> Pose:
    """
    用原點轉成 local XY（m），存成 Pose：
      position.x = 東向位移（m）
      position.y = 北向位移（m）
      position.z = hae（可以之後改成 0 也行）
    """
    x, y = latlon_to_local(lat, lon)
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = hae
    pose.orientation.w = 1.0
    return pose


class CotToPoseArrayWorker(pytak.QueueWorker):
    """
    從 TAK 收到 CoT：
      - callsign 以 "waypoint1", "waypoint2", ..., "waypointN!" 結尾：
          → 收集所有點，遇到含 "!" 的那一筆時：
              * 發 global PoseArray 到 /waypoint/global_position
              * 發 local  PoseArray 到 /waypoint/local_position
      - callsign 以 "obstacle1", "obstacle2", ..., "obstacleN!" 類似處理，
          → global PoseArray 到 /obstacle/global_position
    """

    async def run(self):
        global cot_waypoint_buffer, cot_obstacle_buffer
        print("[CotToPoseArrayWorker] Started (TAK RX → PoseArray topics)")
        while True:
            data = await self.queue.get()
            if data is None:
                await asyncio.sleep(0.01)
                continue

            try:
                xml_str = data.decode(errors="ignore")
                callsign, event_type, lat, lon, hae = parse_cot(xml_str)

                # 只處理 type = "b-m-p-s-m"
                if event_type != "b-m-p-s-m":
                    continue

                cs_lower = callsign.lower()

                # ---------- Waypoints ----------
                if cs_lower.startswith("waypoint"):
                    cot_waypoint_buffer.append((lat, lon, hae))
                    print(f"[CotToPoseArrayWorker][WAYPOINT] got {callsign}: lat={lat}, lon={lon}, hae={hae}")

                    # callsign 以 '!' 結尾代表一批 waypoint 結束
                    if callsign.endswith("!"):
                        # Global PoseArray
                        global_poses = [make_pose(la, lo, h) for (la, lo, h) in cot_waypoint_buffer]

                        pa_global = PoseArray()
                        pa_global.header = Header()
                        pa_global.header.stamp = rospy.Time.now()
                        pa_global.header.frame_id = "map"
                        pa_global.poses = global_poses

                        # Local PoseArray（相對原點）
                        local_poses = [make_local_pose(la, lo, h) for (la, lo, h) in cot_waypoint_buffer]

                        pa_local = PoseArray()
                        pa_local.header = Header()
                        pa_local.header.stamp = rospy.Time.now()
                        pa_local.header.frame_id = "local"  # 你也可以改成 "map" or "odom"
                        pa_local.poses = local_poses

                        if pub_waypoint_posearray is not None:
                            pub_waypoint_posearray.publish(pa_global)
                            print(
                                f"[CotToPoseArrayWorker] Published {len(global_poses)} waypoints "
                                f"to {WAYPOINT_POSEARRAY_TOPIC}"
                            )

                        if pub_waypoint_local_posearray is not None:
                            pub_waypoint_local_posearray.publish(pa_local)
                            print(
                                f"[CotToPoseArrayWorker] Published {len(local_poses)} local waypoints "
                                f"to {WAYPOINT_LOCAL_POSEARRAY_TOPIC}"
                            )

                        cot_waypoint_buffer = []

                # ---------- Obstacles ----------
                elif cs_lower.startswith("obstacle"):
                    cot_obstacle_buffer.append((lat, lon, hae))
                    print(f"[CotToPoseArrayWorker][OBSTACLE] got {callsign}: lat={lat}, lon={lon}, hae={hae}")

                    if callsign.endswith("!"):
                        poses = [make_pose(la, lo, h) for (la, lo, h) in cot_obstacle_buffer]

                        pa = PoseArray()
                        pa.header = Header()
                        pa.header.stamp = rospy.Time.now()
                        pa.header.frame_id = "map"
                        pa.poses = poses

                        if pub_obstacle_posearray is not None:
                            pub_obstacle_posearray.publish(pa)
                            print(
                                f"[CotToPoseArrayWorker] Published {len(poses)} obstacles "
                                f"to {OBSTACLE_POSEARRAY_TOPIC}"
                            )
                        cot_obstacle_buffer = []

            except Exception as e:
                print(f"[CotToPoseArrayWorker] ERROR while parsing/sending PoseArray: {e}")


# ============================================================
# 主 asyncio 邏輯（啟動 pytak workers）
# ============================================================
async def main_async():
    print("[main_async] starting (ROS↔TAK + TAK→PoseArray)")

    config = ConfigParser()
    config["mycottool"] = TAK_CFG
    cfg = config["mycottool"]

    print("[main_async] creating CLITool")
    clitool = pytak.CLITool(cfg)

    print("[main_async] calling clitool.setup()")
    await clitool.setup()
    print("[main_async] clitool.setup() done")

    print("[main_async] adding workers")
    clitool.add_tasks({
        GPSWorker(clitool.tx_queue, cfg),
        GoalPoseArrayWorker(clitool.tx_queue, cfg),
        ObstaclePoseArrayWorker(clitool.tx_queue, cfg),
        CotToPoseArrayWorker(clitool.rx_queue, cfg),
    })

    print("[main_async] calling clitool.run() (this will keep running)")
    await clitool.run()
    print("[main_async] clitool.run() returned (normally 不會到這裡)")


def init_ros():
    global pub_waypoint_posearray, pub_waypoint_local_posearray, pub_obstacle_posearray

    print("[init_ros] initializing ROS node & subscribers/publishers")
    rospy.init_node("tak_ros_node", anonymous=True)

    # ROS → TAK
    rospy.Subscriber(GPS_TOPIC,      PoseStamped,  gps_callback)
    rospy.Subscriber(GOAL_TOPIC,     PoseArray, goal_callback)
    rospy.Subscriber(OBSTACLE_TOPIC, PoseArray, obstacle_callback)

    # TAK → ROS（global + local）
    pub_waypoint_posearray       = rospy.Publisher(WAYPOINT_POSEARRAY_TOPIC,       PoseArray, queue_size=10)
    pub_waypoint_local_posearray = rospy.Publisher(WAYPOINT_LOCAL_POSEARRAY_TOPIC, PoseArray, queue_size=10)
    pub_obstacle_posearray       = rospy.Publisher(OBSTACLE_POSEARRAY_TOPIC,       PoseArray, queue_size=10)

    print("[init_ros] ROS subscribers & publishers registered")


# ============================================================
# __main__
# ============================================================
if __name__ == "__main__":
    print("[__main__] starting tak_ros_main")

    init_ros()

    try:
        asyncio.run(main_async())
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user. Shutting down.")
    except Exception as e:
        print(f"[__main__] main_async crashed: {e}")

