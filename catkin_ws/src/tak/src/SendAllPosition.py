#!/usr/bin/env python3

import asyncio, socket, platform, json, os
import xml.etree.ElementTree as ET
import pytak
import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseArray
from configparser import ConfigParser
import cot_utils 

class MySerializer(pytak.QueueWorker):
    def __init__(self, queue, config):
        super().__init__(queue, config)
        # 初始化所有緩存變數
        self.latest_gps = {"lat": None, "lon": None, "hae": 99}
        self.latest_obstacles = []
        self.latest_goals = []
        self.latest_starts = []
        
        # ROS 訂閱
        rospy.Subscriber("/fix", NavSatFix, self.gps_callback)
        rospy.Subscriber("/detected_obstacles/gps/pose_array", PoseArray, self.obstacle_callback)
        rospy.Subscriber("/goal/gps/pose_array", PoseArray, self.goal_callback)
        rospy.Subscriber("/start_point/gps/pose_array", PoseArray, self.start_callback)

    def start_callback(self, msg: PoseArray):
        self.latest_starts = msg.poses

    def goal_callback(self, msg: PoseArray):
        self.latest_goals = msg.poses

    def gps_callback(self, data):
        self.latest_gps["lat"] = data.latitude
        self.latest_gps["lon"] = data.longitude

    def obstacle_callback(self, msg: PoseArray):
        self.latest_obstacles = msg.poses

    async def handle_data(self, data):
        """QueueWorker 要求的必要方法，此處留空"""
        pass

    async def run(self):
        self._logger.info("CoT Serializer loop started...")
        while not rospy.is_shutdown():
            try:
                if self.latest_gps["lat"] is None:
                    await asyncio.sleep(1)
                    continue

                # 使用固定的 DEVICE_UID 避免衝突
                v_uid = cot_utils.DEVICE_UID 
                
                # --- 1. 發送船隻 ---
                await self.put_queue(cot_utils.generate_gps_cot(self.latest_gps))
                
                # --- 2. 發送圓圈 (增加 Debug Log) ---
                self._logger.info(f"Sending Rings for {v_uid}")
                await self.put_queue(cot_utils.generate_range_ring_cot(self.latest_gps, 50, v_uid, "-65536"))
                await self.put_queue(cot_utils.generate_range_ring_cot(self.latest_gps, 100, v_uid, "-256"))

                # --- 3. 發送目標點與直線 ---
                for idx, pose in enumerate(self.latest_goals):
                    # 發送目標點點位 (3個參數)
                    goal_xml = cot_utils.generate_goal_cot(idx, pose.position.x, pose.position.y, pose.position.z)
                    await self.put_queue(goal_xml)
                    
                    # 發送直線 (4個參數：GPS, Pose, Index, UID)
                    line_xml = cot_utils.generate_line_cot(self.latest_gps, pose, idx, v_uid)
                    await self.put_queue(line_xml)

                # --- 4. 發送障礙物與起點 ---
                if self.latest_obstacles:
                    for idx, pose in enumerate(self.latest_obstacles):
                        await self.put_queue(cot_utils.generate_obstacle_cot(idx, pose.position.x, pose.position.y, pose.position.z))
                
                if self.latest_starts:
                    for idx, pose in enumerate(self.latest_starts):
                        await self.put_queue(cot_utils.generate_start_cot(idx, pose.position.x, pose.position.y, pose.position.z))

            except Exception as e:
                self._logger.error(f"FATAL Error in Serializer: {e}")

            await asyncio.sleep(1)

class MyReceiver(pytak.QueueWorker):
    async def handle_data(self, data):
        """實作此方法以避免 NotImplementedError"""
        pass

    async def run(self):
        while not rospy.is_shutdown():
            try:
                data = await asyncio.wait_for(self.queue.get(), timeout=1.0)
                await self.handle_data(data)
            except asyncio.TimeoutError:
                continue

async def main():
    rospy.init_node('tak_bridge_node', anonymous=True)

    config = ConfigParser()
    config["mycottool"] = {
        "COT_URL": "tls://192.168.133.16:8089",
        "PYTAK_TLS_CLIENT_CERT": "/home/moos-dawg/pytak/catkin_ws/JS/files/client_3_cert.pem",
        "PYTAK_TLS_CLIENT_KEY": "/home/moos-dawg/pytak/catkin_ws/JS/files/client_3_key.pem",
        "PYTAK_TLS_CA_CERT": "/home/moos-dawg/pytak/catkin_ws/JS/files/client_3-trusted.pem",
        "PYTAK_TLS_DONT_CHECK_HOSTNAME": "1",
        "PYTAK_TLS_DONT_VERIFY": "1"
    }
    cot_config = config["mycottool"]

    clitool = pytak.CLITool(cot_config)
    await clitool.setup()

    clitool.add_tasks(set([
        MySerializer(clitool.tx_queue, cot_config), 
        MyReceiver(clitool.rx_queue, cot_config)
    ]))

    await clitool.run()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
        
