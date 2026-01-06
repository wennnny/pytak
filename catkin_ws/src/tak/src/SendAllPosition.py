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
        self.latest_gps = {"lat": None, "lon": None, "hae": 99}
        self.latest_obstacles = []
        self.latest_goals = []
        
        rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.gps_callback)
        rospy.Subscriber("/detected_obstacles/gps/pose_array", PoseArray, self.obstacle_callback)
        rospy.Subscriber("/goal/gps/pose_array", PoseArray, self.goal_callback) # 新增目標點訂閱
        rospy.Subscriber("/start_point/gps/pose_array", PoseArray, self.start_callback)

    def start_callback(self, msg: PoseArray):
        self.latest_starts = msg.poses

    def goal_callback(self, msg: PoseArray):
        self.latest_goals = msg.poses

    def gps_callback(self, data):
        self.latest_gps["lat"] = data.latitude
        self.latest_gps["lon"] = data.longitude
        self.latest_gps["hae"] = 99

    def obstacle_callback(self, msg: PoseArray):
        self.latest_obstacles = msg.poses

    async def run(self):
        while not rospy.is_shutdown():
            
            if self.latest_gps["lat"] is not None:
                await self.put_queue(cot_utils.generate_gps_cot(self.latest_gps))

            for idx, pose in enumerate(self.latest_obstacles):
                await self.put_queue(cot_utils.generate_obstacle_cot(idx, pose.position.x, pose.position.y, pose.position.z))

            for idx, pose in enumerate(self.latest_goals):
                goal_cot = cot_utils.generate_goal_cot(
                    idx, 
                    pose.position.x, # 緯度
                    pose.position.y, # 經度
                    pose.position.z
                )
                await self.put_queue(goal_cot)

            for idx, pose in enumerate(self.latest_starts):
                start_cot = cot_utils.generate_start_cot(idx, pose.position.x, pose.position.y, pose.position.z)
                await self.put_queue(start_cot)

            await asyncio.sleep(1)

class MyReceiver(pytak.QueueWorker):
    """處理從 TAK Server 接收到的資訊 (例如其他隊友的位置)"""
    async def run(self):
        while True:
            data = await self.queue.get()
            # self._logger.info(f"[Received] {data.decode()}")
            await asyncio.sleep(0.1)

async def main():
    rospy.init_node('tak_bridge_node', anonymous=True)

    config = ConfigParser()
    config["mycottool"] = {
        "COT_URL": "tls://140.113.148.80:8089",
        "PYTAK_TLS_CLIENT_CERT": "/home/moos-dawg/Downloads/pytak/catkin_ws/key/argtest2_cert.pem",
        "PYTAK_TLS_CLIENT_KEY": "/home/moos-dawg/Downloads/pytak/catkin_ws/key/argtest2_key.pem",
        "PYTAK_TLS_CA_CERT": "/home/moos-dawg/Downloads/pytak/catkin_ws/key/argtest2-trusted.pem",
        "PYTAK_TLS_DONT_CHECK_HOSTNAME": "1",
        "PYTAK_TLS_DONT_VERIFY": "1"
    }
    config = config["mycottool"]

    clitool = pytak.CLITool(config)
    await clitool.setup()

    clitool.add_tasks(set([
        MySerializer(clitool.tx_queue, config), 
        MyReceiver(clitool.rx_queue, config)
    ]))

    await clitool.run()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
