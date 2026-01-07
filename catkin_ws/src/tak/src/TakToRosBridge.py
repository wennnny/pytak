#!/usr/bin/env python3

import asyncio
import xml.etree.ElementTree as ET
import pytak
import rospy
import sys
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Header, String
from configparser import ConfigParser

class TakToRosReceiver(pytak.QueueWorker):
    def __init__(self, queue, config):
        super().__init__(queue, config)
        
        # --- ROS 發佈者 ---
        self.pub_waypoint = rospy.Publisher('/waypoint/global_position', PoseArray, queue_size=10)
        self.pub_obstacle = rospy.Publisher('/obstacle/global_position', PoseArray, queue_size=10)
        self.pub_moos = rospy.Publisher('/cot_chat/message', String, queue_size=10)
        
        # 座標緩存
        self.waypoint_buffer = {}
        self.obstacle_buffer = {}

    async def handle_data(self, data):
        """解析 XML 並根據類型發佈到對應的 ROS Topic"""
        try:
            # 確保資料格式正確
            if isinstance(data, bytes):
                xml_str = data.decode('utf-8')
            else:
                xml_str = data

            root = ET.fromstring(xml_str)
            
            # 1. 檢查是否為聊天訊息 (GeoChat, type="b-t-f")
            cot_type = root.get("type", "")
            if cot_type == "b-t-f":
                self._process_chat_message(root)
                return

            # 2. 取得位置資訊 (point)
            point_el = root.find("point")
            if point_el is None: return
            
            lat_main = float(point_el.get("lat"))
            lon_main = float(point_el.get("lon"))
            hae_main = float(point_el.get("hae", 0.0))

            contact = root.find("./detail/contact")
            callsign = contact.get("callsign", "Unknown") if contact is not None else "Unknown"
            
            # 3. 處理 "route" 指令 (u-d-r 或 u-d-f)
            if callsign.lower() == "route":
                self._process_route_event(root, lat_main, lon_main, hae_main)
                return

            # 4. 處理單一標記 (waypoint / obstacle)
            pose = Pose()
            pose.position.x = lat_main
            pose.position.y = lon_main
            pose.position.z = hae_main

            if callsign.startswith("waypoint"):
                self._process_single_buffer(callsign, pose, self.waypoint_buffer, self.pub_waypoint, "Waypoints")
            elif callsign.startswith("obstacle"):
                self._process_single_buffer(callsign, pose, self.obstacle_buffer, self.pub_obstacle, "Obstacles")

        except Exception as e:
            rospy.logerr(f"[TakToRos] Error parsing CoT: {e}")

    def _process_chat_message(self, root):
        """解析聊天訊息並執行指令替換"""
        remarks = root.find("./detail/remarks")
        if remarks is not None and remarks.text:
            msg_text = remarks.text.strip().lower()
            
            command_map = {
                "moos_d": "DEPLOY",
                "moos_p": "PAUSE",
                "moos_r": "RETURN"
            }
            
            if msg_text in command_map:
                final_cmd = command_map[msg_text]
                self.pub_moos.publish(String(data=final_cmd))
                rospy.loginfo(f"[TakToRos] Command Mapped: {msg_text} -> {final_cmd}")
            elif msg_text.startswith("moos"):
                self.pub_moos.publish(String(data=msg_text.upper()))
                rospy.loginfo(f"[TakToRos] Forwarded: {msg_text.upper()}")

    def _process_route_event(self, root, lat_main, lon_main, hae_main):
        pose_array = PoseArray(header=Header(stamp=rospy.Time.now(), frame_id="map"))
        p0 = Pose()
        p0.position.x, p0.position.y, p0.position.z = lat_main, lon_main, hae_main
        pose_array.poses.append(p0)
        
        for link in root.findall("./detail/link"):
            p_str = link.get("point")
            if p_str:
                coords = p_str.split(',')
                if len(coords) >= 2:
                    p = Pose()
                    p.position.x, p.position.y, p.position.z = float(coords[0]), float(coords[1]), hae_main
                    pose_array.poses.append(p)
        
        if pose_array.poses:
            self.pub_waypoint.publish(pose_array)

    def _process_single_buffer(self, callsign, pose, buffer_dict, publisher, label):
        is_last = callsign.endswith("!")
        clean_name = callsign.rstrip("!")
        buffer_dict[clean_name] = pose
        if is_last:
            pa = PoseArray(header=Header(stamp=rospy.Time.now(), frame_id="map"))
            pa.poses = [buffer_dict[k] for k in sorted(buffer_dict.keys())]
            publisher.publish(pa)
            buffer_dict.clear()

    async def run(self):
        rospy.loginfo("TakToRosBridge Receiver Task started...")
        while not rospy.is_shutdown():
            try:
                # 使用 wait_for 避免 queue.get() 無限阻塞導致無法響應 rospy.is_shutdown
                data = await asyncio.wait_for(self.queue.get(), timeout=1.0)
                await self.handle_data(data)
            except asyncio.TimeoutError:
                continue
            except Exception as e:
                rospy.logerr(f"[TakToRos] Receiver Loop Error: {e}")

async def main():
    # 確保 ROS Node 初始化
    if not rospy.is_shutdown():
        rospy.init_node('tak_to_ros_bridge', anonymous=True)

    # TAK Server 設定
    config = ConfigParser()
    config["mycottool"] = {
        "COT_URL": "tls://140.113.148.80:8089",
        "PYTAK_TLS_CLIENT_CERT": "/home/moos-dawg/Downloads/pytak/catkin_ws/key/argtest2_cert.pem",
        "PYTAK_TLS_CLIENT_KEY": "/home/moos-dawg/Downloads/pytak/catkin_ws/key/argtest2_key.pem",
        "PYTAK_TLS_CA_CERT": "/home/moos-dawg/Downloads/pytak/catkin_ws/key/argtest2-trusted.pem",
        "PYTAK_TLS_DONT_CHECK_HOSTNAME": "1",
        "PYTAK_TLS_DONT_VERIFY": "1"
    }
    
    # 修正：直接傳遞 Section Proxy
    cot_config = config["mycottool"]
    clitool = pytak.CLITool(cot_config)
    
    try:
        await clitool.setup()
        # 註冊接收端任務
        clitool.add_tasks(set([TakToRosReceiver(clitool.rx_queue, cot_config)]))
        await clitool.run()
    except Exception as e:
        rospy.logerr(f"[TakToRos] Critical failure in main: {e}")

if __name__ == "__main__":
    try:
        # 使用 asyncio.run 啟動
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Process exited with error: {e}")

