#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Simple thrust + rudder controller with auto/manual, joystick, and sync_flag gating.

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, UInt8, Bool

class Node(object):
    def __init__(self):
        # ---- Parameters ----
        self.linear_scaling = rospy.get_param("~linear_scaling", 1.0)  # 推力縮放
        max_deg = rospy.get_param("~steering_angle", 24.0)             # 最大舵角（度）
        self.max_steer_rad = np.deg2rad(max_deg)                       # 轉成弧度

        # ---- State ----
        self.auto = 0                  # 0: manual, 1: auto
        self.sync_flag = False         # True 時暫停輸出
        # 欲輸出的訊息（由各來源回調寫入，timer 週期性發送）
        self.left_msg = Float32()
        self.right_msg = Float32()
        self.left_ang_msg = Float32()
        self.right_ang_msg = Float32()

        # ---- Publishers ----
        self.left_pub = rospy.Publisher("left_cmd", Float32, queue_size=10)
        self.right_pub = rospy.Publisher("right_cmd", Float32, queue_size=10)
        self.left_ang_pub = rospy.Publisher("left_ang_cmd", Float32, queue_size=10)
        self.right_ang_pub = rospy.Publisher("right_ang_cmd", Float32, queue_size=10)
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.control_mode_pub = rospy.Publisher("control_mode", UInt8, queue_size=10)
        self.remap_pub = rospy.Publisher("js/joy", Joy, queue_size=10)  # 保留給其他模組使用

        # ---- Subscribers ----
        self.sub_cmd = rospy.Subscriber("cmd_vel", Twist, self.cb_cmd, queue_size=1)
        self.sub_joy = rospy.Subscriber("joy", Joy, self.cb_joy, queue_size=1)
        self.sub_sync_flag = rospy.Subscriber("sync_flag", Bool, self.cb_sync_flag, queue_size=1)

        # 新增：自動模式的直接命令（-100 ~ 100）
        self.sub_desired_thrust = rospy.Subscriber(
            "/pub2ros/js/desired_thrust", Float32, self.cb_desired_thrust, queue_size=1
        )
        self.sub_desired_rudder = rospy.Subscriber(
            "/pub2ros/js/desired_rudder", Float32, self.cb_desired_rudder, queue_size=1
        )

        # ---- Timer ----
        self.timer = rospy.Timer(rospy.Duration(0.1), self.cb_publish)

    # ---------------- Core mapping helpers ----------------
    def apply_thrust_rudder(self, thrust_norm, rudder_rad):
        """
        將規範化推力 [-1,1] 與舵角 [rad] 寫入輸出訊息（雙馬達同推力，左右舵角相同）
        """
        thrust_norm = float(np.clip(thrust_norm, -1.0, 1.0))
        rudder_rad = float(np.clip(rudder_rad, -self.max_steer_rad, self.max_steer_rad))

        self.left_msg.data = thrust_norm * self.linear_scaling
        self.right_msg.data = thrust_norm * self.linear_scaling
        self.left_ang_msg.data = rudder_rad
        self.right_ang_msg.data = rudder_rad

    def t2t_from_cmdvel(self, x, z):
        """
        從 cmd_vel 轉換：只用 linear.x 當推力、angular.z 當方向（舵角）。
        - x ∈ [-1,1] 映射到 推力 ∈ [-1,1]
        - z ∈ [-1,1] 映射到 舵角 ∈ [-max, +max]（弧度）
        """
        x = float(np.clip(x, -1.0, 1.0))
        z = float(np.clip(z, -1.0, 1.0))

        # 原本的方向對應（左為正z -> 負號留著）
        rudder = -z * self.max_steer_rad

        # ★重點：若在倒車（x<0），把舵角反向，維持「左打=船頭左」的直覺
        if x < 0.0:
            rudder = -rudder

        self.apply_thrust_rudder(thrust_norm=x, rudder_rad=rudder)

    # ---------------- Callbacks ----------------
    def cb_publish(self, _event):
        # 同步旗標：為 True 時不輸出致動器命令
        if not self.sync_flag:
            self.left_pub.publish(self.left_msg)
            self.right_pub.publish(self.right_msg)
            self.left_ang_pub.publish(self.left_ang_msg)
            self.right_ang_pub.publish(self.right_ang_msg)

        # 模式通報：7=auto, 6=manual
        mode_msg = UInt8()
        mode_msg.data = 7 if self.auto else 6
        self.control_mode_pub.publish(mode_msg)

    def cb_sync_flag(self, msg):
        self.sync_flag = bool(msg.data)

    def cb_cmd(self, msg):
        # 只有自動模式才吃外部 cmd_vel
        if self.auto:
            self.t2t_from_cmdvel(msg.linear.x, msg.angular.z)

    def cb_desired_thrust(self, msg):
        """
        自動模式才生效。輸入範圍：-100 ~ +100
        將 thrust 映射到 [-1, 1]，舵角維持現值（除非 rudder 也更新）
        """
        if not self.auto:
            return
        thrust_norm = float(np.clip(msg.data, -100.0, 100.0)) / 100.0
        # 保留既有 rudder，僅更新 thrust
        current_rudder = self.left_ang_msg.data
        self.apply_thrust_rudder(thrust_norm, current_rudder)

    def cb_desired_rudder(self, msg):
        """
        自動模式才生效。輸入範圍：-100 ~ +100
        將 rudder 比例映射到 [-max_steer, +max_steer]（弧度），推力維持現值。
        """
        if not self.auto:
            return
        rudder_ratio = float(np.clip(msg.data, -100.0, 100.0)) / 100.0
        rudder = rudder_ratio * self.max_steer_rad  # 和 cmd_vel 一致：左正→加負號

        # 取得目前 thrust（規範化後）來判斷是否倒車
        current_thrust = 0.0
        if self.linear_scaling != 0.0:
            current_thrust = float(np.clip(self.left_msg.data / self.linear_scaling, -1.0, 1.0))

        # ★重點：若在倒車，把舵角反向
        if current_thrust < 0.0:
            rudder = -rudder

        # thrust 維持現值，只更新 rudder
        self.apply_thrust_rudder(current_thrust, rudder)

    def cb_joy(self, data):
        self.joy_remap(data)
        
        # ---- 模式切換 ----
        if (data.buttons[7] == 1) and not self.auto:
            self.auto = 1
            rospy.loginfo("going auto")
        elif (data.buttons[6] == 1) and self.auto:
            self.auto = 0
            rospy.loginfo("going manual")

        # 只有手動模式才用搖桿直接控制與發 cmd_vel
        if not self.auto:
            cmd = Twist()  # ★無論如何先建立，避免 UnboundLocalError

            # 防呆：避免 Axis/Buttons 長度不夠時拋例外
            axes = list(data.axes) if hasattr(data, "axes") else []
            buttons = list(data.buttons) if hasattr(data, "buttons") else []
            ax1 = axes[1] if len(axes) > 1 else 0.0
            ax3 = axes[3] if len(axes) > 3 else 0.0
            btn4 = buttons[4] if len(buttons) > 4 else 0

            if btn4 == 1:
                # 手動按住觸發輸出
                thrust_norm = float(np.clip(ax1, -1.0, 1.0))
                turn_norm   = float(np.clip(ax3, -1.0, 1.0))

                rudder = -turn_norm * self.max_steer_rad
                # 倒車時反向舵角以維持「左打=船頭左」
                if thrust_norm < 0.0:
                    rudder = -rudder

                self.apply_thrust_rudder(thrust_norm, rudder)

                cmd.linear.x  = thrust_norm
                cmd.angular.z = turn_norm
            else:
                # 鬆開就停
                self.apply_thrust_rudder(0.0, 0.0)
                cmd.linear.x  = 0.0
                cmd.angular.z = 0.0

            # ★只在手動模式才 publish cmd_vel
            self.cmd_pub.publish(cmd)
        # 自動模式下不 publish cmd_vel（避免自回饋造成干擾）

    def joy_remap(self, data):
        # 保留給其他模組使用
        self.remap_pub.publish(data)


def main():
    rospy.init_node("twist2drive", anonymous=True)
    node = Node()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()