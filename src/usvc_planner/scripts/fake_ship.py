#!/usr/bin/env python

# -*- coding: utf-8 -*-
"""
fake_ship: 仿真虚拟船节点
  - 订阅 /usvc_planner/cmd (GuidanceCmd: obj_speed + obj_heading)
  - 一阶滞后跟踪期望航向/速度（模拟船响应延迟, 便于调 PD/PI）
  - 运动学积分更新经纬度（球面近似）
  - 发布 /usvc_navi/navi (usvc_navi/navi) 替代静态 fake_navi
  - 同时广播 /tf map->ship_base 便于 RViz 可视化
闭环: planner_node -> fake_ship -> navi -> planner_node
"""
import math
import rospy
from usvc_navi.msg import navi
from usvc_planner.msg import GuidanceCmd
from geometry_msgs.msg import TransformStamped
import tf2_ros

# 地球半径（米, 球面近似）
R_EARTH = 6378137.0

def wrap_angle_deg(a):
    while a >= 360.0:
        a -= 360.0
    while a < 0.0:
        a += 360.0
    return a

def wrap_angle_diff(d):
    while d > 180.0:
        d -= 360.0
    while d <= -180.0:
        d += 360.0
    return d

class FakeShip:
    def __init__(self):
        self.tau_heading = rospy.get_param("~tau_heading", 1.0)
        self.tau_speed   = rospy.get_param("~tau_speed",   1.5)
        self.rate_hz     = rospy.get_param("~rate",       50)
        self.pub_navi_hz = rospy.get_param("~pub_navi_hz", 10)

        self.lat     = rospy.get_param("~lat",       32.8194)
        self.lon     = rospy.get_param("~lon",       119.1138)
        self.alt     = rospy.get_param("~alt",       0.0)
        self.heading = rospy.get_param("~heading",   50.0)
        self.speed   = 0.0
        self.roll    = 0.0
        self.pitch   = 0.0

        self.obj_speed   = 0.0
        self.obj_heading = self.heading
        self.reached     = False
        self.last_cmd_t  = rospy.Time.now()

        self.navi_pub = rospy.Publisher("/usvc_navi/navi", navi, queue_size=1, tcp_nodelay=True)
        self.cmd_sub  = rospy.Subscriber("/usvc_planner/cmd", GuidanceCmd, self.on_cmd, tcp_nodelay=True)
        self.br       = tf2_ros.TransformBroadcaster()

        self.last_t = rospy.Time.now()
        self.navi_acc = 0
        self.pub_navi_period = 1.0 / self.pub_navi_hz
        rospy.loginfo("[FakeShip] 启动. 起始位姿 lat=%.6f lon=%.6f heading=%.1f",
                      self.lat, self.lon, self.heading)

    def on_cmd(self, msg):
        self.obj_speed   = float(msg.obj_speed)
        self.obj_heading = float(msg.obj_heading)
        self.reached     = bool(msg.reached)
        self.last_cmd_t  = rospy.Time.now()

    def step(self, dt):
        # dt 限幅, 避免首帧或调诗时出现大 dt 导致积分爆炸
        if dt <= 0 or dt > 0.5:
            return
        # 一阶滞后: 指数衰减公式, 比 euler 积分更稳定
        # heading: d(heading)/dt = (obj - heading) / tau
        alpha_h = 1.0 - math.exp(-dt / self.tau_heading)
        d_h = wrap_angle_diff(self.obj_heading - self.heading)
        self.heading = wrap_angle_deg(self.heading + d_h * alpha_h)
        # speed: 同样用指数衰减
        alpha_s = 1.0 - math.exp(-dt / self.tau_speed)
        self.speed += (self.obj_speed - self.speed) * alpha_s
        if self.reached:
            self.speed = 0.0
        # 调试: 打印收到的 cmd 与当前 heading/speed (0.5s 一次)
        if not hasattr(self, '_dbg_t'):
            self._dbg_t = rospy.Time.now()
        if (rospy.Time.now() - self._dbg_t).to_sec() > 0.5:
            rospy.loginfo("[FakeShip] cmd(h=%.2f s=%.2f) ship(h=%.2f s=%.2f)",
                          self.obj_heading, self.obj_speed,
                          self.heading, self.speed)
            self._dbg_t = rospy.Time.now()

        h_rad = math.radians(self.heading)
        vn = self.speed * math.cos(h_rad)
        ve = self.speed * math.sin(h_rad)

        dlat = vn / R_EARTH * (180.0 / math.pi)
        cos_lat = max(math.cos(math.radians(self.lat)), 1e-6)
        dlon = ve / (R_EARTH * cos_lat) * (180.0 / math.pi)

        self.lat += dlat * dt
        self.lon += dlon * dt

    def make_navi(self):
        msg = navi()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.ins_sta       = 1
        msg.timeTick      = int(rospy.Time.now().to_sec() * 1000) & 0xFFFFFFFF
        msg.gyro_z        = 0.0
        msg.roll         = self.roll
        msg.pitch        = self.pitch
        msg.heading      = self.heading
        msg.vel_x        = self.speed
        msg.vel_n        = self.speed * math.cos(math.radians(self.heading))
        msg.vel_e        = self.speed * math.sin(math.radians(self.heading))
        msg.vel_d        = 0.0
        msg.latitude     = self.lat
        msg.longitude    = self.lon
        msg.altitude     = self.alt
        msg.satellite_count = 12
        msg.pos_sta      = 1
        msg.heading_sta  = 1
        msg.position_std_lat = 0.05
        msg.position_std_lon = 0.05
        return msg

    def make_tf(self):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id  = "ship_base"
        cos_lat = max(math.cos(math.radians(self.lat)), 1e-6)
        x = (self.lon - 119.1138) * R_EARTH * cos_lat * math.pi / 180.0
        y = (self.lat - 32.8194) * R_EARTH * math.pi / 180.0
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        h_enu = math.pi/2 - math.radians(self.heading)
        t.transform.rotation.z = math.sin(h_enu / 2.0)
        t.transform.rotation.w = math.cos(h_enu / 2.0)
        return t

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = (now - self.last_t).to_sec()
            self.last_t = now

            self.step(dt)

            self.navi_acc += dt
            if self.navi_acc >= self.pub_navi_period:
                self.navi_acc = 0
                self.navi_pub.publish(self.make_navi())
                self.br.sendTransform(self.make_tf())

            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("fake_ship", anonymous=False)
    fs = FakeShip()
    try:
        fs.spin()
    except rospy.ROSInterruptException:
        pass
