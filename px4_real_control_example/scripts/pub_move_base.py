#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 发布move_base_simple/goal消息，用于激活fuel

import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
import math

def send_goal(x, y, yaw_deg):
    # 1. 初始化节点
    rospy.init_node('simple_goal_sender', anonymous=True)
    
    # 2. 创建发布者
    # 话题名称: /move_base_simple/goal
    # 消息类型: PoseStamped
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    
    # 3. 等待连接 (重要)
    # 如果不等待，消息可能在建立连接前就发送出去了，导致接收端收不到
    rospy.loginfo("Waiting for publisher connection...")
    while pub.get_num_connections() == 0 and not rospy.is_shutdown():
        rospy.sleep(0.1)
    
    # 4. 构建消息
    goal = PoseStamped()
    
    # --- Header 设置 ---
    goal.header.seq = 0
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "world"  # 重要：坐标系通常是 "map" 或 "odom"
    
    # --- Position (位置) ---
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = 0.0  # 2D 导航 z 通常为 0
    
    # --- Orientation (姿态) ---
    # 将欧拉角 (Roll, Pitch, Yaw) 转换为 四元数 (x, y, z, w)
    # 2D 平面导航只需要改变 Yaw (偏航角)
    yaw_rad = math.radians(yaw_deg) # 角度转弧度
    q = quaternion_from_euler(0, 0, yaw_rad)
    
    goal.pose.orientation.x = q[0]
    goal.pose.orientation.y = q[1]
    goal.pose.orientation.z = q[2]
    goal.pose.orientation.w = q[3]
    
    # 5. 发布消息
    pub.publish(goal)
    rospy.loginfo("Goal sent: x=%.2f, y=%.2f, yaw=%.2f°" % (x, y, yaw_deg))
    
    # 稍微等待一下确保发送完成
    rospy.sleep(1)

if __name__ == '__main__':
    try:
        # --- 这里修改你的目标点 ---
        target_x = 0.2    # 米
        target_y = 0.2    # 米
        target_yaw = 0 # 度 (90度是朝向 Y 轴正方向)
        
        send_goal(target_x, target_y, target_yaw)
        
    except rospy.ROSInterruptException:
        pass


