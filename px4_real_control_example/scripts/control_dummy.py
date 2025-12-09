#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped

def main():
    rospy.init_node('main_controller', anonymous=True)
    
    # 发布话题
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    
    rate = rospy.Rate(20) # 20Hz
    
    # 假设这个脚本想让飞机飞到 (2, 2, 2)
    pose = PoseStamped()
    pose.pose.position.x = 2
    pose.pose.position.y = 2
    pose.pose.position.z = 2

    rospy.loginfo(f"Controller Script Started. Publishing setpoints to ({pose.pose.position.x}, {pose.pose.position.y}, {pose.pose.position.z})...")
    rospy.loginfo(" switch to [OFFBOARD] mode using the switcher script.")

    while not rospy.is_shutdown():
        # 持续发布指令
        # 即使现在是 AUTO.LOITER 模式，发这个指令也是安全的，飞控会忽略它
        # 一旦切入 OFFBOARD，飞控会立刻响应这个指令
        pose.header.stamp = rospy.Time.now()
        local_pos_pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    main()