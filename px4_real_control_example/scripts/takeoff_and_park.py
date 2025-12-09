#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

# 全局变量
current_state = State()
current_pose = PoseStamped() # 新增：用于存储当前位置

# 状态回调
def state_cb(msg):
    global current_state
    current_state = msg

# 新增：位置回调函数
def pos_cb(msg):
    global current_pose
    current_pose = msg

def main():
    rospy.init_node('takeoff_script', anonymous=True)

    # 1. 订阅状态
    state_sub = rospy.Subscriber("mavros/state", State, state_cb)
    
    # 2. 新增：订阅本地位置信息 (为了获取当前高度)
    local_pos_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, pos_cb)
    
    # 3. 发布位置指令
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    
    # 服务客户端
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    
    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    rate = rospy.Rate(20) # 20Hz

    # 等待连接
    while not rospy.is_shutdown() and not current_state.connected:
        rospy.loginfo("Waiting for FCU connection...")
        rate.sleep()

    # 设定目标高度
    target_alt = 1.5
    
    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = target_alt

    # 发送一些设定点以允许切换到 OFFBOARD
    for i in range(100):   
        if rospy.is_shutdown():
            break
        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()
    
    rospy.loginfo("Starting takeoff sequence...")

    while not rospy.is_shutdown():
        # 1. 尝试切换 OFFBOARD 和 解锁 (保持原逻辑，每5秒重试一次)
        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
            if set_mode_client.call(offb_set_mode).mode_sent:
                rospy.loginfo("Offboard enabled")
            last_req = rospy.Time.now()
        else:
            if not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                if arming_client.call(arm_cmd).success:
                    rospy.loginfo("Vehicle armed")
                last_req = rospy.Time.now()
        
        # 持续发布位置指令
        local_pos_pub.publish(pose)

        # 2. 修改后的逻辑：基于高度判断
        # 只有当模式是 OFFBOARD 且已解锁时，才检查高度
        if current_state.mode == "OFFBOARD" and current_state.armed:
            # 获取当前高度
            current_z = current_pose.pose.position.z
            
            # 计算误差 (绝对值)
            err_z = abs(current_z - target_alt)
            
            # 打印当前高度以便调试 (可选，每隔一点时间打印一次防止刷屏)
            # rospy.loginfo_throttle(1, "Current Altitude: %.2f, Target: %.2f" % (current_z, target_alt))

            # 判定到达：误差小于 0.15 米 (可以根据需要调整阈值)
            if err_z < 0.08:
                rospy.loginfo("Target altitude reached (Current: %.2f). Switching to AUTO.LOITER.", current_z)
                
                loiter_mode = SetModeRequest()
                loiter_mode.custom_mode = 'AUTO.LOITER'
                
                if set_mode_client.call(loiter_mode).mode_sent:
                    rospy.loginfo("Switched to AUTO.LOITER. Takeoff complete.")
                    break # 任务完成，退出循环

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass