#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from std_msgs.msg import Bool # 引入 Bool 类型消息

# 全局变量
current_state = State()
current_pose = PoseStamped() 
last_vision_time = rospy.Time(0)
vision_data_received = False

# 状态回调
def state_cb(msg):
    global current_state
    current_state = msg

# 本地位置回调
def pos_cb(msg):
    global current_pose
    current_pose = msg

# 视觉定位回调
def vision_cb(msg):
    global last_vision_time, vision_data_received
    last_vision_time = rospy.Time.now()
    vision_data_received = True


def main():
    rospy.init_node('takeoff_script', anonymous=True)

    # 1. 订阅
    state_sub = rospy.Subscriber("mavros/state", State, state_cb)
    local_pos_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, pos_cb)
    vision_sub = rospy.Subscriber("/mavros/vision_pose/pose", PoseStamped, vision_cb)
    
    # 2. 发布
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    
    # 3. 服务
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    
    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    rate = rospy.Rate(20) 

    # 等待连接
    while not rospy.is_shutdown() and not current_state.connected:
        rospy.loginfo("Waiting for FCU connection...")
        rate.sleep()

    target_alt = 1.1
    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = target_alt

    # 预发送
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
    
    # 悬停点锁定
    hover_pose = PoseStamped()
    hover_started = False

    while not rospy.is_shutdown():
        
            
        # 1. 模式维护 (OFFBOARD + ARM)
        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
            if set_mode_client.call(offb_set_mode).mode_sent:
                rospy.loginfo("Offboard enabled")
            last_req = rospy.Time.now()
        else:
            if not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                if arming_client.call(arm_cmd).success:
                    rospy.loginfo("Vehicle armed")
                last_req = rospy.Time.now()

        # 2. 起飞与悬停逻辑
        if not hover_started:
            local_pos_pub.publish(pose) # 发送起飞目标
            
            if current_state.mode == "OFFBOARD" and current_state.armed:
                current_z = current_pose.pose.position.z
                if abs(current_z - target_alt) < 0.1:
                    # 检查视觉数据
                    if vision_data_received and (rospy.Time.now() - last_vision_time) < rospy.Duration(0.5):
                        rospy.loginfo("Target reached. Entering Hover Mode.")
                        # 锁定位置
                        hover_pose = current_pose
                        hover_pose.pose.position.z = target_alt 
                        hover_started = True
        else:
            # 已经到达高度，持续发送悬停点
            local_pos_pub.publish(hover_pose)
            rospy.loginfo_throttle(2, "Script A: Hovering... Waiting for handover.")
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
