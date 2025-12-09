#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
from mavros_msgs.srv import SetMode, SetModeRequest

def change_mode(mode_name):
    rospy.init_node('mode_switcher', anonymous=True)
    
    rospy.wait_for_service("/mavros/set_mode")
    try:
        set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
        
        req = SetModeRequest()
        req.custom_mode = mode_name
        
        resp = set_mode_client.call(req)
        
        if resp.mode_sent:
            rospy.loginfo(f"Mode switch to {mode_name} REQUESTED successfully.")
        else:
            rospy.logwarn(f"Mode switch to {mode_name} FAILED.")
            
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("[Usage]: python mode_switcher.py [OFFBOARD | AUTO.LOITER | POSCTL]\
              \n[OFFBOARD]: code control \
              \n[AUTO.LOITER]: GNSS can be used\
              \n[POSCTL]: no GNSS, use VIO/LIO pose")
        sys.exit(1)
        
    mode = sys.argv[1]
    print(f"Attempting to switch to: {mode}")
    
    # 简单的安全提示
    if mode == "OFFBOARD":
        print("[WARNING]: Ensure your control script (Script 3) is running NOW before switching!")
        
    change_mode(mode)