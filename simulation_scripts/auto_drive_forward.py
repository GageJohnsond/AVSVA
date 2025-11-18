#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def auto_drive():
    rospy.init_node('husky_auto_drive', anonymous=True)
    
    # Husky listens on this topic in simulation
    pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
    
    # Get speed from parameter (default 0.5 m/s)
    linear_speed = rospy.get_param('~linear_speed', 0.5)
    rate = rospy.Rate(20)  # 20 Hz

    move_cmd = Twist()
    move_cmd.linear.x = linear_speed   # Forward speed
    move_cmd.angular.z = 0.0          # No turning

    rospy.loginfo(f"Husky auto-driving forward at {linear_speed} m/s")
    rospy.loginfo("Press Ctrl+C to stop")

    while not rospy.is_shutdown():
        pub.publish(move_cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        auto_drive()
    except rospy.ROSInterruptException:
        pass