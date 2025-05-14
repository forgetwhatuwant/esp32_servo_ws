#!/usr/bin/env python3

import rospy
from std_msgs.msg import UInt16
import time

def test_gripper():
    # Initialize ROS node
    rospy.init_node('test_gripper_node', anonymous=True)
    
    # Create publisher
    pub = rospy.Publisher('/sroi_gripper/command', UInt16, queue_size=10)
    
    # Set rate to 50Hz
    rate = rospy.Rate(50)
    
    print("Starting gripper test...")
    print("Publishing to /sroi_gripper/command at 50Hz")
    
    try:
        while not rospy.is_shutdown():
            print("Moving from 0 to 90 degrees")
            start_time = time.time()
            while time.time() - start_time < 5:  # Run for 5 seconds
                # Calculate current angle based on elapsed time
                elapsed = time.time() - start_time
                angle = int((elapsed / 5.0) * 90)  # Linear interpolation from 0 to 90
                pub.publish(angle)
                rate.sleep()
                
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    try:
        test_gripper()
    except rospy.ROSInterruptException:
        pass
