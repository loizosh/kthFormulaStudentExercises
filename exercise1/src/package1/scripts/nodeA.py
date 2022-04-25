#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16

def talker():
    rospy.init_node('nodeA', anonymous=True)
    pub = rospy.Publisher('hadjiloizou', Int16, queue_size=1)
    
    rate = rospy.Rate(20) # 20hz
    k = 1 # Initialize variable to be published
    n = 4 # Step size

    while not rospy.is_shutdown():
        k = k + n
        pub.publish(k)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass    
