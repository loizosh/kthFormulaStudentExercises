#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16

pub = rospy.Publisher('/kthfs/result', Int16, queue_size=1)

def callback(msg):
    q = 0.15 # Divisor
    pub.publish( int(msg.data/q) )

def main():
    rospy.init_node('nodeB', anonymous=True)
    # rate = rospy.Rate(20) # 20hz
    rospy.Subscriber('hadjiloizou', Int16, callback)
    rospy.spin()    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass