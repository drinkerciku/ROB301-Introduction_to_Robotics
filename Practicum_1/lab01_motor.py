#!/usr/bin/env python
import rospy
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String


def publisher_node():
    '''
    TODO: complete the publisher function here
    '''
    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    rate = rospy.Rate(10)	# 10Hz
    twist = Twist()
    
    # cnt the ticks of the clock at the 10 Hz frequency (every 0.1 secomnds)
    cnt_ticks = 0    
    
    # start driving straight
    twist.linear.x = 0.1

    while not rospy.is_shutdown():

        cmd_pub.publish(twist)
        rate.sleep()
        
        cnt_ticks += 1

	# it will take 10 seconds for translation and start rotating clockwise
	# after reaching a displacement of 1m
        if cnt_ticks == 101:
	    twist.linear.x = 0
	    twist.angular.z = -0.1
	
	# it will take an additional 63 seconds to complete a rotation
	if cnt_ticks == 729:
	    twist.angular.z = 0

    pass


def main():

    try:
        rospy.init_node('motor')
        publisher_node()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()