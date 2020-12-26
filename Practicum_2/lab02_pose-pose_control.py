#!/usr/bin/env python
import rospy
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String

def deliverable_1(ticks,twist_obj):

    if ticks == 1:
	    twist_obj.angular.z = 0.1
    
    if ticks == 29: 
        twist_obj.angular.z = 0
        twist_obj.linear.x = 0.1

    if ticks == 235:
        twist_obj.linear.x = 0
        twist_obj.angular.z = 0.2

    if ticks == 352:
	    twist_obj.angular.z = 0

    return

def deliverable_2(ticks,twist_obj):

    #format for each entry: [tick time, [angular velocity, linear velocity]]
    angOff = 2
    linOff = 2
    
    robotMarks = [[0, [0, 0.1]], [100, [0.2, 0]], [179, [0, 0.1]], [279, [0.2, 0]], [358, [0, 0.1]], [458, [0.2, 0]], [537, [0, 0]]]

    notEnd = False
    for i in range (0, len(robotMarks)-1):

        #the following 2 if statements are to make changing the offsets easier
        if i % 2 == 0:
            curOffset = [i/2,i/2]
            nextOffset = [i/2+1, i/2] #formatted linear, angular
        if i % 2 == 1:
            currOffset = [int(i/2)+1, int(i/2)]
            nextOffset = [int(i/2)+1, int(i/2)+1]

        #sees if our current time is in a certain range, set angular and liner velocity to the relevent values
        if ticks >= robotMarks[i][0] + (linOff*curOffset[0] + angOff*curOffset[0]) and ticks < robotMarks[i+1][0]+(linOff*nextOffset[0] + angOff*nextOffset[1]):
            print ("ticks: {}, set: {}".format(ticks, i))
            twist_obj.angular.z = robotMarks[i][1][0]
            twist_obj.linear.x = robotMarks[i][1][1]
            notEnd = True

    #stop the robot at the end
    if not(notEnd):
        twist_obj.angular.z = robotMarks[len(robotMarks)-1][1][0]
        twist_obj.linear.x = robotMarks[len(robotMarks)-1][1][1]

    return

def deliverable_3(ticks, twist_obj):
    if ticks < 191:
        twist_obj.linear.x = 0.1
    elif ticks < 250:
        twist_obj.angular.z = 0.4
    elif ticks < 258:
        twist_obj.angular.z = 0
    else:
        twist_obj.linear.x = 0
    return

def publisher_node():
    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
    rate = rospy.Rate(10)
    twist = Twist()

    deliverableDict = {
        1:deliverable_1,
        2:deliverable_2,
        3:deliverable_3
    }

    deliverableNum = 2
    
    # count the ticks of the clock every 0.1 seconds
    cnt_ticks = 0

    while not rospy.is_shutdown():
	
	cmd_pub.publish(twist)
	rate.sleep()

        # give commands to the robot to perform the desired task described in Lab 2 handout
	cnt_ticks += 1
        deliverableDict[deliverableNum](cnt_ticks, twist)

    pass


def main():
    try:
        rospy.init_node('lab02')
        publisher_node()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()