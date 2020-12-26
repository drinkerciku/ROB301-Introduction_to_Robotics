#!/usr/bin/env python
import rospy
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry

class robot_line():

    def __init__(self):
        # initialize arguments incorporated in the robot_line object
        self.line_pos = 0
        self.goal = 320
        # intialize publisher node
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        # initialize subscriber node with a callback function to update the self.line_pos
        self.line_sub = rospy.Subscriber('line_idx', String, self.update_callback, queue_size = 1)

    def update_callback(self, new_pos):
        self.line_pos = int(new_pos.data)
        return new_pos.data

    def bang_bang_control(self):

        twist = Twist()
        # frequency of 25Hz
        rate = rospy.Rate(25)
        # start driving straight
        twist.linear.x = 0.1

        # error variable initialized at 0
        error = 0

        # flag for the beginning of tracing a line
        flag = 0

        while not rospy.is_shutdown():

            # start implementing the Bang Bang Controller
            error = self.line_pos - self.goal
            print(error)

            if self.line_pos != 0 or flag != 45:

                if error < 0:
                    twist.angular.z = 0.25
                    twist.linear.x = 0.1
                    if error < -90:
                        twist.linear.x = 0
                elif error > 0 :
                    twist.angular.z = -0.25
                    twist.linear.x = 0.1
                    if error > 90:
                        twist.linear.x = 0
                else:
                    twist.angular.z = 0
                    twist.linear.x = 0.1

                if self.line_pos == 0:
                    flag += 1
                else:
                    flag = 0

            else:

                twist.angular.z = 0
                twist.linear.x = 0

            self.cmd_pub.publish(twist)
            rate.sleep()

        pass

    def P_control(self):

        # initialize control variables
        twist = Twist()
        error = 0
        twist.linear.x = 0.15
		
		rate = rospy.Rate(25)

        # proportional gain
        k_p = 0.2

        while not rospy.is_shutdown():

            # implement Proportional Controller
            error = self.line_pos - self.goal
            self.cmd_pub.publish(twist)
			rate.sleep()
		
		pass
        
    def PI_control(self):

        # initialize control variables
        twist = Twist()
        error = 0
        rate = rospy.Rate(25)
        twist.linear.x = 0.1

        # control gains
        k_p = 0.005
        k_i = 0.00015

        # correction variables
        correction_p = 0
        correction_i = 0
        integral = 0

        integral_limit = 1200

        while not rospy.is_shutdown():

            # implement Proportional-Integral Controller
            error = self.line_pos - self.goal
            correction_p = k_p * error
            integral += error*0.75

            if integral > integral_limit:
                integral = integral_limit
            elif integral < -integral_limit:
                integral = - integral_limit

            correction_i = k_i*integral
            twist.angular.z = -correction_p - correction_i

            self.cmd_pub.publish(twist)
            rate.sleep()

        pass

def main():
    try:
        rospy.init_node('motor')
        robot = robot_line()
        robot.bang_bang_control()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()