#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys, select, os
if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

e = """
Communications Failed
"""

def getKey(): #you can ignore this function. It's for stopping the robot when press 'Ctrl+C'
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key



class PIDcontrol():
    def __init__(self):
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.color_sub = rospy.Subscriber('line_idx', String, self.camera_callback, queue_size=1)

        self.line_pos = 0
        self.goal = 320


    def camera_callback(self, data):
        self.line_pos = int(data.data)
        pass


    def follow_the_line(self):
        # initialize control variables
        twist = Twist()
        error = 0
        prev_err = 0
        rate = rospy.Rate(25)
	twist.linear.x = 0.15

        # control gains 
        k_p = 0.003
        k_i = 0.00015
        k_d = 0.01
        k_acc = 70 #acceptable value for the error to be under, to prevent windup

        # correction variables
        correction_p = 0
        correction_i = 0
        integral = 0
        derivative = 0

        integral_limit = 1200

        while not rospy.is_shutdown():
            # implement Proportional-Integral Controller

            print (integral)

            error = self.line_pos - self.goal
            correction_p = k_p * error

            if (error < 0):
                integral += error + k_acc
            elif (error > 0):
                integral += error - k_acc

            if integral > integral_limit:
                integral = integral_limit
            elif integral < -integral_limit:
                integral = -integral_limit

            correction_i = k_i*integral

            derivative = error - prev_err
            prev_err = error
            correction_d = k_d*derivative

            twist.angular.z = -correction_p - correction_i - correction_d
	    self.cmd_pub.publish(twist)
            rate.sleep()
    pass


if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('Lab3')
    PID = PIDcontrol()
    try:
        while(1):
            key = getKey()
            PID.follow_the_line()
            if (key == '\x03'): #stop the robot when exit the program
                break
    except rospy.ROSInterruptException:
        print("comm failed")