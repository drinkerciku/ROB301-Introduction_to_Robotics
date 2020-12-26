#!/usr/bin/env python
 
import numpy as np
import time
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
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.color_sub = rospy.Subscriber('camera_rgb', String, self.colour_callback, queue_size=10)
        self.line_sub = rospy.Subscriber('line_idx', String, self.camera_callback, queue_size=10)
        self.fwd_sub = rospy.Subscriber('fwd_Movment', String, self.fwd_callback, queue_size=10)
        self.CurColour = None
        self.xVel = 0.1
        self.line_pos = 0
        self.goal = 320
 
    def colour_callback(self, msg):
        '''
        callback function that receives the most recent colour measurement from the camera.
        '''
        rgb = msg.data.replace('r:','').replace('b:','').replace('g:','').replace(' ','')
        r,g,b = rgb.split(',')
        r,g,b=(float(r), float(g),float(b))
        self.CurColour = np.array([r,g,b])
 
    def fwd_callback(self, data):
        self.xVel = float(data.data)
        print (self.xVel)
        
 
    def camera_callback(self, data):
        self.line_pos = int(data.data)
        pass
 
    def ninety_turn(self):
        twist = Twist() 
        twist.angular.z = 0.3
        self.cmd_pub.publish(twist)
        time.sleep(6)
        twist.angular.z = 0
        self.cmd_pub.publish(twist)
        time.sleep(1)
        twist.angular.z = -0.3
        self.cmd_pub.publish(twist)
        time.sleep(6)
 
    def follow_the_line(self):
        # initialize control variables
        twist = Twist()        
        error = 0
        prev_err = 0
        rate = rospy.Rate(25)
        twist.linear.x = self.xVel
            
        # control gains 
        k_p = 0.005
        k_i = 0.00015
        k_d = 0.02
        k_acc = 70 #acceptable value for the error to be under, to prevent windup
 
        # correction variables
        correction_p = 0
        correction_i = 0
        integral = 0
        derivative = 0
 
        integral_limit = 1200
 
        while not rospy.is_shutdown():
            twist.linear.x = self.xVel
            error = self.line_pos - self.goal
            derivative = error - prev_err
            prev_err = error
 
            if self.xVel == 0:
                self.ninety_turn()
 
            if (np.amin(self.CurColour)<210 or self.line_pos == 0):
                twist.angular.z = 0
            else:
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
 
                correction_d = k_d*derivative
 
                twist.angular.z = -correction_p - correction_i - correction_d
        
            print (twist.linear.x)
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