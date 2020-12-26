#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry

import matplotlib.pyplot as plt
import math
import numpy as np
import sys, select, os
if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

e = """
Communications Failed
"""

saved_x = []
saved_gt = []
saved_P = []
saved_t = []
count = 0

def getKey():
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

class KalmanFilter(object):
    def __init__(self, h, d, x_0, Q, R, P_0, T):
        self.h = h
        self.d = d

        self.Q = Q
        self.R = R
        self.P = P_0
        self.x = float(x_0)
        self.x_predict = self.x
        self.P_predict = self.P

        self.T = T

        self.u = 0 # initialize the cmd_vel input
        self.phi = np.nan #initialize the measurement input
        self.gt = 0
		

    def cmd_callback(self, cmd_msg):
        self.u = cmd_msg.linear.x

    ## scall_callback updates self.phi with the most recent measurement of the tower.
    def scan_callback(self, data):
        self.phi = float(data.data)*math.pi/180

    ## gt_callback updates the true robot pose ***only to be used for plotting)***
    def gt_callback(self,msg):
        self.gt = msg.pose.pose.position.x

    def exact_measure(self):
        if (self.x < self.d):
            return math.atan(self.h/(self.d-self.x))
        else:
            return math.atan(self.h/(self.d-self.x)) + math.pi

    def lin_measure(self):
        return (self.h/((self.h)**2 + (self.d - self.x)**2))

    ## call within run_kf to update the state with the measurement 
    def predict(self, u):
        rospy.loginfo("TODO: update state via the motion model, and update the covariance with the process noise")
        
        self.x = self.x + self.T*u
        self.P = self.P + self.Q*(self.T*self.T)

        return 

    ## call within run_kf to update the state with the measurement 
    def measurement_update(self, phi):

        if math.isnan(phi):
            rospy.loginfo('nan received')
            # ignore nan measurements when tower is occluded
        else:
            # perform measurement update
            rospy.loginfo("TODO: update state when a new measurement has arrived using this function")
			
            # doing the covariance update (excluding state)
            meas_cov = (self.lin_measure()**2)*self.P + self.R
            kal_gain = self.P*self.lin_measure()/meas_cov
            self.P = self.P - (kal_gain**2)*meas_cov
    
            #updating state
            meas_pred = self.exact_measure()
            meas_res = self.phi - meas_pred
            self.x = self.x + kal_gain*meas_res
            print ("location: {}, measure pos: {}, true measurement: {}".format (self.x, self.d-self.h/math.tan(self.phi), self.phi))
            print ("covariance: {}".format (self.P))

        return

    def run_kf(self):

        current_input = self.u
        current_measurement = self.phi
        
        self.predict(current_input)
        self.measurement_update(current_measurement)
        self.state_pub.publish(str(float(self.x)))

        # for plotting
        global count
        saved_t.append(self.T * count)
        saved_x.append(self.x)
        saved_gt.append(self.gt)
        saved_P.append(self.P)
        count += 1



if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
        
    rospy.init_node('Lab4')
    try:
        h = 1.3 #y distance to tower
        d = 2.5 #x distance to tower (from origin)  
        
        x_0 = 0. #initial state position
        
        Q = 0.005  #process noise covariance
        R = 0.001  #measurement noise covariance
        P_0 = 1e-5 #state covariance
        hz = 15 #frequency of KF loop
        T = 1.0/hz
        kf = KalmanFilter(h, d, x_0, Q, R, P_0, T)
        kf.scan_sub = rospy.Subscriber('scan_angle', String, kf.scan_callback, queue_size=1)
        kf.cmd_sub = rospy.Subscriber('cmd_vel_noisy', Twist, kf.cmd_callback)
        kf.gt_sub = rospy.Subscriber('odom',Odometry,kf.gt_callback)
        kf.state_pub = rospy.Publisher('state', String, queue_size = 1) #used to publish the estimated state (can subscribe to this with your pid controller)
        rospy.sleep(1)
        rate = rospy.Rate(hz)
        while not rospy.is_shutdown():
            kf.run_kf()  
            rate.sleep()
            

    except Exception as e:
        # plotting
        fig, ax = plt.subplots()
        ax.plot(saved_t, saved_x, label='est')
        ax.plot(saved_t, saved_gt, label='gt')

        ax.set(xlabel='time (s)', ylabel='x (m)',
            title='Distance')
        ax.legend()
        fig.savefig("x.png")

        fig2, ax2 = plt.subplots()
        ax2.plot(saved_t, saved_P)

        ax2.set(xlabel='time (s)', ylabel='covariance (m^2)',
            title='Covariance')
        fig2.savefig("covariance.png")
        print(e)

    finally:
        rospy.loginfo("goodbye")