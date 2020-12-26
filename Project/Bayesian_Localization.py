#!/usr/bin/env python
import rospy
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
import re
import sys, select, os
 
if os.name == 'nt':
    import msvcrt
else:
    import tty, termios
 
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
 
 
class BayesLoc:
 
    def __init__(self, P0, colourCodes, colourMap, transProbBack, transProbForward):
        self.colour_sub = rospy.Subscriber('camera_rgb', String, self.colour_callback)
        self.line_sub = rospy.Subscriber('line_idx', String, self.line_callback)
        self.cmd_pub= rospy.Publisher('fwd_Movment', String, queue_size=1)
 
        self.colourDict =  {0:"green", 1:"orange", 2:"purple", 3:"yellow", 4:"white"}
        self.probability = P0 ## initial state probability is equal for all states
        self.colourCodes = colourCodes
        self.colourMap = colourMap
        self.transProbBack = transProbBack
        self.transProbForward = transProbForward
        self.numStates = len(P0)
        self.statePrediction = np.zeros(np.shape(P0))
 
        self.curColour = None ##most recent measured colour in rgb
        self.curColourIndex = 0
 
 
    def colour_callback(self, msg):
        '''
        callback function that receives the most recent colour measurement from the camera.
        '''
        rgb = msg.data.replace('r:','').replace('b:','').replace('g:','').replace(' ','')
        r,g,b = rgb.split(',')
        r,g,b=(float(r), float(g),float(b))
        self.curColour = np.array([r,g,b])
 
    
    def waitforcolour(self):
        while(1):
            if self.curColour is not None:
                break
 
    def measurement_model(self):
        if self.curColour is None:
            self.waitforcolour()
        prob = np.zeros(len(self.colourCodes))
        sumDist = 0
        for i, colour in enumerate(self.colourCodes):
            isSame = True
            euclidDist = 0
            for j, val in enumerate (colour):
                euclidDist += (abs(colour[j] - self.curColour[j]))**2
 
            euclidDist = math.sqrt(euclidDist)
            if (euclidDist != 0):
                prob[i] = 1/euclidDist
            elif (euclidDist == 0):
                prob[i] = 1      
 
            sumDist += prob[i]
 
        #maxDist = max(prob)
        #for i in range (0, len(prob)):
        #    prob[i] = maxDist - prob[i]
        #    sumDist += prob[i]
 
        for i in range (0, len(prob)):
            prob[i] = prob[i]/sumDist
 
        self.curColourIndex = np.argmax(prob)
        return prob
 
    def statePredict(self,control):
        q = np.copy(self.probability)
        for i in range(self.numStates): #i is currently considered next pt / possible position
            totalProb = 0
            for j in range (self.numStates): #j is the considered current pt
                dist = (i-j)
                if (dist == self.numStates-1):
                    dist = -1
                if (dist == -self.numStates+1):
                    dist = 1
 
                if (dist == 1):
                    totalProb += self.transProbForward[control]*self.probability[j]
                if (dist == -1):
                    totalProb += self.transProbBack[1-control]*self.probability[j]
            q[i] = totalProb
        self.probability = q
 
    def stateUpdate(self):
        #rospy.loginfo('updating state')
        q = np.copy(self.probability)
        print ("measurements: {}".format(self.measurement_model()))
        for i in range(self.numStates): # goes thorugh possible current states
            probSensing = self.measurement_model()
            q[i]=probSensing[self.colourMap[i]]*self.probability[i] #the probability of being at the color of i defined by colormap[i] * prob of being at i
        s = sum(q)
        for i in range(len(q)):
            q[i] = q[i] / s
        self.probability = q  
    
def printProbsFormatted(probs):
    if (len(probs) == 11):
        print ("--- {:.3f} {:.3f} {:.3f} ---".format(probs[7],probs[6],probs[5]))
        print ("{:.3f}               {:.3f}".format(probs[8],probs[4]))
        print ("{:.3f}               {:.3f}".format(probs[9],probs[3]))
        print ("{:.3f}               {:.3f}".format(probs[10],probs[2]))
        print ("--- {:.3f}  ----  {:.3f} ---".format(probs[0],probs[1]))
        print("")
 
def onWhite (meas):
    white = [220,220,220]
    for i, val in enumerate(meas):
        if abs(val - white[i]) > 15:
            return False
    return True
 
if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
 
    # 0: Green, 1: Purple, 2: Orange, 3: Yellow, 4: Line   
    colour_maps = [3, 2, 0, 3, 2, 1, 0, 1, 2, 0, 3] ## current map starting at cell#2 and ending at cell#12
    colourDict = {0:"green", 1:"purple", 2:"orange", 3:"yellow"}
    destinations = [0, 1, 9]
    
    colour_codes = [[72, 255, 72], #green FAKE NEWS
                   [145,145,255], #purple
                   #[255, 144, 0], #orange
                   [255, 200, 0], #orange (had to redefine, else the bot cant sense btw orange and yellow)
                   [255, 255, 0]] #yellow 
 
    #colour_codes = [[100, 255, 100], #green These are aligned with the example vals, may not be best if they change again. 
                    #[200, 200,255], #purple
                    #[255, 200, 0], #orange
                    #[255, 255, 0]] #yellow 
 
    trans_prob_fwd = [0.1,0.9]
    trans_prob_back = [0.2,0.8]
                 
    rospy.init_node('final_project')
    bayesian=BayesLoc([1.0/len(colour_maps)]*len(colour_maps), colour_codes, colour_maps, trans_prob_back,trans_prob_fwd)
    prob = []
    rospy.sleep(0.5)    
    state_count = 0
    
    prev_state=None
    try:
        measurementCounter = 0
        prevWhite = True
        curWhite = True
        curWhiteConfirmed = True
        recentChange = False
        newLocation = True
        goFwd = True
        stopCounter = 0
 
        while (1):
            key = getKey()
            if (key == '\x03'): 
                rospy.loginfo('Finished!')
                rospy.loginfo(prob)
                break
            
            curWhite = onWhite(bayesian.curColour)
 
            ##the following flips "new location" whenever the bot moves from a colored tile to white, or vice versa. The extra checks are to ensure new location is flipped only when the bot sees the same new color 5 times in a row
            if (curWhite!=prevWhite and (recentChange == False)):
                recentChange = True
                prevWhite = curWhite
                #print (prevWhite)
            elif (recentChange and curWhite != prevWhite and measurementCounter < 5):
                recentChnage = False
                measurementCounter = 0 
            elif (recentChange and curWhite == prevWhite and measurementCounter < 5):
                measurementCounter += 1
            elif (recentChange and curWhite == prevWhite and measurementCounter >= 5):
                newLocation = True
                curWhiteConfirmed = curWhite
                recentChange = False
                measurementCounter = 0
 
            if (newLocation and curWhiteConfirmed == False): #if at a new location, and the current measurement is not white. Aka when steps on new tile
                newLocation = False
                bayesian.statePredict(1) #control input: 1 = fwd, 0 = back
                printProbsFormatted(bayesian.probability)
                bayesian.stateUpdate()
                printProbsFormatted(bayesian.probability)
                print ("")
                print ("")
 
                curState = np.argmax(bayesian.probability)
                certainties = np.copy(bayesian.probability)
                certainties = np.sort(certainties)
 
                print (curState)
                print (certainties)
                if (curState in destinations and certainties[len(certainties)-1] > 0.5):
                    bayesian.cmd_pub.publish("0.1")
                    time.sleep(3)
                    bayesian.cmd_pub.publish("0")
                    time.sleep(9)
                    bayesian.cmd_pub.publish("0.1")
             
            
 
    #except Exception as e:
    #    print("comm failed:{}".format(e))
 
    finally:
        rospy.loginfo(bayesian.probability)
        cmd_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        twist = Twist()
        cmd_publisher.publish(twist)
