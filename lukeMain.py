#!/usr/bin/env python

#Imports

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time

class RobotMethods():

    def __init__(self):
        # Create Node
        rospy.init_node('robot_methods', anonymous=True)
        #Velocity Publisher
        self.velPub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        #Laser Subscriber
        self.laserSub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        #Twist Variable
        self.velocity = Twist()
        #LaserScan Variable
        self.laser_msg = LaserScan()
        #Rate
        self.rate = rospy.Rate(10)
        #Ctrl C
        self.ctrl_c = False

    def publishVel(self): #Check there are connections to the created node before publishing, otherwise wait for connections
        while not self.ctrl_c:
            connections = self.velPub.get_num_connections()
            if connections > 0:
                self.velPub.publish(self.velocity)
                break
            else:
                self.rate.sleep()

    def shutdownhook(self):
       # works better than the rospy.is_shutdown()
       self.ctrl_c = True

    def laser_callback(self, msg):#Laser callback returns the lasers scan data
        self.laser_msg = msg
 
 
    def get_laserFront(self):
        # This is because some lasers such as 360 sometimes return NaNs, aswell as sometimes the laser not seeing 
        #the robot is stuck on a slight bit of a wall, this scan means the robot sees everything in front of it
        
        time.sleep(2)
        o = 300#Scan between 300th degree and 360th degree for wide view of front angle
        la = []
        while o <= 360:

            la.append(self.laser_msg.ranges[o])
            o = o + 1
        la=np.asarray(la)
        laNaN = la[~np.isnan(la)] # Remove nans from array
        laM = np.sum(laNaN)
        laM = laM / len(laNaN) #Find mean to get average distance from walls at front laser
        return laM

    def get_laser_full(self):
       time.sleep(2)
       return self.laser_msg.ranges # This is for the robot to decide which way to turn

    def stop_robot(self):
       #rospy.loginfo("shutdown time! Stop the robot")
       self.velocity.linear.x = 0.0
       self.velocity.angular.z = 0.0
       self.publishVel()

    def move_straight_time(self, speed, time):
 
       # Initilize velocities
       self.velocity.linear.y = 0
       self.velocity.linear.z = 0
       self.velocity.angular.x = 0
       self.velocity.angular.y = 0
       self.velocity.angular.z = 0
 
       self.velocity.linear.x = speed
 
       i = 0
       # loop to publish the velocity estimate, current_distance = velocity * (t1 - t0)
       while (i <= time):
 
           # Publish the velocity
           self.velPub.publish(self.velocity)
           i += 1
           self.rate.sleep()
 
       # set velocity to zero to stop the robot
       self.stop_robot()
 
       s = "Moved robot " + " for " + str(time) + " seconds" #Push to console to show how long the robot moved for
       return s

    def checkMoveBack(self,array, speed): #Each distance is added to an array, if the distance stays roughly the same for
        #5 iterations, the robot moves back, as this usually means robot is stuck on a wall the lasers don't pick up
        while (len(array) >= 5):
                if(array[0] - array[-1] > 0.2):
                    
        
                    print("moving back")
                    self.velocity.linear.y = 0
                    self.velocity.linear.z = 0
                    self.velocity.angular.x = 0
                    self.velocity.angular.y = 0
                    self.velocity.angular.z = 0
                    self.velocity.linear.x = -speed
                    self.velPub.publish(self.velocity)

                    array = []
                else:
                    array=[]


    def turn(self, clockwise, speed, time):
 
       # Initilize velocities
       self.velocity.linear.x = 0
       self.velocity.linear.y = 0
       self.velocity.linear.z = 0
       self.velocity.angular.x = 0
       self.velocity.angular.y = 0
 
       if clockwise == "clockwise":
           self.velocity.angular.z = -speed
       else:
           self.velocity.angular.z = speed
 
       i = 0
       # loop to publish the velocity estimate, current_distance = velocity * (t1 - t0)
       while (i <= time):
 
           # Publish the velocity
           self.velPub.publish(self.velocity)
           i += 1
           self.rate.sleep()
 
       # set velocity to zero to stop the robot
       self.stop_robot()
 
       s = "Turned robot " + str(clockwise) + " for " + str(time) + " seconds"
       return s

import numpy as np

class robotLogic:
    def __init__(self,speed,time): # Initialise variables
        self.rc = RobotMethods()#Create object from methods class
        self.motion = None
        self.speed = speed
        self.time = time
        self.d = None

    def dist(self):
        return (self.rc.get_laserFront()) #Method to call the laser and get the distance to the wall

    def move(self):# Moves the robot
        distance = self.dist()
        distanceArray = []

        while True:
            while (distance>0.75):# Whilst robot is more then 0.75 away from a wall, move straight
                straightTime = self.rc.move_straight_time(self.speed, self.time)
                print(straightTime)
                distance=self.dist()# Calculate distance again to see if loop can be exited
                print("Current distance from wall : ", distance)# Print current distance to wall
                distanceArray.append(distance) #Add distance to array for checking moveback

                #self.rc.checkMoveBack(distanceArray,self.speed)
            
            self.rc.stop_robot
            self.motion = self.where_turn()# Gets the left and right scans, compares the distances to the left and right walls
            # and outputs which way to turn based on which way has a closer wall
            turnTime = self.rc.turn(self.motion,self.speed,self.time)
            print(turnTime)
            distance = self.dist()

    def where_turn(self):
            self.d=self.rc.get_laser_full()# Get all laser scans
            l1=[]#Left side
            l2=[]#Right side
            #Math logic to split the ranged scan
            cnt=len(self.d)
            print("cnt = " + str(cnt))
            i=0
            j=cnt/2
            print("j = " + str(j))
            while(i<cnt/2):
                l1.append(self.d[i])
                i=i+1
                #print("l1 = " + str(l1))
            while(j<cnt):
                l2.append(self.d[int(j)])
                #print("l2 = " + str(l2))
                j=j+1
            #Calculate mean of each array, to see which way the robot should turn
            v1,v2=self.mean(l1,l2)
            print("sum of right= ",v1)
            print("sum of left= ",v2)

            if(v1>v2):
                self.d=None
                return "clockwise"# If left is greater then right, turn clockwise
            else:
                self.d=None
                return "counter-clockwise"# Opposite

    def mean(self,x,y):# Work out means
       x=np.asarray(x)#Numpy array
       x = x[~np.isnan(x)]# Remove NaNs
       y=np.asarray(y)
       y = y[~np.isnan(y)]
       m1=np.sum(x)
       m2=np.sum(y)
       return [m1,m2]

robot = robotLogic(speed = 2, time = 5)# Create robot object using robotLogic class
robot.move() #Get robot to move