#!/usr/bin/env python

#Imports

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

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

    def publishVel(self):
        while not self.ctrl_c:
            connections = self.velPub.get_num_connections()
            if connections > 0:
                self.velPub.publish(self.velocity)
                break
            else
            self.rate.sleep()

    def shutdownhook(self):
       # works better than the rospy.is_shutdown()
       self.ctrl_c = True

    def laser_callback(self, msg):
    self.laser_msg = msg
 
 
   def get_laser(self, pos):
       time.sleep(10)
       return self.laser_msg.ranges[pos]

    def get_laser_full(self):
       time.sleep(10)
       return self.laser_msg.ranges

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
 
       #if motion == "forward":
           #self.velocity.linear.x = speed
       #elif motion == "backward":
           #self.velocity.linear.x = - speed
 
       i = 0
       # loop to publish the velocity estimate, current_distance = velocity * (t1 - t0)
       while (i <= time):
 
           # Publish the velocity
           self.vel_publisher.publish(self.velocity)
           i += 1
           self.rate.sleep()
 
       # set velocity to zero to stop the robot
       self.stop_robot()
 
       s = "Moved robot " + motion + " for " + str(time) + " seconds"
       return s

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
           self.vel_publisher.publish(self.velocity)
           i += 1
           self.rate.sleep()
 
       # set velocity to zero to stop the robot
       self.stop_robot()
 
       s = "Turned robot " + clockwise + " for " + str(time) + " seconds"
       return s

    def where_turn(self):
       self.d=self.rc.get_laser_full()
       l1=[]
       l2=[]
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
           l2.append(self.d[j])
           #print("l2 = " + str(l2))
           j=j+1
       v1,v2=self.mean(l1,l2)
       print("sum of right= ",v1)
       print("sum of left= ",v2)
 
       if(v1>v2):
           self.d=None
           return "clockwise"
       else:
           self.d=None
           return "counter-clockwise"
 
   def mean(self,x,y):
       x=np.asarray(x)
       x = x[~np.isnan(x)]
       #print(x)
       y=np.asarray(y)
       y = y[~np.isnan(y)]
       #print(y)
       m1=np.sum(x)
       m2=np.sum(y)
       return [m1,m2]

import numpy as np

class robotLogic:
    def __init__(self,speed,time):
        self.rc = RobotControl()
        self.motion = None
        self.speed = speed
        self.time = time
        self.d = None

    def dist(self):
        return (self.rc.get_laser(360))

    def move(self):
        distance = self.dist()

        while True:
            while (distance>0.5):
                self.rc.move_straight_time(speed, time)
                distance=self.dist()
                print("Current distance from wall : ", distance

            self.rc.stop_robot
            self.motion = self.rc.where_turn
            self.rc.turn(self.motion,self.speed,self.time)
            print("turning ", self.motion)
            distance = self.dist()

robot = robotLogic(speed = 2, time = 5)
robot.mov()

            