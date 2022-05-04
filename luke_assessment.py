#!/usr/bin/env python
 
#Imports
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time
 
# Robot Control Class - Contains all functions to control robot, no logic
class RobotControl():
 
   # Initialize robot with publishers and subscribers
   def __init__(self):
       # Create Node
       rospy.init_node('robot_control_node', anonymous=True)
       # Velocity publisher - allows the robot to move
       self.vel_publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
       # Laser subscriber - takes in the laser scan data
       self.laser_subscriber = rospy.Subscriber(
           '/scan', LaserScan, self.laser_callback)
 
       #Creates a twist variable to allow linear and angular changes to self.velocity
       self.velocity = Twist()
       #Creates a LaserScan to get laser info from self.laser_msg
       self.laser_msg = LaserScan()
       #Creates a ctrl_c variable so that when variable is false, the robot will run forever
       self.ctrl_c = False
       #Sets rate to send messages to robot (10hz)
       self.rate = rospy.Rate(10)
       #When shutting down, run the shutdownhook function
       rospy.on_shutdown(self.shutdownhook)
 
   #Function to publish velocity to robot
   def publish_once_in_velocity(self):
       #While the robot is running
       while not self.ctrl_c:
           #Checks if anything is subscribing to the publisher
           connections = self.vel_publisher.get_num_connections()
           if connections > 0:
               self.vel_publisher.publish(self.velocity)
               break
           else:
               self.rate.sleep()
 
   def shutdownhook(self):
       # works better than the rospy.is_shutdown()
       self.ctrl_c = True
 
   def laser_callback(self, msg):
       self.laser_msg = msg
 
 
   def get_laser(self, pos):
       time.sleep(10)
       return self.laser_msg.ranges[pos]
 
 
   def get_front_laser(self):
       time.sleep(10)
       return self.laser_msg.ranges[90]
 
   def get_laser_full(self):
       time.sleep(10)
       return self.laser_msg.ranges
 
   def stop_robot(self):
       #rospy.loginfo("shutdown time! Stop the robot")
       self.velocity.linear.x = 0.0
       self.velocity.angular.z = 0.0
       self.publish_once_in_velocity()
 
   def move_straight(self):
 
       # Initilize velocities
       self.velocity.linear.x = 0.5
       self.velocity.linear.y = 0
       self.velocity.linear.z = 0
       self.velocity.angular.x = 0
       self.velocity.angular.y = 0
       self.velocity.angular.z = 0
 
       # Publish the velocity
       self.publish_once_in_velocity()
 
   def move_straight_time(self, motion, speed, time):
 
       # Initilize velocities
       self.velocity.linear.y = 0
       self.velocity.linear.z = 0
       self.velocity.angular.x = 0
       self.velocity.angular.y = 0
       self.velocity.angular.z = 0
 
       if motion == "forward":
           self.velocity.linear.x = speed
       elif motion == "backward":
           self.velocity.linear.x = - speed
 
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
 
 
if __name__ == '__main__':
   #rospy.init_node('robot_control_node', anonymous=True)
   robotcontrol_object = RobotControl()
   try:
       robotcontrol_object.move_straight()
 
   except rospy.ROSInterruptException:
       pass
 
import numpy as np
 
class Project:
 
   def __init__(self,speed,time):
       self.rc=RobotControl()
       self.motion=None
       self.speed=speed
       self.time=time
       self.d=None
   def dist(self):
       return(self.rc.get_laser(90))
   def mov(self):
       distance=self.dist()
       print(distance)
       while True:
           while (distance>1):
               self.rc.move_straight()
               distance=self.dist()
               print("Current distance from wall : ", distance)
 
           self.rc.stop_robot()
           self.motion=self.where_turn()
           self.rc.turn(self.motion,self.speed,self.time)
           print("turning ", self.motion)
           distance=self.dist()
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
 
robot=Project(speed=2,time=5)
robot.mov()