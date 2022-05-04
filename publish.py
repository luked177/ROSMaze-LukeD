#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class Chatter:

    def __init__(self):
        rospy.init_node("chatter")
        self.publisher = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.laser_cb)

    def laser_cb(self, laser_msg):
        self.distance = 0.7
        if laser_msg.ranges[0] > 1:
            if laser_msg.ranges[0] > self.distance:
                t = Twist()
                t.linear.x = 0.2
                t.angular.z = 0.0
                self.publisher.publish(t)
            elif laser_msg.ranges[90] > self.distance:
                t = Twist()
                t.linear.x = 0.0
                t.angular.z = -0.3
                self.publisher.publish(t)
        if laser_msg.ranges[0] > self.distance and laser_msg.ranges[15] > self.distance and laser_msg.ranges[345] > self.distance:
            t = Twist()
            t.linear.x = 0.2
            t.angular.z = 0.0
            self.publisher.publish(t)
        else:
            t = Twist()
            t.linear.x = 0.0
            t.angular.z = 0.5
            self.publisher.publish(t)
                

    def run(self):
        rospy.spin()

c = Chatter()
c.run()